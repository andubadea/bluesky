import bluesky as bs
import numpy as np

from bluesky.traffic.asas import ConflictResolution
from bluesky.tools.aero import kts, ft
from bluesky.tools.geo import kwikdist

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'HALTSBCR',
        'plugin_type': 'sim'
    }
    return config

class HaltSBCR(ConflictResolution):
    def __init__(self):
        super().__init__()
        self.enable_altitude_CR = False
        self.cruiselayerdiff = 50 * ft
        self.frnt_tol = 20 #deg
        self.dist_tol = 80
        self.rpz = bs.traf.cd.rpz_def * 1.1 # For good measure
        self.cruise_spd = bs.traf.m22delay.cruise_spd
        
    
    def resolve(self, conf, ownship, intruder):
        """Velocity and altitude-only solving.
        """
        # Make a copy of traffic data, track and ground speed
        newgscapped = np.copy(ownship.gs)
        newalt      = np.copy(ownship.alt)
        
        # Iterate over aircraft in conflict
        for idx1 in np.argwhere(conf.inconf).flatten():
            # Get the conflict pairs
            idx_pairs = self.get_pairs(conf, ownship, intruder, idx1)
            # Find the new speed and altitude for this aircraft
            gs_new, alt_new = self.M22CR(conf, ownship, intruder, idx1, idx_pairs)
            # Apply these
            newgscapped[idx1] = gs_new
            newalt[idx1]      = alt_new  
            
        # Apply the autopilot things for VS and track
        newvs          = ownship.ap.vs
        newtrack       = ownship.ap.trk
        
        return newtrack, newgscapped, newvs, newalt
    
    def M22CR(self, conf, ownship, intruder, idx1, idx_pairs):
        """This function provides a velocity and altitude for the ownship.
        """
        # Get the velocity of the ownship
        v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1]])# [m/s]
        # Get the distance to other aircraft
        dist2others = conf.dist_mat[idx1]
        # Get the lookahead time
        t = bs.settings.asas_dtlookahead
        # Get the separation distance
        self.hpz = conf.hpz[idx1]
        
        # For each intruder, we want to collect all the information and then make a decision based on all.
        # Initialise per-intruder variables
        n_intr = len(idx_pairs)
        recd_speed= [ownship.ap.tas[idx1]] * n_intr # Speed in metres, None means maintain current speed
        should_hold_altitude = [None] * n_intr #True if hold otherwise False, None means indifferent
        should_ascend = [None] * n_intr # True or False, None means indifferent
        should_descend = [None] * n_intr #True or False, None means indifferent
        los_list = [False] * n_intr
        
        # Iterate over all intruders
        for i, idx_pair in enumerate(idx_pairs):
            # Get the index of the intruder
            idx2 = intruder.id.index(conf.confpairs[idx_pair][1])
            # Get the velocity of the intruder
            v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
            # Extract conflict bearing and distance information
            qdr = conf.qdr[idx_pair]
            dist= conf.dist[idx_pair]
            # Find the bearing of the intruder with respect to where we are heading
            qdr_intruder = ((qdr - ownship.trk[idx1]) + 180) % 360 - 180  
            qdr_difference = ((qdr - ownship.trk[idx2]) + 180) % 360 - 180
            # Check whether the intruder is in front or in the back
            intr_in_front = (-self.frnt_tol < qdr_intruder < self.frnt_tol)
            intr_in_back = (qdr_intruder < -180 + self.frnt_tol or 180 - self.frnt_tol < qdr_intruder)
            intr_front_aligned = intr_in_front and -self.frnt_tol < qdr_difference < self.frnt_tol
            head_on = (abs((np.degrees(self.angle(v1, v2)))) > (180-self.frnt_tol))
            # Determine if intruder is close in altitude:
            alt_ok = ((abs(ownship.alt[idx1] - intruder.alt[idx2])) > self.hpz)
            # Determine if we have a LOS
            los = (dist <= self.rpz)
            # Does the priority check out? If true, then ownship has greater priority
            if intr_in_back:
                # Ownship has priority if it is front of the intruder
                own_has_priority = True
            elif intr_front_aligned:
                # Ownship doesn't have priority if intruder is in front.
                own_has_priority = False
            else:
                # Determine the priority based on proximity to intersection
                own_has_priority = self.check_prio(conf, ownship, intruder, idx1, idx2)
            
            # First of all, if we have a loss of separation
            if los:
                # Aircraft are either in a true LOS or just on top of each other. 
                # For both, we do the same thing: kill their vs, make lower priority one to go slow
                if own_has_priority:
                    # We have priority, continue our way
                    recd_speed[i] = self.cruise_spd
                    should_hold_altitude[i] = True
                    should_ascend[i] = False
                    should_descend[i] = False
                    continue
                else:
                    # We don't have priority, we go slow.
                    recd_speed[i] = 5*kts
                    should_hold_altitude[i] = True
                    should_ascend[i] = False
                    should_descend[i] = False
                    continue
                
            elif intr_in_front:
                # Intruder is in front. If they are also aligned with us and a bit too close, we slow down to create distance.
                if dist < self.dist_tol and intr_front_aligned:
                    # Woah there slow down
                    recd_speed[i] = 10*kts
                elif dist > self.dist_tol and intr_front_aligned:
                    # In this case, we just match the speed, as there is enough distance between em.
                    recd_speed[i] = bs.traf.gs[idx2]
                elif head_on:
                    # This shouldn't be a full head-on, but they are definitely moving towards each other.
                    # One will probably turn, so make the other stop.
                    if own_has_priority:
                        recd_speed[i] = self.cruise_spd
                    else:
                        if bs.traf.id[idx1] == 'D928' and bs.traf.id[idx2] == 'D968':
                            print('hi')
                        # Just stop
                        recd_speed[i] = 0*kts
                elif not own_has_priority:
                    # We will need to halt
                    recd_speed[i] = self.get_halt_speed(conf, ownship, intruder, idx1, idx2)
                    if bs.traf.id[idx1] == 'D928' and bs.traf.id[idx2] == 'D968':
                            print(recd_speed[i], recd_speed)
                else:
                    pass
                    
                if self.enable_altitude_CR:
                    # We can potentially perform an overtake manoeuver. Check if we can ascend.
                    can_ascend, _ = self.ac_above_below_check(conf, ownship, intruder, idx1, dist2others)
                    
                    if can_ascend and not alt_ok:
                        # Only ascend if we are on the same level
                        should_hold_altitude[i] = False
                        should_ascend[i] = True
                        should_descend[i] = False
                        continue
                    else:
                        # We only overtake by ascending. maintain altitude.
                        should_hold_altitude[i] = True
                        should_ascend[i] = False
                        should_descend[i] = False
                        continue
                else:
                    # Maintain altitude.
                    should_hold_altitude[i] = True
                    should_ascend[i] = False
                    should_descend[i] = False
                    continue
                
            elif intr_in_back:
                # We can ignore this  intruder, they're the ones that need to solve.
                recd_speed[i] = self.cruise_spd
                continue
            
            elif own_has_priority:
                # We can ignore this intruder.
                recd_speed[i] = self.cruise_spd
                continue
            else:
                # We will need to halt
                recd_speed[i] = self.get_halt_speed(conf, ownship, intruder, idx1, idx2)
                should_hold_altitude[i] = True
                should_ascend[i] = False
                should_descend[i] = False
                continue
            
        # We're done with the for loop, we have some decisions to make.
        # First of all, the new velocity is the smallest one in the speed list.
        gs_new = min(min(recd_speed), bs.traf.ap.tas[idx1])
        # Let's work on the altitude
        if np.any(should_hold_altitude):
            # Hold altitude
            alt_new = bs.traf.alt[idx1]
        elif np.any(should_ascend):
            # We should ascend a layer if we aren't already ascending
            if abs(bs.traf.vs[idx1]) > 0:
                # Keep this resolution altitude
                alt_new = bs.traf.cr.alt[idx1]
            else:
                # Go up a layer
                alt_new = self.get_layer_above(idx1)
        else:
            # Maintain altitude
            alt_new = bs.traf.alt[idx1]
        return gs_new, alt_new
    
    def get_halt_speed(self, conf, ownship, intruder, idx1, idx2):
        """Returns the velocity of the ownship (idx1) for the conflict with this intruder (idx2).
        """
        cruise_spd = bs.traf.TrafficHandler.spd
        # First, find the intersection between the trajectories of the two aircraft
        exists, int_lat, int_lon = self.geo_intersec_point(
            bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.hdg[idx1],
            bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.hdg[idx2]
        )
        
        if not exists:
            # Try again with flipped intruder bearing
            if bs.traf.hdg[idx2] < 180:
                exists, int_lat, int_lon = self.geo_intersec_point(
                    bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.hdg[idx1],
                    bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.hdg[idx2]+180
                )
            else:
                exists, int_lat, int_lon = self.geo_intersec_point(
                    bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.hdg[idx1],
                    bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.hdg[idx2]-180
                )
                
            if not exists:
                # Still doesn't exist, weird
                return bs.traf.ap.tas[idx1]
        
        # Get the distance to the intersection
        int_dist = kwikdist(bs.traf.lat[idx1], bs.traf.lon[idx1],
                            int_lat, int_lon)
        
        # We're supposed to stop at rpz*1.1
        stopping_point_dist = int_dist - conf.rpz_def * 1.1
        distance_to_start_stopping = self.distaccel(cruise_spd, 0, idx1)*1.2
        
        if stopping_point_dist < distance_to_start_stopping:
            return 0
        else:
            return bs.traf.ap.tas[idx1]
    
    def geo_intersec_point(self, lata, lona, brnga, latb, lonb, brngb):
        """
            This function calculate the intersection point if exists of two points\r\n
                given their coordinates (latitude and longitude in DD format) and bearings in degrees.\r\n
            x1      (float)     : Longitude of the first point (DD format).\r\n
            y1      (float)     : Latitude of the first point (DD format).\r\n
            brng1   (float)     : Bearing of the first point (degrees format).\r\n
            x1      (float)     : Longitude of the second point (DD format).\r\n
            y1      (float)     : Latitude of the second point (DD format).\r\n
            brng1   (float)     : Bearing of the second point (degrees format).\r\n
            return :\r\n
            error   (Bool)      : True if an error is detected.\r\n
            result  (dict, str) :  if error is True, return error message.
                                if error is False, return coordinates of intersection\r
                                point in a dict struct (DD format).\r\n
        """

        # Convert to radians (sin, np.cos and tan works with radians)
        lon1 = np.deg2rad(lona)
        lat1 = np.deg2rad(lata)
        brng1 = np.deg2rad(brnga)

        lon2 = np.deg2rad(lonb)
        lat2 = np.deg2rad(latb)
        brng2 = np.deg2rad(brngb)

        # Calculate the angular distance between point 1 and point 2
        dlon = lon2 - lon1 # Distance between longitude points
        dlat = lat2 - lat1 # Distance between latitude points

        # Great-circle distance between point 1 and point 2
        haversine = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2

        # angular distance between point 1 and point 2
        ang_dist_1_2 = 2 * np.arcsin(np.sqrt(haversine))

        # Calculate the initial and final bearings between point 1 and point 2
        initial_bearing = np.arccos((np.sin(lat2) - np.sin(lat1) * np.cos(ang_dist_1_2)) / (np.sin(ang_dist_1_2) * np.cos(lat1)))
        final_bearing = np.arccos((np.sin(lat1) - np.sin(lat2) * np.cos(ang_dist_1_2)) / (np.sin(ang_dist_1_2) * np.cos(lat2)))

        # Adjust initial and final bearings
        if np.sin(lon2 - lon1) > 0:
            bearing_1_2 = initial_bearing
            bearing_2_1 = (2 * np.pi) - final_bearing

        else:
            bearing_1_2 = (2 * np.pi) - initial_bearing
            bearing_2_1 = final_bearing


        # Angles between different points
        ang_1 = brng1 - bearing_1_2    # angle p2<--p1-->p3
        ang_2 = bearing_2_1 - brng2    # angle p1<--p2-->p3

        # Check for ambiguous or inifite intersection
        # infinite intersections   
        if np.sin(ang_1) == 0 and np.sin(ang_2) == 0:
            return  False, 0,0

        # ambiguous intersection (antipodal/360°)
        if np.sin(ang_1) * np.sin(ang_2) < 0:
            return  False, 0,0

        # if no errors, calculte intersection point angle, latitude and longitude
        ang_3 = np.arccos(-np.cos(ang_1) * np.cos(ang_2) + np.sin(ang_1) * np.sin(ang_2) * np.cos(ang_dist_1_2))    # angle p1<--p3-->p2

        # angular distance between point 1 and intersection point (point 3)
        ang_dist_1_3 = np.arctan2(np.sin(ang_dist_1_2) * np.sin(ang_1) * np.sin(ang_2), np.cos(ang_2) + np.cos(ang_1) * np.cos(ang_3))

        # Latitude of point 3
        lat3 = np.arcsin(np.sin(lat1) * np.cos(ang_dist_1_3) + np.cos(lat1) * np.sin(ang_dist_1_3) * np.cos(brng1))

        # Longitude of point 3
        delta_long_1_3 = np.arctan2(np.sin(brng1) * np.sin(ang_dist_1_3) * np.cos(lat1), np.cos(ang_dist_1_3) - np.sin(lat1) * np.sin(lat3))
        lon3 = lon1 + delta_long_1_3

        # Convert to degrees
        lon3 = np.rad2deg(lon3)
        lat3 = np.rad2deg(lat3)

        # Return error flag as False and intersection points longitude and latitude
        return True, lat3, lon3
    
    def get_layer_above(self, idx):
        '''Get the layer above the current layer of the aircraft.'''
        possible_layers = np.arange(self.cruiselayerdiff, 501, self.cruiselayerdiff)
        
        layer_index = possible_layers.index(min(possible_layers, key = lambda x: abs(x-bs.traf.alt[idx]/ft)))
        if layer_index + 1 < len(possible_layers)-1:
            # We can hop
            return possible_layers[layer_index+1] * ft
        else:
            # We can't hop
            return possible_layers[layer_index] * ft

    def get_pairs(self, conf, ownship, intruder, idx):
        '''Returns the indices of conflict pairs that involve aircraft idx
        '''
        idx_pairs = np.array([], dtype = int)
        for idx_pair, pair in enumerate(conf.confpairs):
            if (ownship.id[idx] == pair[0]):
                idx_pairs = np.append(idx_pairs, idx_pair)
        return idx_pairs
    
    def check_prio(self, conf, ownship, intruder, idx1, idx2):
        """Returns true if ownship has priority, and false if it doesn't. 
        """
        # The aircraft closest to the intersection point between the headings of the
        # two aircraft has priority. Basically, whoever has a greater bearing with respect
        # to the other.
        qdr_1_wrt_2 = ((conf.qdr_mat[idx2, idx1] - ownship.trk[idx2]) + 180) % 360 - 180
        qdr_2_wrt_1 = ((conf.qdr_mat[idx1, idx2] - ownship.trk[idx1]) + 180) % 360 - 180 
        
        if abs(qdr_1_wrt_2) < 90 and abs(qdr_2_wrt_1) > 90:
            return True
        
        if abs(qdr_1_wrt_2) < abs(qdr_2_wrt_1):
            # Ownship is closer to physical intersection
            return True
        
        return False
    
    def check_prio_all(self, conf, ownship, intruder, idx1):
        """Returns True if ownship has priority over all of its intruders.
        """
        # Get all the idx of the intruders for this aircraft.
        idx_pairs = self.get_pairs(conf, ownship, intruder, idx1)
        for idx_pair in idx_pairs:
            idx2 = intruder.id.index(conf.confpairs[idx_pair][1])
            # Check if this intruder is in the back
            qdr = bs.traf.cd.qdr_mat[idx1, idx2]
            qdr_intruder = ((qdr - ownship.trk[idx1]) + 180) % 360 - 180  
            intr_in_back = (qdr_intruder < -180 + self.frnt_tol or 180 - self.frnt_tol < qdr_intruder)
            # Also check the other priority rules
            other_prio = self.check_prio(conf, ownship, intruder, idx1, idx2)
            if not intr_in_back or not other_prio:
                # We don't have prio over all other aircraft
                return False
        
        # If we reach this point, we have priority over all intruders
        return True
            
    def ac_above_below_check(self, conf, ownship, intruder, idx1, dist2others):
        """This function checks if the aircraft can ascend or descend in function of what
        other aircraft are around it.
        """
        can_ascend = True
        can_descend = True
        # Get aircraft that are close
        is_close = np.where(dist2others < self.rpz * 2)[0]
        # Get the vertical distance for these aircraft
        vertical_dist = ownship.alt[idx1] - intruder.alt[is_close]
        # Check if any is smaller than cruise layer difference
        cruise_diff_ascend = np.logical_and(0 > vertical_dist, vertical_dist > (-self.cruiselayerdiff * 1.1))
        cruise_diff_descend = np.logical_and(0 < vertical_dist, vertical_dist < (self.cruiselayerdiff * 1.1))
        # Check also if any is smaller than conf.hpz
        conf_diff = np.abs(vertical_dist) > conf.hpz[idx1]
        # Do the or operation on these two
        dealbreaker_ascend = np.logical_or(cruise_diff_ascend, conf_diff) 
        dealbreaker_descend = np.logical_or(cruise_diff_descend, conf_diff)
            
        return can_ascend, can_descend
    
    def norm_sq(self, x):
        return np.dot(x, x)
    
    def norm(self,x):
        return np.sqrt(self.norm_sq(x))
    
    def normalized(self, x):
        l = self.norm_sq(x)
        assert l > 0, (x, l)
        return x / np.sqrt(l)
    
    def angle(self, a, b):
        ''' Find non-directional angle between vector a and b'''
        unit_a = a / np.linalg.norm(a)
        unit_b = b / np.linalg.norm(b)
        return np.arccos(np.clip(np.dot(unit_a, unit_b), -1.0, 1.0))
    
    # We want to override the HDGACTIVE flag for aircraft to always follow the heading from AP
    @property
    def hdgactive(self):
        ''' Return a boolean array sized according to the number of aircraft
            with True for all elements where heading is currently controlled by
            the conflict resolution algorithm.
        '''
        return np.array([False] * len(self.active))
    
    @property
    def vsactive(self):
        ''' Return a boolean array sized according to the number of aircraft
            with True for all elements where heading is currently controlled by
            the conflict resolution algorithm.
        '''
        return np.array([False] * len(self.active))
    
    def distaccel(self,v0,v1,idx):
        """Calculate distance travelled during acceleration/deceleration
        v0 = start speed, v1 = endspeed, axabs = magnitude of accel/decel
        accel/decel is detemremind by sign of v1-v0
        axabs is acceleration/deceleration of which absolute value will be used
        solve for x: x = vo*t + 1/2*a*t*t    v = v0 + a*t """
        axabs = bs.traf.perf.axmax[idx]
        return 0.5*np.abs(v1*v1-v0*v0)/np.maximum(.001,np.abs(axabs))
    
    def resumenav(self, conf, ownship, intruder):
        '''
            Decide for each aircraft in the conflict list whether the ASAS
            should be followed or not, based on if the aircraft pairs passed
            their CPA AND if ownship is a certain distance away from the intruding
            aircraft.
        '''
        # Add new conflicts to resopairs and confpairs_all and new losses to lospairs_all
        self.resopairs.update(conf.confpairs)

        # Conflict pairs to be deleted
        delpairs = set()
        changeactive = dict()

        # Look at all conflicts, also the ones that are solved but CPA is yet to come
        for conflict in self.resopairs:
            idx1, idx2 = bs.traf.id2idx(conflict)
            # If the ownship aircraft is deleted remove its conflict from the list
            if idx1 < 0:
                delpairs.add(conflict)
                continue
            

            if idx2 >= 0:
                # Distance vector using flat earth approximation
                re = 6371000.
                dist = re * np.array([np.radians(intruder.lon[idx2] - ownship.lon[idx1]) *
                                      np.cos(0.5 * np.radians(intruder.lat[idx2] +
                                                              ownship.lat[idx1])),
                                      np.radians(intruder.lat[idx2] - ownship.lat[idx1])])

                # Relative velocity vector
                vrel = np.array([intruder.gseast[idx2] - ownship.gseast[idx1],
                                 intruder.gsnorth[idx2] - ownship.gsnorth[idx1]])

                # Check if conflict is past CPA
                past_cpa = np.dot(dist, vrel) > 0.0

                # Also check the distance and altitude between the two aircraft.
                distance = self.norm(dist)
                # We want enough distance between aircraft
                dist_ok = (distance > self.rpz * 1.2) 
                # We also want enough altitude
                alt_ok = abs(ownship.alt[idx1]-intruder.alt[idx2]) >= (2*self.cruiselayerdiff)
                # hor_los:
                # Aircraft should continue to resolve until there is no horizontal
                # LOS. This is particularly relevant when vertical resolutions
                # are used.
                hdist = np.linalg.norm(dist)
                hor_los = hdist < conf.rpz[idx1]

                # Bouncing conflicts:
                # If two aircraft are getting in and out of conflict continously,
                # then they it is a bouncing conflict. ASAS should stay active until
                # the bouncing stops.
                is_bouncing = \
                    abs(ownship.trk[idx1] - intruder.trk[idx2]) < self.frnt_tol and \
                    hdist < conf.rpz[idx1] * self.resofach
                    
                # Group some checks together
                # The autopilot is truly ok if both the vertical separation and the speed
                # it wants to apply are ok
                # ap_ok = ap_spd_ok and alt_ok 
                # Navigation is ok if 
                # - Altitude is ok and vertical speed is 0 OR 
                # - Distance between the aircraft is ok OR 
                # - The autopilot is ok AND
                # - The autopilot vertical speed is ok
                nav_ok = (alt_ok or dist_ok)
                conf_ok = (past_cpa and not hor_los and not is_bouncing) or alt_ok

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not (nav_ok and conf_ok)):
                # Enable ASAS for this aircraft
                changeactive[idx1] = True
                # We also need to check if this aircraft needs to be doing a turn, aka, if the
                # autopilot speed is lower than the CR speed. If it is, then we need to update
                # the speed to that value. Thus, either the conflict is still ok and CD won't be
                # triggered again, or a new conflict will be triggered and CR will take over again.
                if self.tas[idx1] > bs.traf.ap.tas[idx1]:
                    self.tas[idx1] = bs.traf.ap.tas[idx1]
                    
                # However, if we have priority, we can resume normal operations
                if self.check_prio_all(conf, ownship, intruder, idx1):
                    # Set the speed to the autopilot one
                    self.tas[idx1] = bs.traf.ap.tas[idx1]
                
            else:
                # Switch ASAS off for ownship if there are no other conflicts
                # that this aircraft is involved in.
                changeactive[idx1] = changeactive.get(idx1, False)
                # If conflict is solved, remove it from the resopairs list
                delpairs.add(conflict)
                    
        for idx, active in changeactive.items():
            # Loop a second time: this is to avoid that ASAS resolution is
            # turned off for an aircraft that is involved simultaneously in
            # multiple conflicts, where the first, but not all conflicts are
            # resolved.
            self.active[idx] = active
            if not active:
                # Waypoint recovery after conflict: Find the next active waypoint
                # and send the aircraft to that waypoint.
                iwpid = bs.traf.ap.route[idx].iactwp
                if iwpid != -1:  # To avoid problems if there are no waypoints
                    bs.traf.ap.route[idx].direct(
                        idx, bs.traf.ap.route[idx].wpname[iwpid])

        # Remove pairs from the list that are past CPA or have deleted aircraft
        self.resopairs -= delpairs