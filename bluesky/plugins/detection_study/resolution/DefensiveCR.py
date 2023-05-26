import numpy as np
import bluesky as bs
import copy
from bluesky.traffic.asas import ConflictResolution
from bluesky.core import Entity
from bluesky.tools.aero import kts
from shapely.geometry import Point


def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'DEFENSIVECR',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config


class DefensiveCR(ConflictResolution):
    def resolve(self, conf, ownship, intruder):
        # Some constants
        turn_time  = 3 #seconds
        time_margin = 5 #seconds
        frnt_tol = 20
        # Get all the values from CD that we would need
        confpairs = conf.confpairs # Pair IDs in conflict
        intent_geom = conf.intent_geom # Linestring of aircraft intent per pair
        dist_to_int = conf.dist_to_int # Distance to intent intersections per pair
        vel_rel_int = conf.vel_rel_int # Velocity relative to intent intersection per pair
        num_turns = conf.num_turns # Number of turns per pair
        mean_turn_angle = conf.mean_turn_angle # Mean turn angle per pair
        qdr_mat = conf.qdr_mat # QDR for all aircraft
        dist_mat = conf.dist_mat # Distance for all aircraft
        los_detected = conf.los_detected # If a LOS was detected or not
        
        # Copies of aircraft autopilot stuff
        newgs       = np.copy(ownship.ap.tas)
        newvs       = np.copy(ownship.ap.vs)
        newalt      = np.copy(ownship.ap.alt)
        newtrack    = np.copy(ownship.ap.trk)
        
        for pair_idx, pair in enumerate(confpairs):
            #print(f'--------- {pair} ---------')
            # Get the aircraft IDs
            ownship_id = pair[0]
            intruder_id = pair[1]
            
            # Get the aircraft IDX
            ownship_idx = bs.traf.id.index(pair[0])
            intruder_idx = bs.traf.id.index(pair[1])
            
            # We first need to determine what type of conflict this is, as in whether this is a normal
            # intersection or a back-to-front conflict or a state-based one. As this is non-cooperative, 
            # either the aircraft in the back solves, or the aircraft with a lower priority. Aircraft with 
            # lower ACID numbers have higher priority as they have been flying for longer. 
            
            # First, check and handle state-based conflicts
            if intent_geom[pair_idx][0] == 'statebased':
                # This is a state-based conflict, so we need to do some special things
                # First, check if the intruder is in the front
                qdr = qdr_mat[ownship_idx, intruder_idx]
                rel_qdr_intruder = ((qdr - ownship.trk[ownship_idx]) + 180) % 360 - 180
                rel_trk_intruder = ((qdr - ownship.trk[intruder_idx]) + 180) % 360 - 180
                intruder_in_front = -frnt_tol < rel_qdr_intruder < frnt_tol
                intruder_in_back = (rel_qdr_intruder < -180 + frnt_tol or 180 - frnt_tol < rel_qdr_intruder)
                intruder_aligned_front = intruder_in_front and -frnt_tol < rel_trk_intruder < frnt_tol
                
                if intruder_aligned_front:
                    # Match the speed of the intruder
                    # First, the speed we want to set is equal to the speed of the intruder
                    gs_to_set = bs.traf.gs[intruder_idx]
                    if gs_to_set < bs.traf.gs[ownship_idx]:
                        # Speed we want to set is smaller, so set it
                        newgs[ownship_idx] = gs_to_set
                    # We're done with this pair, just return
                    #print('Intruder is in front.')
                    continue
                elif intruder_in_back:
                    # We do nothing
                    continue
                elif intruder_in_front:
                    # We also slow down
                    newgs[ownship_idx] = 0
                    continue
                else:
                    # The other aircraft will slow down
                    continue
            # Now for the defensive problems.
            # We basically want to handle each situation one by one, if there are several.
            for i, pair_intent_geom in enumerate(intent_geom[pair_idx]):
                # Geometry can be two things: If it's "none", then we have a back-to-front
                # conflict 
                if pair_intent_geom is None:
                    # We have a back to back conflict
                    # We can determine whether the ownship is in the back or not from the velocity relative to the
                    # intersection. For this, the velocity and distance of the intruder needs to be 0 and veloity of 
                    # the ownship is greater than 0.
                    ownship_in_back =  (dist_to_int[pair_idx][0] != 0) and \
                                    (dist_to_int[pair_idx][1] == 0)
                                    
                    intruder_in_back = (dist_to_int[pair_idx][1] != 0) and \
                                    (dist_to_int[pair_idx][0] == 0) 
                    
                    if intruder_in_back:
                        # Then we must do nothing, as we have priority
                        #print('Intruder in the back.')
                        continue
                                    
                    if ownship_in_back:
                        # This is easy to solve, we just velocity match the aircraft in front.
                        # We might have already changed the speed for this aircraft for other conflicts. So only
                        # change the speed if the one we're about to set is smaller than the one that is already
                        # set.
                        # First, the speed we want to set is equal to the speed of the intruder
                        gs_to_set = bs.traf.gs[intruder_idx]
                        if gs_to_set < bs.traf.gs[ownship_idx]:
                            # Speed we want to set is smaller, so set it
                            newgs[ownship_idx] = gs_to_set
                            
                        # We're done with this pair, just return
                        #print('Intruder is in front.')
                        continue
                elif isinstance(pair_intent_geom, Point):
                    # If we're here, then we must have a classical intersection conflict, and we should solve it
                    # by making the aircraft that has less priority slow down such that the other aircraft
                    # has time to clear the intersection.
                                    
                    # Different prio: closest to intersection gets priority
                    ownship_has_prio = dist_to_int[pair_idx][0] < dist_to_int[pair_idx][1]
                    
                    if ownship_has_prio:
                        # This aircraft doesn't need to do anything for this intruder as it has priority
                        #print('Intruder has lower priority.')
                        continue
                    
                    # Okay time to make the ownship slow down by an appropriate amount. For this, we need to
                    # take into account how much time does the other aircraft have until it reaches the
                    # intersection point. We will then have to allow them to pass while not exactly coming
                    # to a complete stop. 
                    # First, check if we'll be at the intersection point way faster than the other aircraft
                    if vel_rel_int[pair_idx][0] > 0:
                        time_to_int_ownship = dist_to_int[pair_idx][0] / vel_rel_int[pair_idx][0]
                    else:
                        # Aircraft is standing still, so set a large number for this
                        time_to_int_ownship = 999
                        
                    if vel_rel_int[pair_idx][1] > 0:
                        time_to_int_intruder = dist_to_int[pair_idx][1] / vel_rel_int[pair_idx][1]
                    else:
                        # Aircraft is standing still, so set a large number for this
                        time_to_int_intruder = 999
                    
                    # We might also want to add some time to these for every single turn
                    time_to_int_ownship += turn_time * num_turns[pair_idx][0]
                    time_to_int_intruder+= turn_time * num_turns[pair_idx][1]
                    
                    # Now, if we get to the intersection faster than 3 seconds, we just continue.
                    if time_to_int_intruder - time_to_int_ownship > time_margin:
                        #print('Faster to intersection.')
                        continue
                    else:
                        # Let's just wait for the aircraft to pass
                        #print('Going slow.')
                        newgs[ownship_idx] = 0
                        continue
        
        return newtrack, newgs, newvs, newalt
    
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
    
    @property
    def altactive(self):
        ''' Return a boolean array sized according to the number of aircraft
            with True for all elements where heading is currently controlled by
            the conflict resolution algorithm.
        '''
        return np.array([False] * len(self.active))
    
    # Some helper functions
    def norm_sq(self, x):
        return np.dot(x, x)
    
    def norm(self,x):
        return np.sqrt(self.norm_sq(x))
    
    # Need to overwrite resumenav
    def resumenav(self, conf, ownship, intruder):
        '''
            Decide for each aircraft in the conflict list whether the ASAS
            should be followed or not, based on if the aircraft pairs passed
            their CPA.
        '''
        # Add new conflicts to resopairs and confpairs_all and new losses to lospairs_all
        print(conf.confpairs)
        self.resopairs.update(conf.confpairs)

        # Conflict pairs to be deleted
        delpairs = set()
        changeactive = dict()

        # smallest relative angle between vectors of heading a and b
        def anglediff(a, b):
            d = a - b
            if d > 180:
                return anglediff(a, b + 360)
            elif d < -180:
                return anglediff(a + 360, b)
            else:
                return d
            

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
                
                # Also check the distance between aircraft
                distance = self.norm(dist)
                # We want enough distance between aircraft
                dist_ok = (distance > 2*bs.traf.cd.rpz_def) 

                rpz = np.max(conf.rpz[[idx1, idx2]])
                # hor_los:
                # Aircraft should continue to resolve until there is no horizontal
                # LOS. This is particularly relevant when vertical resolutions
                # are used.
                hdist = np.linalg.norm(dist)
                hor_los = hdist < rpz

                # Bouncing conflicts:
                # If two aircraft are getting in and out of conflict continously,
                # then they it is a bouncing conflict. ASAS should stay active until
                # the bouncing stops.
                is_bouncing = \
                    abs(anglediff(ownship.trk[idx1], intruder.trk[idx2])) < 30.0 and \
                    hdist < rpz * self.resofach

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not past_cpa or hor_los or is_bouncing or dist_ok):
                # Enable ASAS for this aircraft
                changeactive[idx1] = True
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
                iwpid = bs.traf.ap.route[idx].findact(idx)
                if iwpid != -1:  # To avoid problems if there are no waypoints
                    bs.traf.ap.route[idx].direct(
                        idx, bs.traf.ap.route[idx].wpname[iwpid])

        # Remove pairs from the list that are past CPA or have deleted aircraft
        self.resopairs -= delpairs