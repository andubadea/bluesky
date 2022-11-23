# SpeedBasedProj should be used as the resolution method when using ProjectedBased as CD

from bluesky.traffic.asas import ConflictResolution
import bluesky as bs
import numpy as np
from bluesky import core
from bluesky import stack
from bluesky.traffic.asas import ConflictResolution
from shapely.geometry import Point, LineString, MultiPoint
from shapely.geometry.polygon import Polygon
from shapely.ops import cascaded_union, nearest_points
from shapely.affinity import translate
from bluesky.tools.geo import kwikdist, kwikqdrdist, latlondist, qdrdist
from bluesky.tools.aero import nm, ft, kts
import bluesky as bs
import numpy as np
import matplotlib.pyplot as plt

# set aircraft layer hopping to True (controlled from streets.py)
hopping = True

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SPEEDBASEDPROJ',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class SpeedBasedProj(ConflictResolution):
    def __init__(self):
        super().__init__()
        self.layer_height = 30 * ft
        self.cruiselayerdiff = self.layer_height * 3
        self.frnt_tol = 20 # Degrees
        self.rpz = 40
        
        self.heading_based = False
    
        with self.settrafarrays():
            self.in_headon = []
            self.stuck = np.array([], dtype = bool)
            self.dest_lat = np.array([])
            self.dest_lon = np.array([])
            self.altactivearr = np.array([], dtype = bool)
        
    def resolve(self, conf, ownship, intruder):
        '''We want to only solve in the velocity direction while still following the heading
        given by the autopilot. For each ownship, it calculates the minimum or maximum velocity
        needed to avoid all intruders. It then applies the solution closest to the current velocity.
        If there is no solution, it should then apply speed 0 by default, and the aircraft stops.'''
             
        # Make a copy of traffic data, track and ground speed
        newgscapped = np.copy(ownship.gs)
        newvs       = np.copy(ownship.vs)
        newalt      = np.copy(ownship.alt)
        newtrack    = np.copy(ownship.trk)
        
        # idx1 is of the ownship, idx2 is of the intruder
        # Iterate over aircraft in conflict
        for idx1 in np.argwhere(conf.inconf).flatten():     
            # Skip if rogue aircraft
            if ownship.id[idx1][0] == 'R':
                continue       
            # Find the pairs in which IDX is involved in a conflict
            idx_pairs = self.pairs(conf, ownship, intruder, idx1)
            # We're doing this because we want to solve for ALL intruders, not only pairwise
            # Find solution for aircraft 'idx'
            gs_new, alt_new, track_new = self.SpeedBasedProj(conf, ownship, intruder, idx1, idx_pairs)
            
            # Write the new velocity of aircraft 'idx' to traffic data
            newgscapped[idx1] = gs_new
            newalt[idx1]      = alt_new  
            newtrack[idx1] = track_new
        
        # Speed based, and 2D, for now.
        newvs          = ownship.ap.vs

        return newtrack, newgscapped, newvs, newalt
    
    def SpeedBasedProj(self, conf, ownship, intruder, idx1, idx_pairs):
        # The resolving function is structured as follows:
        # We first do some preliminary checks: 
        #       -whether we can ascend or descend 
        #       -if we're close to a turn waypoint
        # Then we do a series of checks that result in this
        # intruder being ignored or not: 
        #       -check if there is a loss of separation 
        #       -check if the intent information rules out the conflict 
        #       -check if priority checks out 
        #       -check if aircraft is coming from the back 
        # Lastly, we do conflict resolution
        #print(f'-------- {ownship.id[idx1]} -------')
        
        # ------------------- Pre-processing --------------------
        # Extract ownship data
        #v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1]])# [m/s]
        # Also take distance to other aircraft
        dist2others = conf.dist_mat[idx1]
        # Get the lookahead time
        t = bs.settings.asas_dtlookahead
        # Get the separation distances
        self.hpz = (conf.hpz[idx1]) * bs.settings.asas_marv
        # Initialize velocity obstacles
        VelocityObstacles = []
        
        # Initialise per-intruder variables
        n_intr = len(idx_pairs)
        should_speed= [ownship.ap.tas[idx1]] * n_intr # Speed in metres, None means maintain current speed
        should_hold_altitude = [None] * n_intr #True if hold otherwise False, None means indifferent
        should_ascend = [None] * n_intr # True or False, None means indifferent
        should_descend = [None] * n_intr #True or False, None means indifferent
        los_list = [False] * n_intr
        head_list = [False] * n_intr
        rogue_list = [False] * n_intr
        open_airspace = bs.traf.actedge.edge_airspace_type[idx1] == 0
        landing = (not bs.traf.swlnav[idx1]) and bs.traf.actwp.swlastwp[idx1]
        
        # Initialise track new
        track_new = bs.traf.ap.trk[idx1]
        
        
        # ------------ Aircraft above or below check --------------
        # We check if we can ascend or descend by checking aircraft above
        # or below
        can_ascend, can_descend = self.ac_above_below_check(conf, ownship, intruder, idx1, dist2others, open_airspace)
        #print(can_ascend, can_descend)
        
        # ------------ Start of iteration --------------
        for i, idx_pair in enumerate(idx_pairs):
            # Get the index of the intruder
            idx2 = intruder.id.index(conf.confpairs[idx_pair][1])
            # Get the velocity of the intruder
            #v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
            
            hdg_ownship = conf.conftrks[idx_pair][0]
            hdg_intruder = conf.conftrks[idx_pair][1]
            gs_ownship = ownship.gs[idx1]
            gs_intruder = intruder.gs[idx2]
            
            v1 = np.array([gs_ownship * np.sin(np.radians(hdg_ownship)), gs_ownship * np.cos(np.radians(hdg_ownship))])
            v2 = np.array([gs_intruder * np.sin(np.radians(hdg_intruder)), gs_intruder * np.cos(np.radians(hdg_intruder))])
            
            # Also get the fake speeds and headings and stuff
            
            # Extract conflict bearing and distance information
            qdr = conf.qdr[idx_pair]
            dist= conf.dist_mat[idx1][idx2]
            
            if dist > conf.dist[idx_pair]:
                dist = conf.dist[idx_pair]

            # Find the bearing of the intruder with respect to where we are heading
            qdr_intruder = ((qdr - hdg_ownship) + 180) % 360 - 180
                
            # -------------- State related checks -----------
            # Determine if intruder is in front
            if not open_airspace:
                in_front = (-self.frnt_tol < qdr_intruder < self.frnt_tol)
                # Determine if intruder is in the back
                in_back = (qdr_intruder < -180 + self.frnt_tol or 180 - self.frnt_tol < qdr_intruder)
            else:
                # In open airspace, we want to check if aircraft is entirely behind us, and the tolerance applies
                # to the heading difference.
                in_front = (-80 < qdr_intruder < 80) and (self.heading_diff(qdr, qdr_intruder) < 30)
                in_back = (qdr_intruder < -100 or 100 < qdr_intruder) and (self.heading_diff(qdr, qdr_intruder) < 30)

            # Determine if intruder is close in altitude:
            alt_ok = ((abs(ownship.alt[idx1] - intruder.alt[idx2])) > self.hpz)
            # Determine if we have a LOS
            los = (True if idx_pair in conf.lospairs else False) or conf.dist_mat[idx1,idx2] < self.rpz or conf.dist[idx_pair] < self.rpz
            # Determine if intruder is right above or below
            above = ((ownship.alt[idx1] - intruder.alt[idx2]) < 0)
            below = ((ownship.alt[idx1] - intruder.alt[idx2]) > 0)
            # Determine if intruder is coming towards us
            head_on = (abs((np.degrees(self.angle(v1, v2)))) > (180-self.frnt_tol))
            # Does the intent check out? If true, then the paths won't intersect
            intent_ok = self.check_intent(conf, ownship, intruder, idx1, idx2)
            # Does the priority check out? If true, then ownship has greater priority
            priority_ok = self.check_prio(ownship, intruder, idx1, idx2)
            # Rogue
            intruder_rogue = intruder.id[idx2][0] == 'R'
            
            if intruder_rogue:
                rogue_list[i] = True
                # We're in trouble. It doesn't matter if it's coming from the back or 
                # the front, do SOMETHING.
                if los:
                    # We're either in complete LOS or we're above or below
                    if alt_ok:
                        # Ok no LOS yet, just stop ascending or descending and let
                        # the rogue pass.
                        #print('ROGUE, alt_ok.')
                        should_speed[i] = 5*kts
                        should_hold_altitude[i] = True
                        should_ascend[i] = False
                        should_descend[i] = False
                        los_list[i] = True
                        continue
                    
                    else:
                        # Complete LOS, go slow and try to hop up a layer.
                        should_speed[i] = 5*kts
                        should_hold_altitude[i] = False
                        should_ascend[i] = True
                        should_descend[i] = True
                        los_list[i] = True
                        continue
                    
                else:
                    # We're not at LOS, so no need to go slow, but get out of the way
                    should_hold_altitude[i] = False
                    should_ascend[i] = True
                    should_descend[i] = True
                    los_list[i] = True
                    # Also create a VO for this guy if he's not in the back
                    if not in_back:
                        #print('Add VO rogue.')
                        VelocityObstacles.append(self.get_VO(conf, ownship, intruder, idx1, idx2, idx_pair))
                    continue
                        
            # -------------- What to do if LOS ----------------
            if los:
                # If the circles are overlapping, then VO will not work, so
                # there's no continuing after this if statement, we need return
                #  or continue statements in here
                if alt_ok:
                    # Our distance is small, but we're still ok altitude wise. That means
                    # that one of the aircraft was ascending/descending towards the
                    # other. 
                    if above:
                        # The intruder is above us, so if we have priority, we just continue our way
                        # However, we stop ascending to let the intruder ascend first
                        if priority_ok:
                            #print('LOS, alt ok, intruder above, priority ok.')
                            # We do the same speed and hold altitude
                            should_hold_altitude[i] = True
                            should_ascend[i] = False
                            should_descend[i] = False
                            los_list[i] = True
                            continue
                        else:
                            # We slow down and hold altitude
                            #print('LOS, alt ok, intruder above, priority not ok.')
                            should_speed[i] = 5*kts
                            should_hold_altitude[i] = True
                            should_ascend[i] = False
                            should_descend[i] = False
                            los_list[i] = True
                            continue
                    elif below:
                        # The intruder is below
                        if priority_ok:
                            # We have priority, continue what we were doing
                            #print('LOS, alt ok, intruder below, priority ok.')
                            # We do the same speed and altitude
                            should_hold_altitude[i] = True
                            should_ascend[i] = False
                            should_descend[i] = False
                            los_list[i] = True
                            continue
                        
                        else:
                            #If we can ascend, then we'll do that, unless we're already ascending
                            if can_ascend and ownship.vs[idx1] < 0.1:
                                #print('LOS, alt ok, intruder below, priority not ok.')
                                # We can ascend, and slow down.
                                should_speed[i] = 5*kts
                                should_hold_altitude[i] = False
                                should_ascend[i] = True
                                should_descend[i] = False
                                los_list[i] = True
                                continue
                                
                            elif not can_ascend:
                                # We cannot ascend, stop and let the aircraft pass
                                # We also maintain current altitude
                                #print('LOS, alt ok, intruder below, priority not ok, cannot ascend.')
                                should_speed[i] = 5*kts
                                should_hold_altitude[i] = False
                                should_ascend[i] = False
                                should_descend[i] = True
                                los_list[i] = True
                                continue
                            else:
                                # We're probably already ascending then, so just continue but go slow
                                #print('LOS, alt ok, intruder below, priority not ok, already ascending.')
                                should_speed[i] = 5*kts
                                should_hold_altitude[i] = False
                                should_ascend[i] = True
                                should_descend[i] = False
                                los_list[i] = True
                                continue
                
                else:
                    # There is a complete LOS, so if we have higher priority, then we continue
                    # otherwise we stop
                    if priority_ok:
                        #print('Complete LOS, priority ok.')
                        # LOS anyway, just continue your way
                        los_list[i] = True
                        continue
                    else:
                        # We have lower priority, we maintain altitude and go slow
                        #print('Complete LOS, no priority.')
                        should_speed[i] = 5*kts
                        should_hold_altitude[i] = True
                        should_ascend[i] = False
                        should_descend[i] = False
                        los_list[i] = True
                        continue
                    
            # Until now we either have appended something to all lists and continued, or we
            # did not append anything to any lists. 
                    
            # -------------- What to do if intruder in front ----------
            if in_front:
                # Check if the intruder is actually coming towards us
                if head_on:
                    head_list[i] = True
                    # Head-on conflict, bad situation. Immediately ascend without going further
                    if self.in_headon[idx1] != True:
                        # This means this aircraft hasn't solved for head-on already
                        # Basically, the aircraft with the higher flight number ascends, and the one with the lower
                        # flight number descends. This ensures that any encounter of the sort is
                        # solved, regardless of whether one of the drones is a rogue or not.
                        if self.check_prio(ownship, intruder, idx1, idx2):
                            # Means our flight number is smaller. See if we can ascend or descend
                            if open_airspace:
                                # We first see if we can ascend to the layer above.
                                alt_ascend = self.get_above_cruise_layer(ownship, idx1)
                                if alt_ascend > 0 and can_ascend:
                                    alt = alt_ascend
                                    should_hold_altitude[i] = False
                                    should_ascend[i] = True
                                    should_descend[i] = False
                                else:
                                    # We try to jump up 30 ft
                                    alt = ownship.alt[idx1] + 30*ft
                        else:
                            #We're in constrained  
                            alt_ascend = self.get_above_cruise_layer(ownship, idx1)
                            alt_descend = self.get_below_cruise_layer(ownship, idx1)
                            if alt_ascend > 0 and can_ascend:
                                alt = alt_ascend
                                should_hold_altitude[i] = False
                                should_ascend[i] = True
                                should_descend[i] = False
                                
                            elif alt_descend > 0 and can_descend:
                                alt = alt_descend
                                should_hold_altitude[i] = False
                                should_ascend[i] = False
                                should_descend[i] = True
                                
                            else:
                                # Go to unused layer
                                alt = self.get_above_empty_layer(ownship, idx1)
                                if alt == 0:
                                    alt = self.get_below_empty_layer(ownship, idx1)
                                    
                            # Give stack command
                            if hopping:
                                stack.stack(f'ALT {ownship.id[idx1]} {alt}')
                                stack.stack(f'LNAV {ownship.id[idx1]} ON') 
                                stack.stack(f'VNAV {ownship.id[idx1]} ON')
                        #print('In front, head-on, attempt alt change.')
                        
                    self.in_headon[idx1] = True
                
                elif abs(intruder.vs[idx2]) > 0.1:
                    #print('In front, intruder is performing a manoeuver.')
                    should_hold_altitude[i] = True
                    should_ascend[i] = False
                    should_descend[i] = False
                    
                else:
                    #print('In front, ascend.')
                    should_hold_altitude[i] = False
                    should_ascend[i] = True
                    should_descend[i] = False
                    
            # ------------- What to do if intruder in back --------------       
            elif in_back:
                #print('In back, hold.')
                # We're the ones in the front, don't ascend, but also don't do anything else
                should_hold_altitude[i] = True
                should_ascend[i] = False
                should_descend[i] = False
                continue
            
            # If we pass this check ^ , then intruder is not in the back, nor in the front
            # Solve conflicts in the same plane, so don't ascend or descend.
            else:
                #print('Normal conflict.')
                should_hold_altitude[i] = True
                should_ascend[i] = False
                should_descend[i] = False
            
            # -------------- Intent check --------------
            if intent_ok:
                #print('Intent ok')
                # This means that the aircraft paths don't even get close, 
                # so just continue.
                continue
            
            # -------------- Priority check ------------
            
            if priority_ok and not in_front:
                # We have the greater priority, so if we didn't continue
                # till now, we do now
                # Only do this if we're not in the back though
                #print('Prio ok and not in front')
                continue
            
            # ------------ CONFLICT RESOLUTION -------------
            # We already checked once if we're in a loss of separation, 
            # but only if we are in the back, and then we would stop. 
        
                
            # Get Velocity Obstacle
            #print('Add VO.')
            VelocityObstacles.append(self.get_VO(conf, ownship, intruder, idx1, idx2, idx_pair))
        
        #------------ FOR LOOP OVER ----------------
        #------------ Overall processing -----------
        #First of all, if we can ascend, let's do it. If we're in a head-on, the required
        # command was already given
        ascend = (False not in should_ascend) and (True not in should_hold_altitude)
        descend = (False not in should_descend) and (True not in should_hold_altitude)

        # DO WE HAVE A ROGUE? IF YES, THEN HANDLE IT
        if np.any(rogue_list):
            # We have a rogue aircraft in the list. Ascend to above unused layer if we can.
            i = rogue_list.index(True)
            # Check if we should ascend
            if should_ascend[i]:
                if can_ascend:
                    # Go to cruise layer above
                    alt = self.get_above_cruise_layer(ownship, idx1)
                    if alt > 0 and hopping:
                        stack.stack(f'ALT {ownship.id[idx1]} {alt}')
                        stack.stack(f'LNAV {ownship.id[idx1]} ON') 
                        stack.stack(f'VNAV {ownship.id[idx1]} ON')
                else:
                    #Go to unused layer above
                    alt = self.get_above_empty_layer(ownship, idx1)
                    if alt > 0 and hopping:
                        stack.stack(f'ALT {ownship.id[idx1]} {alt}')
                        stack.stack(f'LNAV {ownship.id[idx1]} ON') 
                        stack.stack(f'VNAV {ownship.id[idx1]} ON')
                        
        if ascend and can_ascend and self.in_headon[idx1] != True:
            alt = self.get_above_cruise_layer(ownship, idx1)
            if alt > 0 and hopping:
                stack.stack(f'ALT {ownship.id[idx1]} {alt}')
                stack.stack(f'LNAV {ownship.id[idx1]} ON') 
                stack.stack(f'VNAV {ownship.id[idx1]} ON')
            
        elif descend and can_descend and self.in_headon[idx1] != True:
            alt = self.get_below_cruise_layer(ownship, idx1)
            if alt>0 and hopping:
                stack.stack(f'ALT {ownship.id[idx1]} {alt}')
                stack.stack(f'LNAV {ownship.id[idx1]} ON') 
                stack.stack(f'VNAV {ownship.id[idx1]} ON')
            
        #If we cannot ascend, let's mark ourselves as stuck
        if not can_ascend and ascend:
            # Means we are most likely stuck behind an aircraft. Mark this aircraft
            # as stuck, and in queue for ascending
            self.stuck[idx1] = True
            #print('STUCK')
            
        if ownship.gs[idx1] !=0:
            # Combine all velocity obstacles into one big polygon
            CombinedObstacles = cascaded_union(VelocityObstacles)
            
            #####################################################
            # SHORTEST WAY OUT METHOD
            #####################################################      
            if self.heading_based == True and open_airspace:
                # We're in open airspace, do heading based.
                # Find the closest point on polygon from the current velocity and attempt
                # solution.
                # Add big circle ring around polygon so we make sure that the solution will
                # not be beyond vmax
                limit_vmax = Point(0,0).buffer(100)-Point(0,0).buffer(vmax)
                # Get the union of these two polygons
                bad_states = limit_vmax.union(CombinedObstacles)
                # Get the closest point on this polygon to the current velocity
                p1, _ = nearest_points(bad_states, Point(v1))
                # P1 is our new solution (combined track and velocity). Get each
                gs_new = np.sqrt(p1.x**2 + p1.y**2)
                track_new = (np.arctan2(p1.x[0,:],p1.y[1,:])*180/np.pi) % 360
                
            else:
                # Get minimum and maximum speed of ownship
                vmin = ownship.perf.vmin[idx1]
                if bs.traf.ap.inturn[idx1] or bs.traf.ap.dist2turn[idx1] < 100:
                    vmax = bs.traf.actwp.nextturnspd[idx1] 
                else:
                    vmax = ownship.perf.vmax[idx1]
                
                # Create velocity line
                v_dir = self.normalized(v1)
                v_line_min = v_dir * vmin
                v_line_max = v_dir * vmax
                
                # Create velocity line
                line = LineString([v_line_min, v_line_max])
                # Get the intersection with the velocity obstacles
                intersection = CombinedObstacles.intersection(line)
                
                #---------------- RESOLUTION SPEEDS ---------------
                # Apply the VO resolution speed
                if intersection:
                    solutions = []
                    if type(intersection) == LineString:
                        for velocity in list(intersection.coords):
                            # Check whether to put velocity "negative" or "positive". 
                            # Drones can fly backwards.
                            if np.degrees(self.angle(velocity, v1)) < 1:
                                solutions.append(self.norm(velocity))
                            else:
                                solutions.append(-self.norm(velocity))
                    elif type(intersection) == Point:
                        # Take this as the velocity then
                        velocity = [intersection.x, intersection.y]
                        if np.degrees(self.angle(velocity, v1)) < 1:
                            solutions.append(self.norm(velocity))
                        else:
                            solutions.append(-self.norm(velocity))
                        
                    elif type(intersection) == MultiPoint:
                        for point in intersection:
                            velocity = [point.x, point.y]
                        if np.degrees(self.angle(velocity, v1)) < 1:
                            solutions.append(self.norm(velocity))
                        else:
                            solutions.append(-self.norm(velocity))
                            
                    else:
                        for line in intersection:
                            for velocity in list(line.coords):
                                # Check whether to put velocity "negative" or "positive". 
                                # Drones can fly backwards.
                                if np.degrees(self.angle(velocity, v1)) < 1:
                                    solutions.append(self.norm(velocity))
                                else:
                                    solutions.append(-self.norm(velocity))
                                
                    pos_speeds = [spd for spd in solutions if spd >= 0]
                    neg_speeds = [spd for spd in solutions if spd < 0]
                    if pos_speeds:
                        gs_new = min(pos_speeds)
                    elif neg_speeds:
                        gs_new = max(neg_speeds)
                    else:
                        gs_new = min(ownship.ap.tas[idx1], vmax)
                        
                elif VelocityObstacles and not intersection:
                    # Means we need to take gs_new as vmax
                    gs_new = vmax
                    #print('HERE')
                    
                else:
                    # Nothing worked, do nothing
                    gs_new = min(ownship.ap.tas[idx1], vmax)
                
        
            gs_new = min([gs_new, min(should_speed), vmax])
            
            # Increase/decrease the speed by a small bit to account for rounding errors
            if bs.traf.gs[idx1] - gs_new > 0:
                #Slowing down
                gs_new = gs_new - 0.01
            else:
                #Speeding up
                gs_new = gs_new + 0.01
        elif ownship.gs[idx1] == 0:
            if landing:
                # We are landing, maintain gs_new = 0
                gs_new = 0
            else:
                gs_new = 5*kts
            
        # Select the right altitude
        if True in should_hold_altitude:
            #print('Hold altitude.')
            alt_new = ownship.alt[idx1]
            self.altactivearr[idx1] = True
        else:
            alt_new = ownship.ap.alt[idx1]
            self.altactivearr[idx1] = False
        
        return gs_new, alt_new, track_new
            

    
    ##### Helper functions #####
    def check_speed(self, conf, ownship, intruder, idx1, speed):
        # Check if the given speed for aircraft idx1 would cause any conflicts
        # with other aircraft in the vicinity. however, ignore aircraft that
        # are in the back or have a lower priority, but always count ones
        # in the front
        # First of all, extract distance to all other aircraft
        dist2others = conf.dist_mat[idx1]
        # Extract the closest aircraft, within lookahead distance
        dist_look = ownship.gs[idx1] * conf.dtlookahead[idx1]
        idx_others = np.where(dist2others < dist_look)[0]
        # Get ownship speed
        v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1]])
        # Initialize Velocity Obstacles
        VelocityObstacles = []
        # Initialize los bool
        los = False
        for idx2 in idx_others:
            # Eliminate aircraft that are in the back, have a lower priority
            # but always consider aircraft in the front
            qdr = conf.qdr_mat[idx1, idx2]
            dist = conf.dist_mat[idx1, idx2]
            if dist < self.rpz:
                los = True
                continue
            qdr_intruder = ((qdr - ownship.trk[idx1]) + 180) % 360 - 180
            in_front = (-self.frnt_tol < qdr_intruder < self.frnt_tol)
            in_back = (qdr_intruder < -180 + self.frnt_tol or 180 - self.frnt_tol < qdr_intruder)
            priority_ok = self.check_prio(ownship, intruder, idx1, idx2)
            if (priority_ok and not in_front) or (in_back):
                continue
            VelocityObstacles.append(self.get_VO_real(conf, ownship, intruder, idx1, idx2))
        if los:
            return False
        # If there aren't any velocity obstacles, return true
        if not VelocityObstacles:
            return True
        # Combine all the VOs
        CombinedObstacles = cascaded_union(VelocityObstacles)
        # Create the speed vector
        v_dir = self.normalized(v1)
        v_to_check = v_dir * speed
        wpxv = v_to_check[0]
        wpyv = v_to_check[1]
        wpoint = Point(wpxv, wpyv)
        return not CombinedObstacles.contains(wpoint)
    
    def get_VO(self, conf, ownship, intruder, idx1, idx2, idx_pair):
        hdg_ownship = conf.conftrks[idx_pair][0]
        hdg_intruder = conf.conftrks[idx_pair][1]
        gs_ownship = ownship.gs[idx1]
        gs_intruder = intruder.gs[idx2]
    
        v2 = np.array([gs_intruder * np.sin(np.radians(hdg_intruder)), gs_intruder * np.cos(np.radians(hdg_intruder))])
        
        t = conf.dtlookahead[idx1]
        # Get QDR and DIST of conflict
        qdr = conf.qdr[idx_pair]
        dist = conf.dist[idx_pair]
        # Get radians qdr
        qdr_rad = np.radians(qdr)
        # Get relative position
        x_rel = np.array([np.sin(qdr_rad)*dist, np.cos(qdr_rad)*dist])
        # Get the speed of the intruder
        #v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
        # Get cutoff legs
        left_leg_circle_point, right_leg_circle_point = self.cutoff_legs(x_rel, self.rpz, t)
        # Extend cutoff legs
        right_leg_extended = right_leg_circle_point * t
        left_leg_extended = left_leg_circle_point * t
        # Get the final VO
        final_poly = Polygon([right_leg_extended, (0,0), left_leg_extended])
        # Translate it by the velocity of the intruder
        final_poly_translated = translate(final_poly, v2[0], v2[1])
        # Return
        return final_poly_translated
    
    def get_VO_real(self, conf, ownship, intruder, idx1, idx2):
        t = conf.dtlookahead[idx1]
        # Get QDR and DIST of conflict
        qdr = conf.qdr_mat[idx1, idx2]
        dist = conf.dist_mat[idx1, idx2]
        # Get radians qdr
        qdr_rad = np.radians(qdr)
        # Get relative position
        x_rel = np.array([np.sin(qdr_rad)*dist, np.cos(qdr_rad)*dist])
        # Get the speed of the intruder
        v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
        # Get cutoff legs
        left_leg_circle_point, right_leg_circle_point = self.cutoff_legs(x_rel, self.rpz, t)
        # Extend cutoff legs
        right_leg_extended = right_leg_circle_point * t
        left_leg_extended = left_leg_circle_point * t
        # Get the final VO
        final_poly = Polygon([right_leg_extended, (0,0), left_leg_extended])
        # Translate it by the velocity of the intruder
        final_poly_translated = translate(final_poly, v2[0], v2[1])
        # Return
        return final_poly_translated
    
    def check_prio(self, ownship, intruder, idx1, idx2):
        if not hasattr(ownship, 'priority'):
            # Determine which ACID number is bigger
            if self.check_flight_numbers(ownship, intruder, idx1, idx2):
                return True
            else:
                return False
            
        ownship_prio = ownship.priority[idx1]
        intruder_prio = intruder.priority[idx2]
        
        if self.in_headon[idx2] == False and self.in_headon[idx1] == True:
            # We are in a head-on, other aircraft isn't
            return True
        
        if self.in_headon[idx2] == True and self.in_headon[idx1] == False:
            # Other aircraft is resolving a head-on, give it prio
            return False
        
        if (ownship_prio > intruder_prio):
            # Priority of intruder is greater, continue.
            return True
        
        if (ownship_prio == intruder_prio):
            # Determine which ACID number is bigger
            if self.check_flight_numbers(ownship, intruder, idx1, idx2):
                return True
            
        return False
    
    def check_flight_numbers(self, ownship, intruder, idx1, idx2):
        """If ACID of idx1 < idx2, returns True, else False.
        """
        id1= ownship.id[idx1]
        id2 = intruder.id[idx2]
        prio_bigger = int(''.join(filter(str.isdigit, id1))) < int(''.join(filter(str.isdigit, id2)))
        if prio_bigger:
                return True
        else:
            return False
            
    def check_intent(self, conf, ownship, intruder, idx1, idx2):
        #if (intruder.intent[idx2] is not None) and (ownship.intent[idx1] is not None):
        # intent1, target_alt1 = ownship.intent[idx1]
        # intent2, target_alt2 = intruder.intent[idx2]
        
        target_alt2 = intruder.selalt[idx2]
        
        # Find closest points between the two intent paths
        #pown, pint = nearest_points(intent1, intent2)
        
        # Find the distance between the points
        #point_distance = kwikdist(pown.y, pown.x, pint.y, pint.x) * nm #[m]
        
        # Also do vertical intent
        # Difference between own altitude and intruder target
        diff = ownship.alt[idx1] - target_alt2
        # Basically, there are three conditions to be met in order to skip
        # a conflict due to intent:
        # 1. The minimum distance between the horizontal intent lines is greater than r;
        # 2. The difference between the current altitude and the target altitude of the 
        # intruder is greater than the vertical separation margin;
        # 3. The altitude difference and vertical velocity of the intruder have the same sign.
        # This means that if the aircraft is coming from above (negative), and the altitude difference
        # is positive (thus target altitude is below ownship), then their paths will intersect. 
        
        # Disconsidered distance-based intent
        # if ((point_distance > self.rpz) or ((abs(diff) >= self.hpz)) and \
        #     (abs(intruder.vs[idx2]) < 0.1) and np.sign(diff) == np.sign(intruder.vs[idx2])):
        
        # Altitude intent
        if ((abs(diff) >= self.hpz)) and (abs(intruder.vs[idx2]) < 0.1) and np.sign(diff) == np.sign(intruder.vs[idx2]):
                # Intent is ok
            return True
        else:
            return False
            
    def get_above_cruise_layer(self, ownship, idx1):
        # Get the cruise layer above the current altitude of the ownship
        return bs.traf.closest_cruise_layer_top[idx1]

        
    def get_below_cruise_layer(self, ownship, idx1):
        # Get the cruise layer below the current altitude of the ownship
        return bs.traf.closest_cruise_layer_bottom[idx1]
    
    def get_above_empty_layer(self, ownship, idx1):
        # Get the cruise layer above the current altitude of the ownship
        return bs.traf.closest_empty_layer_top[idx1]
        
    def get_below_empty_layer(self, ownship, idx1):
        # Get the cruise layer below the current altitude of the ownship
        return bs.traf.closest_empty_layer_bottom[idx1]
        
    def in_cruise_layer(self, ownship, idx1):
        if hasattr(bs.traf, 'flight_layer_type'):
            return bs.traf.flight_layer_type[idx1] == 'C'
        else:
            return True
        
    def heading_diff(self, init, final):
        if init > 360 or init < 0 or final > 360 or final < 0:
            raise Exception("out of range")
        diff = final - init
        absDiff = abs(diff)

        if absDiff == 180:
            return absDiff
        elif absDiff < 180:
            return diff
        elif final > init:
            return absDiff - 360
        else:
            return 360 - absDiff
                
    def ac_above_below_check(self, conf, ownship, intruder, idx1, dist2others, open_airspace):
        can_ascend = True
        can_descend = True
        # Get aircraft that are close
        is_close = np.where(dist2others < self.rpz * 2)[0]
        # Get the vertical distance for these aircraft
        vertical_dist = ownship.alt[idx1] - intruder.alt[is_close]
        # Check if any is smaller than cruise layer difference
        if open_airspace:
            # in open airspace we want to look waay above.
            cruise_diff_ascend = np.logical_and(0 > vertical_dist, vertical_dist > (-self.cruiselayerdiff * 3))
            cruise_diff_descend = np.logical_and(0 < vertical_dist, vertical_dist < (self.cruiselayerdiff * 3))
        else:
            cruise_diff_ascend = np.logical_and(0 > vertical_dist, vertical_dist > (-self.cruiselayerdiff * 1.1))
            cruise_diff_descend = np.logical_and(0 < vertical_dist, vertical_dist < (self.cruiselayerdiff * 1.1))
        # Check also if any is smaller than conf.hpz
        conf_diff = np.abs(vertical_dist) > conf.hpz[idx1]
        # Do the or operation on these two
        dealbreaker_ascend = np.logical_or(cruise_diff_ascend, conf_diff) 
        dealbreaker_descend = np.logical_or(cruise_diff_descend, conf_diff)
        
        # Also check if we're at the bottom or top
        if self.get_above_cruise_layer == 0 or np.any(dealbreaker_ascend):
            can_ascend = False
        if self.get_below_cruise_layer == 0 or np.any(dealbreaker_descend):
            can_descend = False
            
        return can_ascend, can_descend
                        
    def pairs(self, conf, ownship, intruder, idx):
        '''Returns the indices of conflict pairs that involve aircraft idx
        '''
        idx_pairs = np.array([], dtype = int)
        for idx_pair, pair in enumerate(conf.confpairs):
            if (ownship.id[idx] == pair[0]):
                idx_pairs = np.append(idx_pairs, idx_pair)
        return idx_pairs
    
    def reso_pairs(self, conf, ownship, intruder, idx):
        '''Returns the indices of aircraft that are resolving conflicts with aircraft idx.
        '''
        idx_confs = np.array([], dtype = int)
        for pair in self.resopairs:
            if pair[0] == ownship.id[idx]:
                idx_confs = np.append(idx_confs, ownship.id.index(pair[1]))
        return idx_confs
    
    def perp_left(self, a):
        ''' Gives perpendicular unit vector pointing to the "left" (+90 deg)
        for vector "a" '''
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b/np.linalg.norm(b)

    def perp_right(self, a):
        ''' Gives perpendicular unit vector pointing to the "right" (-90 deg)
        for vector "a" '''
        b = np.empty_like(a)
        b[0] = a[1]
        b[1] = -a[0]
        return b/np.linalg.norm(b)
        
    def cutoff_legs(self, x, r, t):
        '''Gives the cutoff point of the right leg.'''
        x = np.array(x)
        # First get the length of x
        x_len = self.norm(x)
        # Find the sine of the angle
        anglesin = r / x_len
        # Find the angle itself
        angle = np.arcsin(anglesin) # Radians
        
        # Find the rotation matrices
        rotmat_left = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])
        
        rotmat_right = np.array([[np.cos(-angle), -np.sin(-angle)],
                           [np.sin(-angle), np.cos(-angle)]])
        
        # Compute rotated legs
        left_leg = rotmat_left.dot(x)
        right_leg = rotmat_right.dot(x)  
        
        circ = x/t
        xc = circ[0]
        yc = circ[1]
        xp_r = right_leg[0]
        yp_r = right_leg[1]
        xp_l = left_leg[0]
        yp_l = left_leg[1]
        
        b_r = (-2 * xc - 2 * yp_r / xp_r * yc)
        a_r = 1 + (yp_r / xp_r) ** 2    
         
        b_l = (-2 * xc - 2 * yp_l / xp_l * yc)
        a_l = 1 + (yp_l / xp_l) ** 2    
        
        x_r = -b_r / (2 * a_r)
        x_l = -b_l / (2 * a_l)
        
        y_r = yp_r / xp_r * x_r
        y_l = yp_l / xp_l * x_l 

        # Compute normalised directions
        right_cutoff_leg_dir = self.normalized(right_leg)
        self.right_cutoff_leg_dir = right_cutoff_leg_dir
        
        left_cutoff_leg_dir = self.normalized(left_leg)
        self.left_cutoff_leg_dir = left_cutoff_leg_dir
        
        return np.array([x_l, y_l]), np.array([x_r, y_r])
    
    def in_confpairs(self, idx):
        in_bool = False
        for pair in bs.traf.cd.confpairs:
            if idx in pair:
                in_bool = True
                break
        return in_bool
                
    def perp(self, a):
        return np.array((a[1], -a[0]))
    
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
    
    def dist_sq(self, a, b):
        return self.norm_sq(b - a)
    
    ### Modified hdgactive function
    @property
    def hdgactive(self):
        ''' Return a boolean array sized according to the number of aircraft
            with True for all elements where heading is currently controlled by
            the conflict resolution algorithm.
        '''
        return np.logical_and(bs.traf.actedge.edge_airspace_type == 0, self.heading_based)
    
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
        return np.logical_and(self.active, self.altactivearr)
    
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
                dist_ok = (distance > 100) 
                # We also want enough altitude
                alt_ok = abs(ownship.alt[idx1]-intruder.alt[idx2]) >= (2.5* self.layer_height)
                # We also want to make sure that the aircraft has finished its vertical manoeuvre 
                vs_ok = abs(ownship.vs[idx1]) < 0.1
                # Lastly, we want to make sure that the autopilot speed command doesn't create
                # other conflicts
                ap_spd_ok = self.check_speed(conf, ownship, intruder, idx1, ownship.ap.tas[idx1]) and (distance > 32)

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
                    abs(ownship.trk[idx1] - intruder.trk[idx2]) < 30.0 and \
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
                nav_ok = (alt_ok or dist_ok) or ap_spd_ok
                conf_ok = (past_cpa and not hor_los and not is_bouncing) or alt_ok

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not (nav_ok and conf_ok)):
                # Are we trying to land?
                landing = (not bs.traf.swlnav[idx1]) and bs.traf.actwp.swlastwp[idx1]
                # Enable ASAS for this aircraft
                changeactive[idx1] = True
                # We also need to check if this aircraft needs to be doing a turn, aka, if the
                # autopilot speed is lower than the CR speed. If it is, then we need to update
                # the speed to that value. Thus, either the conflict is still ok and CD won't be
                # triggered again, or a new conflict will be triggered and CR will take over again.
                if self.tas[idx1] > bs.traf.ap.tas[idx1]:
                    self.tas[idx1] = bs.traf.ap.tas[idx1]
                    
                # What if we want to land?
                if landing:
                    #Attempt to land if nobody below. Otherwise, hold altitude and speed 0.
                    self.tas[idx1] = 0
                    # Check if any aircraft below
                    dist2others = conf.dist_mat[idx1]
                    is_close = np.where(dist2others < 64)[0]
                    # Get the vertical distance for these aircraft
                    vertical_dist = ownship.alt[idx1] - intruder.alt[is_close]
                    
                    # If any of these altitudes is greater than 0, hold altitude and wait.
                    if np.any(vertical_dist > 0):
                        self.alt[idx1] = bs.traf.alt[idx1]
                    else:
                        # We can attempt landing
                        self.alt[idx1] = 0
                
            else:
                # Switch ASAS off for ownship if there are no other conflicts
                # that this aircraft is involved in.
                changeactive[idx1] = changeactive.get(idx1, False)
                # If conflict is solved, remove it from the resopairs list
                delpairs.add(conflict)
                # In case it was a head-on conflict
                self.in_headon[idx1] = False
                    
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