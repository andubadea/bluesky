import numpy as np
import bluesky as bs
import copy
from bluesky.traffic.asas import ConflictResolution
from bluesky.core import Entity
from bluesky.tools.aero import kts


def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'INTENTCR',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config


class IntentCR(ConflictResolution):
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
            if intent_geom[pair_idx][0] is None and intent_geom[pair_idx][1] is None:
                # This is a state-based conflict, so we need to do some special things
                # First, check if the intruder is in the front
                qdr = qdr_mat[ownship_idx, intruder_idx]
                qdr_intruder = ((qdr - ownship.trk[intruder_idx]) + 180) % 360 - 180
                intruder_in_front = frnt_tol < qdr_intruder < frnt_tol
                intruder_in_back = (qdr_intruder < -180 + frnt_tol or 180 - frnt_tol < qdr_intruder)
                
                if intruder_in_front:
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
                elif dist_to_int[pair_idx][0] < dist_to_int[pair_idx][1]:
                    # We slow down
                    print('Going slow state-based.')
                    newgs[ownship_idx] = 0
                    continue
                else:
                    # The other aircraft will slow down
                    continue
            # Now for the intent based problems.        
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
                
            # If we're here, then we must have a classical intersection conflict, and we should solve it
            # by making the aircraft that has less priority slow down such that the other aircraft
            # has time to clear the intersection.
            ownship_has_prio = int(''.join(filter(str.isdigit, ownship_id))) < \
                               int(''.join(filter(str.isdigit, intruder_id)))
            
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