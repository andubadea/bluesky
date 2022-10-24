from bluesky.core.simtime import timed_function
import bluesky as bs
import numpy as np
from bluesky import core
from bluesky.traffic.asas import ConflictResolution
from shapely.geometry import Point, LineString
from shapely.ops import cascaded_union, nearest_points
from bluesky import stack
from bluesky.tools.geo import kwikdist, kwikqdrdist, latlondist, qdrdist
from shapely.geometry.polygon import Polygon
from shapely.affinity import translate
from bluesky.tools.aero import nm, ft, kts
from bluesky.tools.misc import degto180

def init_plugin():
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'M2CENTRALTACTICAL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class M2CentralTactical(ConflictResolution):
    def __init__(self):
        super().__init__()
        self.frnt_tol = 20
        self.rpz = 40
        
    def resolve(self, conf, ownship, intruder):
        # Make a copy of traffic data, track and ground speed
        newgscapped = np.copy(ownship.gs)
        newalt = np.copy(ownship.alt)
        
        for idx1 in np.argwhere(conf.inconf).flatten():
            # Skip if rogue aircraft
            if ownship.id[idx1][0] == 'R':
                continue  
            
            idx_pairs = self.pairs(conf, ownship, intruder, idx1)
            
            gs_new, alt_new = self.SpeedBasedC(conf, ownship, intruder, idx1, idx_pairs)
            
            # Write the new velocity of aircraft 'idx' to traffic data
            newgscapped[idx1] = gs_new
            newalt[idx1] = alt_new
            
        # only speed-based for now
        newtrack       = ownship.ap.trk
        newvs          = ownship.ap.vs
        
    def SpeedBasedC(self, conf, ownship, intruder, idx1, idx_pairs):
        # ------------------- Pre-processing --------------------
        # Extract ownship data
        v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1]])# [m/s]
        # Also take distance to other aircraft
        dist2others = conf.dist_mat[idx1]
        # Get the lookahead time
        t = bs.settings.asas_dtlookahead
        # Get the separation distances
        self.hpz = (conf.hpz[idx1]) * bs.settings.asas_marv
        # Initialize velocity obstacles
        VelocityObstacles = []
        
        n_intr = len(idx_pairs)
        landing = (not bs.traf.swlnav[idx1]) and bs.traf.actwp.swlastwp[idx1]
        should_speed= [ownship.ap.tas[idx1]] * n_intr # Speed in metres, None means maintain current speed
        
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
            # Determine if intruder is in the back
            int_in_back = (qdr_intruder < -180 + self.frnt_tol or 180 - self.frnt_tol < qdr_intruder)
            # Determine if intruder is close in altitude:
            alt_ok = ((abs(ownship.alt[idx1] - intruder.alt[idx2])) > self.hpz)
            # Determine if we have a LOS
            los = (dist <= self.rpz)
            # Rogue
            intruder_rogue = intruder.id[idx2][0] == 'R'
            # Does the priority check out? If true, then ownship has greater priority
            own_has_prio = (self.check_prio(ownship, intruder, idx1, idx2) and (not intruder_rogue)) or int_in_back
            
            # Start solvin
            if los:
                if own_has_prio:
                    # just continue
                    continue
                else:
                    # just go slow
                    should_speed[i] = 5*kts
                    continue
                    
            if own_has_prio:
                # Just continue
                continue
            
            # Get the VO
            VelocityObstacles.append(self.get_VO(conf, ownship, intruder, idx1, idx2))
            
        if ownship.gs[idx1] != 0:
            # Combine all velocity obstacles into one big polygon
            CombinedObstacles = cascaded_union(VelocityObstacles)
            
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
        else:
            if landing:
                # We are landing, maintain gs_new = 0
                gs_new = 0
            else:
                gs_new = 5*kts
                
        return gs_new
        
##### Helper functions ####
    def get_VO(self, conf, ownship, intruder, idx1, idx2):
        t = conf.dtlookahead[idx1]
        # Get QDR and DIST of conflict
        qdr = conf.qdr_mat[idx1, idx2]
        dist = conf.dist_mat[idx1,idx2]
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
        return np.array([False] * len(self.active))