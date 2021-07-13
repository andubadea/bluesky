from bluesky.traffic.asas import ConflictResolution
import bluesky as bs
import numpy as np
from bluesky.traffic.asas import ConflictResolution
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.ops import cascaded_union, nearest_points
from shapely.affinity import translate
from bluesky.tools.geo import kwikdist
from bluesky.tools.aero import nm
import bluesky as bs
import numpy as np
import itertools

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SPEEDBASED',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class SpeedBased(ConflictResolution): 
    # Define some variables
    def __init__(self):
        super().__init__()
        
    def resolve(self, conf, ownship, intruder):
        '''We want to only solve in the velocity direction while still following the heading
        given by the autopilot. For each ownship, it calculates the minimum or maximum velocity
        needed to avoid all intruders. It then applies the solution closest to the current velocity.
        If there is no solution, it should then apply speed 0 by default, and the aircraft stops.'''
             
        # Make a copy of traffic data, track and ground speed
        newtrack    = np.copy(ownship.trk)
        newgscapped = np.copy(ownship.gs)
        newvs       = np.copy(ownship.vs)
        
        # Iterate over aircraft in conflict
        for idx in list(itertools.compress(range(len(bs.traf.cr.active)), bs.traf.cr.active)):
            # Find the pairs in which IDX is involved in a conflict
            idx_pairs = self.pairs(conf, ownship, intruder, idx)
            
            # Find solution for aircraft 'idx'
            gs_new, vs_new = self.SpeedBased(conf, ownship, intruder, idx, idx_pairs)
            
            # Write the new velocity of aircraft 'idx' to traffic data
            newgscapped[idx] = gs_new    
            newvs[idx] = vs_new       
        
        # Speed based, and 2D, for now.
        alt            = ownship.ap.alt 
        newtrack       = ownship.ap.trk
        
        return newtrack, newgscapped, newvs, alt


    def SpeedBased(self, conf, ownship, intruder, idx, idx_pairs):
        # Extract ownship data
        v_ownship = np.array([ownship.gseast[idx], ownship.gsnorth[idx]])# [m/s]
        # Take minimum separation and multiply it with safely factor
        r = conf.rpz[idx] * self.resofach
        
        print(conf.rpz[idx])
        
        # Also take distance to other aircraft
        dist2others = conf.dist_mat[idx]
        
        # Get the T factor, set it as bs.settings.asas_dtlookahead
        t = bs.settings.asas_dtlookahead

        VelocityObstacles = []
        # Go through all conflict pairs for aircraft "idx", basically take
        # intruders one by one, and create their polygons
        for i, idx_pair in enumerate(idx_pairs):
            idx_intruder = intruder.id.index(conf.confpairs[idx_pair][1])
            # Extract conflict bearing and distance information
            qdr = conf.qdr[idx_pair]
            dist= conf.dist[idx_pair]
            print(dist)
            
            # TODO: Introduce proper priority.
            qdr_intruder = ((qdr - ownship.trk[idx]) + 180)%360 - 180
            # Check if intruder is coming from the right or the back.
            if (-180 <= qdr_intruder  < -135) or (135 <= qdr_intruder  < 180):
                # go to next pair
                continue
            # Not coming from the back, but is it coming from the right?
            if not (0 <= qdr_intruder  <= 180):
                # go to next pair
                continue       
            
            # If we have a loss of separation, or the conflict is vertical,
            # just break, and stop the vertical speed
            if dist < r:
                return 1, 0   
            
            # Let's also do some intent check
            own_intent = ownship.intent.intent[idx]
            intruder_intent = intruder.intent.intent[idx_intruder]
            # Find closest points between the two intent paths
            pown, pint = nearest_points(own_intent, intruder_intent)
            # Find the distance between the points
            point_distance = kwikdist(pown.y, pown.x, pint.y, pint.x) * nm #[m]
            
            if point_distance > r:
                continue
            
            #r = r * 1.1
            
            # Determine the index of the intruder
            idx_intruder = intruder.id.index(conf.confpairs[idx_pair][1])
            
            # Convert qdr from degrees to radians
            qdr = np.radians(qdr)

            # Relative position vector between ownship and intruder
            x = np.array([np.sin(qdr)*dist, np.cos(qdr)*dist])
    
            v_intruder = np.array([intruder.gseast[idx_intruder], intruder.gsnorth[idx_intruder]])

            # Get circle
            circle = Point(x/t).buffer(r/t)

            # Get cutoff legs
            left_leg_circle_point, right_leg_circle_point = self.cutoff_legs(x, r, t)

            right_leg_extended = right_leg_circle_point * t
            left_leg_extended = left_leg_circle_point * t

            #triangle_poly = Polygon([right_leg_extended, right_leg_circle_point, left_leg_circle_point, left_leg_extended])

            #final_poly = cascaded_union([triangle_poly, circle])
            
            final_poly = Polygon([right_leg_extended, (0,0), left_leg_extended])
            
            final_poly_translated = translate(final_poly, v_intruder[0], v_intruder[1])
            
            VelocityObstacles.append(final_poly_translated)
        
        # Combine all velocity obstacles into one figure
        CombinedObstacles = cascaded_union(VelocityObstacles)
        
        # Get minimum and maximum speed of ownship
        vmin = ownship.perf.vmin[idx]
        vmax = ownship.perf.vmax[idx]
        # Create velocity line
        v_dir = self.normalized(v_ownship)
        v_line_min = v_dir * vmin
        v_line_max = v_dir * vmax
        
        # Create velocity line
        line = LineString([v_line_min, v_line_max])

        intersection = CombinedObstacles.intersection(line)
        

        
        # For now let's just go with the slowest solution
        if intersection:
            solutions = []
            for velocity in list(intersection.coords):
                solutions.append(self.norm(velocity))
            gs_new = min(solutions)

        else:
            gs_new = ownship.ap.tas[idx]
        
        return gs_new, ownship.ap.vs[idx]
    
    def pairs(self, conf, ownship, intruder, idx):
        '''Returns the indices of conflict pairs that involve aircraft idx
        '''
        idx_pairs = np.array([], dtype = int)
        for idx_pair, pair in enumerate(conf.confpairs):
            if (ownship.id[idx] == pair[0]):
                idx_pairs = np.append(idx_pairs, idx_pair)
        return idx_pairs
    
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
    
    # def cutoff_legs(self, x, r, t):
    #     '''Gives the cutoff point of the left leg.'''
    #     # Find vector that describes radius
    #     r_vec = self.perp_left(x) * r
    #     # Find the big left leg vector
    #     left_leg = x + r_vec
    #     # Find the left leg direction
    #     left_cutoff_leg_dir = self.normalized(left_leg)
    #     # Save this for later
    #     self.left_cutoff_leg_dir = left_cutoff_leg_dir
    #     # Find the length of the left cutoff leg
    #     left_cutoff_leg = np.sqrt(self.norm_sq(x/t) - (r/t)*(r/t))
    #     # Return left cutoff vector
    #     return left_cutoff_leg * left_cutoff_leg_dir
        
    def cutoff_legs(self, x, r, t):
        '''Gives the cutoff point of the right leg.'''
        x = np.array(x)
        # First get the length of x
        x_len = self.norm(x)
        # Find the sine of the angle
        anglesin = r / x_len
        # Find the angle itself
        angle = np.arcsin(anglesin) # Radians
        
        print(angle, anglesin, x, x_len, r)
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
        return np.arccos(np.dot(a,b)/(self.norm(a) * self.norm(b)))
    
    def dist_sq(self, a, b):
        return self.norm_sq(b - a)
    
    ### Modified hdgactive function
    @property
    def hdgactive(self):
        ''' Return a boolean array sized according to the number of aircraft
            with True for all elements where heading is currently controlled by
            the conflict resolution algorithm.
        '''
        #TODO: Here is a good place to implement Open Airpace vs Restricted Airspace logic
        return np.array([False] * len(self.active))
    
    ### Modified resumenav function
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
                distance = self.norm(dist)
                dist_ok = distance > 100
                # Also check the distance between the two aircraft.

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

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not past_cpa or hor_los or is_bouncing or not dist_ok):
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