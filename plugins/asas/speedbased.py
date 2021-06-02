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
        
        # Iterate over aircraft in conflict
        for idx in list(itertools.compress(range(len(bs.traf.cr.active)), bs.traf.cr.active)):
            # Find the pairs in which IDX is involved in a conflict
            idx_pairs = self.pairs(conf, ownship, intruder, idx)
            
            # Find ORCA solution for aircraft 'idx'
            gs_new = self.SpeedBased(conf, ownship, intruder, idx, idx_pairs)
            
            # Write the new velocity of aircraft 'idx' to traffic data
            newgscapped[idx] = gs_new           
        
        # Speed based, and 2D, for now.
        vscapped       = ownship.ap.vs
        alt            = ownship.ap.alt 
        newtrack       = ownship.ap.trk
        
        return newtrack, newgscapped, vscapped, alt


    def SpeedBased(self, conf, ownship, intruder, idx, idx_pairs):
        # Extract ownship data
        v_ownship = np.array([ownship.gseast[idx], ownship.gsnorth[idx]])# [m/s]
        # Take minimum separation and multiply it with safely factor
        r = conf.rpz * self.resofach
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
            
            # TODO: Introduce proper priority.
            # For now, if aircraft is coming from the front right, then it has priority.
            if -180 <= ((qdr - ownship.trk[idx]) + 180)%360 - 180 < 0:
                # go to next pair
                continue
            
            # Let's also do some intent check
            own_intent = ownship.intent.intent[idx]
            intruder_intent = intruder.intent.intent[idx_intruder]
            # Find closest points between the two intent paths
            pown, pint = nearest_points(own_intent, intruder_intent)
            # Find the distance between the points
            point_distance = kwikdist(pown.y, pown.x, pint.y, pint.x) * nm #[m]
            
            if point_distance > r:
                continue
            
            
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
            right_leg_circle_point = self.right_cutoff_leg(x, r, t)
            left_leg_circle_point = self.left_cutoff_leg(x, r, t)

            right_leg_extended = right_leg_circle_point * t
            left_leg_extended = left_leg_circle_point * t

            triangle_poly = Polygon([right_leg_extended, right_leg_circle_point, left_leg_circle_point, left_leg_extended])

            final_poly = cascaded_union([triangle_poly, circle])
            
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
        
        return gs_new
    
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
    
    def left_cutoff_leg(self, x, r, t):
        '''Gives the cutoff point of the left leg.'''
        # Find vector that describes radius
        r_vec = self.perp_left(x) * r
        # Find the big left leg vector
        left_leg = x + r_vec
        # Find the left leg direction
        left_cutoff_leg_dir = self.normalized(left_leg)
        # Save this for later
        self.left_cutoff_leg_dir = left_cutoff_leg_dir
        # Find the length of the left cutoff leg
        left_cutoff_leg = np.sqrt(self.norm_sq(x/t) - (r/t)*(r/t))
        # Return left cutoff vector
        return left_cutoff_leg * left_cutoff_leg_dir
        
    def right_cutoff_leg(self, x, r, t):
        '''Gives the cutoff point of the right leg.'''
        # Find vector that describes radius
        r_vec = self.perp_right(x) * r
        # Find the big right leg vector
        right_leg = x + r_vec
        # Find the right leg direction
        right_cutoff_leg_dir = self.normalized(right_leg)
        # Save this for later
        self.right_cutoff_leg_dir = right_cutoff_leg_dir
        # Find the length of the right cutoff leg
        right_cutoff_leg = np.sqrt(self.norm_sq(x/t) - (r/t)*(r/t))
        # Return right cutoff vector
        return right_cutoff_leg * right_cutoff_leg_dir
                
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