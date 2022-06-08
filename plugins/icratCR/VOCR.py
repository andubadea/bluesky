from bluesky.traffic.asas import ConflictResolution
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.affinity import translate
from shapely.ops import cascaded_union, nearest_points
import bluesky as bs
import numpy as np

# This function is present in all plugins, and is used to name the plugin and
# to set the plugin specific settings.
def init_plugin():
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'VOCR',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

# For conflict resolution plugins, we subclass the ConflictResolution class.
class VOCR(ConflictResolution):
    def __init__(self):
        super().__init__()
        
    # This function is called when conflicts are detected. It is called from within
    # the simulation loop. The parameters that are passed down are conf, ownship and intruder. 
    # The conf parameter is an instance of the ConflictDetection class, and contains information
    # about the conflicts. The ownship and intruder parameters are instances of the Traffic class. 
    # They are named this way to facilitate clear naming when using the Traffic class.  
    def resolve(self, conf, ownship, intruder):
        # We make a copy of the data of the aircraft data, so that we can modify it.
        newgs = np.copy(ownship.gs)
        newvs       = np.copy(ownship.vs)
        newalt      = np.copy(ownship.alt)
        newtrack    = np.copy(ownship.trk)
        
        # The conf variable gives us all the aircraft in conflict in a bool array
        # We can iterate over all of these aircraft and make then turn right
        for idx1 in np.argwhere(conf.inconf).flatten():  
            # Find the conflict pairs in which IDX is involved in a conflict
            idx_pairs = self.pairs(conf, ownship, intruder, idx1)
            
            # Get the velocity of the aircraft
            v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1]])# [m/s]
            
            # Initialize the list of velocity obstacles
            VelocityObstacles = []
            
            # Iterate over conflict pairs
            for i, idx_pair in enumerate(idx_pairs):
                # Get the index of the intruder
                idx2 = intruder.id.index(conf.confpairs[idx_pair][1])
                
                # Get the velocity of the intruder
                v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
                
                # Get the velocity obstacle
                try:
                    velocity_obstacle = self.get_VO(conf, ownship, intruder, idx1, idx2, idx_pair)
                except:
                    velocity_obstacle = None
                
                # If the velocity obstacle is not None, add it to the list
                if velocity_obstacle is not None:
                    VelocityObstacles.append(velocity_obstacle)
                
            # It could be that we have no velocity obstacles. In that case, we can't do anything.
            if not VelocityObstacles:
                continue
                
            # Now that we have all the velocity obstacles, we can compute the ideal speed
            # Compute the combined velocity obstacles as a shapely polygon
            CombinedObstacles = cascaded_union(VelocityObstacles)
            
            # The closest point between the current velocity and the combined velocity obstacles
            # is one way to solve this. We can use the nearest_points function to find the closest
            # point.
            v_sol, _ = nearest_points(CombinedObstacles.exterior, Point(v1[0], v1[1]))
            
            # Transform this velocity in ground speed and heading
            newgs[idx1] = np.sqrt(v_sol.x**2 + v_sol.y**2)
            newtrack[idx1] = (np.arctan2(v_sol.x,v_sol.y)*180/np.pi)%360
            
        # We send the new data to the simulation.
        return newtrack, newgs, newvs, newalt
    
    def get_VO(self, conf, ownship, intruder, idx1, idx2, idx_pair):
        '''Returns the VO of aircraft idx1 and idx2'''
        t = conf.dtlookahead[idx1]
        # Get QDR and DIST of conflict
        qdr = conf.qdr[idx_pair]
        dist = conf.dist[idx_pair]
        # Get radians qdr
        qdr_rad = np.radians(qdr)
        # Get relative position
        x_rel = np.array([np.sin(qdr_rad)*dist, np.cos(qdr_rad)*dist])
        # Get the speed of the intruder
        v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2]])
        # Get cutoff legs
        left_leg_circle_point, right_leg_circle_point = self.cutoff_legs(x_rel, conf.rpz_def, t)
        # Extend cutoff legs
        right_leg_extended = right_leg_circle_point * t
        left_leg_extended = left_leg_circle_point * t
        # Get the final VO
        final_poly = Polygon([right_leg_extended, (0,0), left_leg_extended])
        # Translate it by the velocity of the intruder
        final_poly_translated = translate(final_poly, v2[0], v2[1])
        # Return
        return final_poly_translated
    
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
        
    