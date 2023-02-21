import bluesky as bs
import numpy as np
import shapely as sp
import math
import pyproj
import geopandas as gpd
import matplotlib.pyplot as plt
import copy

from shapely.geometry import LineString, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely import STRtree
from shapely.ops import split, nearest_points, linemerge, snap
from shapely.affinity import rotate
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection
from bluesky.tools import geo

"""
This detection plugin only detects intersections in paths between aircraft, and
returns a list of LineStrings representing the intersections to be used in 
training a supervised learning detection and resolution model. 
"""

def init_plugin():
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'PROJECTIONCD',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class ProjectionCD(ConflictDetection):
    def __init__(self):
        super().__init__()
        # Lookahead parameters
        self.lookahead_min = 100 #metres
        self.lookahead_max = 300 #metres
        
        # New detection parameters
        self.intent_geom = [] # Linestring of aircraft intent per pair
        self.dist_to_int = [] # Distance to intent intersections per pair
        self.num_turns = [] # Number of turns per pair
        self.mean_turn_angle = [] # Mean turn angle per pair
        self.qdr_mat = [] # QDR for all aircraft
        self.dist_mat = [] # Distance for all aircraft
        
        self.rough_geometries = []
        self.ac_leg_positions = []
        self.dlookaheads = []
        
        # Distance buffer for shapely
        self.precision = 0.001 # metres
        
        self.colors = ['yellow', 'orange', 'green', 'purple', 'pink']
        
        # Get the city centre
        try:
            self.city_centre_coords = bs.traf.TrafficSpawner.city_centre_coords
        except:
            print('City centre cannot be set, defaulting to Vienna.')
            self.city_centre_coords = [48.208758, 16.372449]
            
        # Get the UTM coordinate system
        utm_crs = self.convert_wgs_to_utm(*self.city_centre_coords)
        
        # Initialise the coordinate transformer
        self.transform_coords = pyproj.Transformer.from_crs(crs_from=4326, crs_to=utm_crs, always_xy = True)
        
        # Initialise the index
        self.geom_tree = self.create_index()
        
    def reset(self):
        self.__init__()
        pass
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self, ownship, intruder):
        # Update the geometry tree
        self.geom_tree = self.create_index()
        
        # Detect intersections
        self.confpairs, self.lospairs, self.inconf, self.dist_to_int, self.num_turns, \
        self. mean_turn_angle, self.qdr_mat, \
        self.dist_mat, self.intent_geom = self.detect(ownship, intruder)
                
        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique   
        return
    
    def detect(self, ownship, intruder):
        # Query the tree for now
        tree_query = self.geom_tree.query(self.rough_geometries, predicate = 'intersects')
        all_intersections = np.transpose(tree_query)
        
        # Problem is, the query contains self intersections
        # We want the indices where column 0 is not equal to column 1
        acidx_int_pairs = all_intersections[all_intersections[:, 0] != all_intersections[:, 1]]
        
        # Get lospairs and inconf bools
        lospairs, qdr_mat, dist_mat = self.los_detect(ownship, intruder)
        inconf = np.array([False]*ownship.ntraf)
        
        if len(acidx_int_pairs) == 0:
            return [], [], inconf, [], [], [], qdr_mat, dist_mat, []
        
        # For each confpair, we need to get the distance of each aircraft to the intersection,
        # the number of turns until the intersection, and the mean turn angle until the intersection
        # First, initialize the arrays
        conf_pairs = []
        dist_to_int = []
        num_turns = []
        mean_turn_angle = []
        intent_geom = []
        
        for j, pair in enumerate(acidx_int_pairs):
            idx1, idx2 = pair[0], pair[1]
            # Get info from the functions
            dist1, dist2, vel1, vel2, intent1, intent2, int_point, is_conf = self.intersection_info(idx1, idx2)
            if not is_conf:
                continue
            inconf[idx1] = True
            inconf[idx2] = True
            # num_turns1, num_turns2, mean_turn_angle1, mean_turn_angle2 = self.turn_info(idx1, idx2, intent1, intent2)
            
            # Assign the values
            dist_to_int.append([dist1, dist2])
            #num_turns[j] = [num_turns1, num_turns2]
            #mean_turn_angle[j] = [mean_turn_angle1, mean_turn_angle2]
            intent_geom.append([intent1, intent2])
            conf_pairs.append((bs.traf.id[pair[0]], bs.traf.id[pair[1]]))

            if True:
                print('----------------------------------------------------------------')
                print(bs.traf.id[idx1], bs.traf.id[idx2])
                print(dist1, dist2)
                print(vel1, vel2)
                plt.figure('intersection')
                plt.plot(intent1.coords.xy[1], intent1.coords.xy[0], color = 'red')
                plt.plot(intent2.coords.xy[1], intent2.coords.xy[0], color = 'blue')
                plt.scatter(int_point.y, int_point.x, color = 'green')
                plt.scatter(self.ac_leg_positions[idx1].y,self.ac_leg_positions[idx1].x, marker = 'x', color = 'red')
                plt.scatter(self.ac_leg_positions[idx2].y,self.ac_leg_positions[idx2].x, marker = 'x', color = 'blue')
                ax = plt.gca()
                ax.set_aspect('equal', adjustable = 'box')
                plt.show(block = True)
                
        for los_pair in lospairs:
            if los_pair not in conf_pairs:
                print(los_pair)

        return conf_pairs, lospairs, inconf, dist_to_int, num_turns, mean_turn_angle, qdr_mat, dist_mat, intent_geom
    
    def intersection_info(self, idx1, idx2):
        """Function that outputs information about the intersection between the intents of two
        aircraft, depending on the type of intersection.
        
        As we are currently checking rough intersections, then we have several cases to consider:
        Case 1: Intersection is a MultiLineString, it can most probably just be merged in a single LineString
        Case 2: Intersection is a Point
        Case 3: Intersection is a LineString
        """
        # Get the geometries of the aircraft
        intent1 = self.rough_geometries[idx1]
        intent2 = self.rough_geometries[idx2]
        
        # Get the intersection point between the two intents
        intersection = intent1.intersection(intent2)
        
        if isinstance(intersection, MultiLineString):
            # Merge the line
            intersection = linemerge(intersection)
        
        if isinstance(intersection, Point):
            # Either a type 1 or a type 2 intersection
            return self.handle_point_intersection(idx1, idx2, intent1, intent2, intersection)

        if isinstance(intersection, LineString):
            # Either a type 3 or a type 4 intersection
            return self.handle_line_intersection(idx1, idx2, intent1, intent2, intersection)
        
        if isinstance(intersection, MultiPoint):
            print('MULTIPOINT')
            return 0, 0, 0, 0, intent1, intent2, intersection, False

        if isinstance(intersection, GeometryCollection):
            print('GEOMETRY COLLECTION')
            return 0, 0, 0, 0, intent1, intent2, intersection, False
    
    def handle_point_intersection(self, idx1, idx2, intent1, intent2, intersection):
        '''There are two types of point intersections:
        - Type 1:   If point is the first point or back point of either intent, then it means that the
                    intents are barely touching and the intents are 99.99% certain to be too long.
        - Type 2:   If the above isn't the case, then it is a normal intersection. Process
                    the rough geometries into accurate ones.
        '''

        int_point = intersection # Rename to int_point to make it clearer
        # Check if it's a type 1:
        if (int_point.x, int_point.y) == intent1.coords[0] or \
            (int_point.x, int_point.y) == intent2.coords[0] or \
            (int_point.x, int_point.y) == intent1.coords[-1] or \
            (int_point.x, int_point.y) == intent2.coords[-1]:
                # This is a type 1 intersection, ignore it, not a conflict (yet).
                return 0, 0, 0, 0, intent1, intent2, intersection, False
        else:
            # This is a type 2 intersection. Let's process the information and the intents
            # into accurate ones.
            # There is no way in which this intersection is not in front of both aircraft.
            # If one of the aircraft would already be past the intersection, and this is not
            # caught by the previous if statement, then the intersection should be a linestring.
            # Thus, handle it as a front intersection.
            # We split the two rough intent lines into two geometries
            intent1_back, intent1_front = self.cut_line_with_point(intent1, self.ac_leg_positions[idx1])
            intent2_back, intent2_front = self.cut_line_with_point(intent2, self.ac_leg_positions[idx2])
            
            # These two lines are probably too long, so we need to clip them
            # First, the front lines. Check if they intersection is too far away
            if intent1_front.project(int_point) > self.dlookaheads[idx1] or \
                intent2_front.project(int_point) > self.dlookaheads[idx2]:
                # The intersection is too far away, not a conflict (yet)
                return 0, 0, 0, 0, intent1, intent2, intersection, False
            else:
                # We need to cut the linestrings at the intersection points
                intent1_front_cut, _ = self.cut_line_with_point(intent1_front, int_point)
                intent2_front_cut, _ = self.cut_line_with_point(intent2_front, int_point)
                
                # Compute the distance to the intersection point
                dist1 = intent1_front_cut.length
                dist2 = intent2_front_cut.length
                
                # The velocities are both positive as we're heading towards the intersection
                vel1 = bs.traf.gs[idx1]
                vel2 = bs.traf.gs[idx2]
            
                # The intent is just the front part of the lines.
                # Return stuff
                return dist1, dist2, vel1, vel2, intent1_front_cut, intent2_front_cut, int_point, True
                
        
    def handle_line_intersection(self, idx1, idx2, intent1, intent2, intersection):
        ''' If we have a linestring intersection, then we can have the following types:
        - Type 1:   The aircraft are merging within the same point. We can check this by 
                    determining whether any of the aircraft is on (or veeeery close) to the
                    intersection line, and if their back intents do not intersect.
        - Type 2:   The aircraft are one behind the other. In this case, we can create a linestring from
                    the  two intents and determine which aircraft is in front.
        - Type 3:   The intersection is between the aircraft. This means that one of the aircraft is 
                    past the intersection point, but their back extension is still within the intersection line.
                    To determine if this is the case, we determine if the back line of an aircraft yields the
                    same intersection. If so, then we have this case.'''
        
        # First of all, we can easily check if it's a type 2 intersection
        ac1_pos = self.ac_leg_positions[idx1]
        ac2_pos = self.ac_leg_positions[idx2]
        
        # Get the back and the front of the intents. Careful with these as the aircraft
        # points do not lie on the routes themselves because of floating point errors.
        intent1_back, intent1_front = self.cut_line_with_point(intent1, self.ac_leg_positions[idx1])
        intent2_back, intent2_front = self.cut_line_with_point(intent2, self.ac_leg_positions[idx2])
        
        # Cut the back intents to dimension
        # Remember these two are reversed, the first waypoint is the actual position of the aircraft
        intent1_back_cut= self.cut(LineString(intent1_back.coords[::-1]), self.rpz_def)
        intent2_back_cut= self.cut(LineString(intent2_back.coords[::-1]), self.rpz_def)
        
        # We need to cut the front intents to dimension
        intent1_front_cut= self.cut(intent1_front, self.dlookaheads[idx1])
        intent2_front_cut= self.cut(intent2_front, self.dlookaheads[idx2])
        
        # Now we can check whether an aircraft is in front of another aircraft.
        ac1_in_front = intent2_front_cut.distance(intent1_back_cut) < self.precision
        ac2_in_front = intent1_front_cut.distance(intent2_back_cut) < self.precision
            
        if ac1_in_front:
            # Aircraft 1 is in front.
            # The intersection point is aircraft 1 itself. 
            int_point = ac1_pos
            
            # We need to cut the linestring of the aircraft in the back at the intersection point
            intent2_front_cut, _ = self.cut_line_with_point(intent2_front, int_point)
            
            # If the length of this guy is greater than the lookahead distance, we don't have a conflict yet
            if intent2_front_cut.length > self.dlookaheads[idx2]:
                return 0, 0, 0, 0, intent1, intent2, intersection, False

            # The intent of aircraft 1 is the front part of the line cut at lookahead distance
            intent1_front_cut = self.cut(intent1_front, self.dlookaheads[idx1])
            
            # The velocity and distance of aircraft 1 is 0
            vel1 = 0
            vel2 = bs.traf.gs[idx2] - bs.traf.gs[idx1]
            dist1 = 0
            dist2 = intent2_front_cut.length
        
            # Return info
            return dist1, dist2, vel1, vel2, intent1_front_cut, intent2_front_cut, int_point, True
        
        elif ac2_in_front:
            # Either they are perfectly equal (very unlikely), or ac2 is in front of ac1.
            # The intersection point is aircraft 2 itself. 
            int_point = ac2_pos
            
            # We need to cut the linestring of the aircraft in the back at the intersection point
            intent1_front_cut, _ = self.cut_line_with_point(intent1_front, int_point)
            
            # If the length of this guy is greater than the lookahead distance, we don't have a conflict yet
            if intent1_front_cut.length > self.dlookaheads[idx1]:
                return 0, 0, 0, 0, intent1, intent2, intersection, False

            # The intent of aircraft 2 is the front part of the line cut at lookahead distance
            intent2_front_cut = self.cut(intent2_front, self.dlookaheads[idx2])
            
            # The velocity and distance of aircraft 2 is 0
            vel1 = bs.traf.gs[idx1] - bs.traf.gs[idx2]
            vel2 = 0
            dist1 = intent1_front_cut.length
            dist2 = 0
            
            # Return info
            return dist1, dist2, vel1, vel2, intent1_front_cut, intent2_front_cut, int_point, True
            
        else:
            # Aircraft are either diverging or merging.
            # We can check this by seeing if the first point of the intersection point is on the front
            # of the intents or the back of the intents for each aircraft.
            # We can see if we're diverging if the front of the intents do not intersect.
            diverging = intent1_front.intersects(intent2_front)
            
            # We can also check if 
            int_in_front1 = intersection.coords[0] in list(intent1_front.coords)
            int_in_front2 = intersection.coords[0] in list(intent2_front.coords)
            int_in_back1 = intersection.coords[0] in list(intent1_back.coords)
            int_in_back2 = intersection.coords[0] in list(intent2_back.coords)
            
            if int_in_front1 and int_in_front2:
                # Intersection is just a point in front of both these aircraft, we can use the other function to
                # compute it.
                int_point = Point(intersection.coords[0])
                # Just run the point intersection function
                return self.handle_point_intersection(idx1, idx2, intent1, intent2, int_point)
            
            elif int_in_back1 and int_in_back2:
                # We are past the intersection point, so no conflict here
                return 0, 0, 0, 0, intent1, intent2, intersection, False
            
            elif diverging:
                # We are past the intersection point, so no conflict here
                return 0, 0, 0, 0, intent1, intent2, intersection, False
            
            elif int_in_front1 and int_in_back2:
                # This can happen if one aircraft (ac2) is past the intersection point and turned already, however
                # the rpz back extension is still intersecting with the intent of the previous aircraft (ac1). 
                # In this case, we can set the intersection point at the divergence point itself (thus, the last)
                # point in the intersection linetring.
                int_point = Point(intersection.coords[-1])
                # Now we calculate things as normal, with the inclusion of the clipped back line in the
                # intent of the second aircraft that is past the intersection point.
                intent1_front_cut, _ = self.cut_line_with_point(intent1_front, int_point)
                
                # If the length of this is greater than the lookahead distance, we don't have a conflict
                if intent1_front_cut.length > self.dlookaheads[idx1]:
                    return 0, 0, 0, 0, intent1, intent2, intersection, False

                # From the second aircraft we need both back intent and front intent
                # First invert the back line
                intent2_back_inverted = LineString(intent2_back.coords[::-1])
                # Then cut it
                intent2_back_cut_inverted, _ = self.cut_line_with_point(intent2_back_inverted, int_point)
                # Then reverse it back
                intent2_back_cut = LineString(intent2_back_cut_inverted.coords[::-1])
                
                # Check if this is shorter than rpz/2, if so, we don't have a conflict
                if intent2_back_cut.length < self.rpz_def:
                    return 0, 0, 0, 0, intent1, intent2, intersection, False
                
                # Get the front intent of aircraft 2
                intent2_front_cut = self.cut(intent2_front, self.dlookaheads[idx2])
                
                # Merge the two intents
                intent2_cut = linemerge([intent2_back_cut, intent2_front_cut])
                
                # AC1 is going towards intersection, AC2 is going away from intersection
                vel1 = bs.traf.gs[idx1]
                vel2 = -bs.traf.gs[idx2]
                dist1 = intent1_front_cut.length
                dist2 = -intent2_back_cut.length
                
                # Return info
                return dist1, dist2, vel1, vel2, intent1_front_cut, intent2_cut, int_point, True
            
            elif int_in_front2 and int_in_back1:
                # Previous situation but the other way around. Aircraft 1 is past the intersection point already.
                int_point = Point(intersection.coords[-1])
                # Now we calculate things as normal, with the inclusion of the clipped back line in the
                # intent of the second aircraft that is past the intersection point.
                intent2_front_cut, _ = self.cut_line_with_point(intent2_front, int_point)
                
                # If the length of this is greater than the lookahead distance, we don't have a conflict
                if intent2_front_cut.length > self.dlookaheads[idx2]:
                    return 0, 0, 0, 0, intent1, intent2, intersection, False

                # From the second aircraft we need both back intent and front intent
                # First invert the back line
                intent1_back_inverted = LineString(intent1_back.coords[::-1])
                # Then cut it
                intent1_back_cut_inverted, _ = self.cut_line_with_point(intent1_back_inverted, int_point)
                # Then reverse it back
                intent1_back_cut = LineString(intent1_back_cut_inverted.coords[::-1])
                
                # Check if this is shorter than rpz, if so, we don't have a conflit
                if intent1_back_cut.length < self.rpz_def:
                    return 0, 0, 0, 0, intent1, intent2, intersection, False
                
                # Get the front intent of aircraft 2
                intent1_front_cut = self.cut(intent1_front, self.dlookaheads[idx1])
                
                # Merge the two intents
                intent1_cut = linemerge([intent1_back_cut, intent1_front_cut])
                
                # AC2 is going towards intersection, AC1 is going away from intersection
                vel1 = -bs.traf.gs[idx1]
                vel2 = bs.traf.gs[idx2]
                dist1 = -intent1_back_cut.length
                dist2 = intent2_front_cut.length
                
                # Return info
                return dist1, dist2, vel1, vel2, intent1_cut, intent2_front_cut, int_point, True
            
            else:
                # Something is wrong.
                print('AAAAAAAAAAAAAA')
                return 0, 0, 0, 0, intent1, intent2, intersection, False
    
    
    def turn_info(self, idx1, idx2, intent1, intent2):
        """Function that calculates the number of turns to the intersection between two aircraft.
        """
        # Not all points within the linestring are turns, only if the angle is greater than 25 degrees.
        # Thus, get all the angles above an absolute value of 25 degrees.
        # We can determine this stuff by checking the turn waypoints of aircraft 
        # and whether they belong to the line segment between the aircraft and the 
        # intersection.
        
        # Get the routes
        acrte1 = bs.traf.ap.route[idx1]
        acrte2 = bs.traf.ap.route[idx2]
        
        # Active waypoints
        act_wp1 = acrte1.iactwp
        act_wp2 = acrte2.iactwp
        
        # Turn waypoints
        turnidx1 = np.where(acrte1.wpflyturn)[0]
        turnidx2 = np.where(acrte2.wpflyturn)[0]
        
        # We can determine if there are any turn waypoints between the aircraft and the intersection
        # by simply finding out the index of the waypoint right before the intersection and seeing
        # if any waypoints in betweeen are turn waypoints.
        
        # Get the index of the waypoint right before the intersection
        wptidx_b4_int1 = act_wp1 + (len(split1.coords) - 2)
        wptidx_b4_int2 = act_wp2 + (len(split2.coords) - 2)
        
        # Calculate the number of turns
        num_turns_1 = np.sum(np.logical_and(turnidx1 > act_wp1, turnidx1 < wptidx_b4_int1))
        num_turns_2 = np.sum(np.logical_and(turnidx2 > act_wp2, turnidx2 < wptidx_b4_int2))
        
        # Now that we know the number of turns, calculate the mean turn angle
        # We need to for loop through all the turn waypoints for each aircraft
        angles1 = []
        for i_turn in turnidx1[np.logical_and(turnidx1 > act_wp1, turnidx1 < wptidx_b4_int1)]:
            if not (i_turn < len(acrte1.wplon)-1):
                continue
            # Get the needed stuff
            lat_cur, lon_cur   = acrte1.wplat[i_turn], acrte1.wplon[i_turn]
            lat_prev, lon_prev = acrte1.wplat[i_turn-1], acrte1.wplon[i_turn-1]
            lat_next, lon_next = acrte1.wplat[i_turn+1], acrte1.wplon[i_turn+1]
            
            # Get the angle
            d1=geo.kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=geo.kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                angles1.append(angle)
            else:
                continue
            
        angles2 = []
        for i_turn in turnidx2[np.logical_and(turnidx2 > act_wp2, turnidx2 < wptidx_b4_int2)]:
            if not (i_turn < len(acrte2.wplon)-1):
                continue
            # Get the needed stuff
            lat_cur, lon_cur   = acrte2.wplat[i_turn], acrte2.wplon[i_turn]
            lat_prev, lon_prev = acrte2.wplat[i_turn-1], acrte2.wplon[i_turn-1]
            lat_next, lon_next = acrte2.wplat[i_turn+1], acrte2.wplon[i_turn+1]
            
            # Get the angle
            d1=geo.kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=geo.kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                angles2.append(angle)
            else:
                continue
            
        avg_angle1 = np.average(angles1) if len(angles1) > 0 else 0
        avg_angle2 = np.average(angles2) if len(angles2) > 0 else 0
        
        # Return the number of turns
        return num_turns_1, num_turns_2, avg_angle1, avg_angle2
           
    def create_index(self):
        """Function that creates the geometric tree index. 
        First process the current routes of aircraft, transform them into linestrings,
        add a backwards extension for 0.5 * rpz and create the index."""
        self.rough_geometries = []
        self.ac_leg_positions = []
        self.dlookaheads = []
        # We basically need to loop through all aircraft routes
        for acidx, acrte in enumerate(bs.traf.ap.route):
            # Pass this aircraft if it doesn't have a route
            if len(acrte.wplat) == 0:
                continue
            
            # Get active waypoint
            act_wp = acrte.iactwp
            
            if act_wp < 0:
                # Weird thing
                continue
            
            # Find the position of the aircraft on the current leg
            if act_wp > 0:
                # Dunno when this wouldn't be the case but oh well
                # First, get the transformed coordinates of the current leg
                leg_lon_0, leg_lat_0 = self.transform_coords.transform(acrte.wplon[act_wp-1], acrte.wplat[act_wp-1])
                leg_lon_1, leg_lat_1 = self.transform_coords.transform(acrte.wplon[act_wp], acrte.wplat[act_wp])
                # Get the transformed coordinates of the current position
                ac_lon_utm, ac_lat_utm = self.transform_coords.transform(bs.traf.lon[acidx],bs.traf.lat[acidx])
                # Create the linestring
                current_leg = LineString([[leg_lon_0, leg_lat_0], 
                                          [leg_lon_1, leg_lat_1]])
                # Get the position of the aircraft on the currrent leg
                ac_leg_pos, _ = nearest_points(current_leg, Point([ac_lon_utm, ac_lat_utm]))
            else:
                # We're before the first waypoint
                ac_lon_utm, ac_lat_utm = self.transform_coords.transform(bs.traf.lon[acidx],bs.traf.lat[acidx])
                ac_leg_pos = Point([ac_lon_utm, ac_lat_utm])
                
            # We want to create two linestrings:
            # 1: LineString with length at least 0.5 * rpz before ac position
            # 2: LineString with length at least lookaheaddist in front of ac position
                
            # Convert the route coordinates we need to UTM
            # Convert the coordinates of the route to UTM
            after_rte_utm_lon, after_rte_utm_lat = self.transform_coords.transform(acrte.wplon[act_wp:], acrte.wplat[act_wp:])
            b4_rte_utm_lon, b4_rte_utm_lat = self.transform_coords.transform(acrte.wplon[:act_wp], acrte.wplat[:act_wp])
            
            # -------------- FROM HERE ONWARDS WE DO LAT/LON -------------------
            # We want to create two linestrings:
            # 1: LineString with length at least 0.5 * rpz before ac position
            # 2: LineString with length at least lookaheaddist in front of ac position
            # These two linestrings will be merged to create the rough intent
            
            # Linestring number 1, the before
            # First, create the linestring that includes ac position
            b4_linestring_w_ac = LineString([[ac_leg_pos.y, ac_leg_pos.x], *zip(b4_rte_utm_lat[::-1], b4_rte_utm_lon[::-1])])
            # Get the linestring that covers at least 0.5*rpz
            b4_linestring_w_ac = self.cut_at_coord(b4_linestring_w_ac, self.rpz_def)
            
            # Now create the linestring that covers the after
            after_linestring_w_ac = LineString([[ac_leg_pos.y, ac_leg_pos.x], *zip(after_rte_utm_lat, after_rte_utm_lon)])
            # Cut this one as well
            dlookahead = bs.traf.gs[acidx] * self.dtlookahead_def
            dlookahead = max(min(dlookahead, self.lookahead_max), self.lookahead_min)
            after_linestring_w_ac = self.cut_at_coord(after_linestring_w_ac, dlookahead)
            
            # Now combine these two linestrings and exclude the aircraft coordinate
            rough_intent_LS = LineString([*b4_linestring_w_ac.coords[1:][::-1], *after_linestring_w_ac.coords[1:]])
            
            # Add this as the intent
            self.rough_geometries.append(rough_intent_LS)
            self.ac_leg_positions.append(Point(ac_leg_pos.y, ac_leg_pos.x))
            self.dlookaheads.append(dlookahead)
            
        # Create the index
        return STRtree(self.rough_geometries)
    
    def los_detect(self, ownship, intruder):
        ''' Intrusion between ownship (traf) and intruder (traf/adsb).'''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I
        swlos = (dist < (np.zeros(len(self.rpz)) + self.rpz)) * (np.abs(dalt) < self.hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return lospairs, qdr, dist
        
    
    def convert_wgs_to_utm(self, lat: float, lon: float):
        """Based on lat and lng, return best utm epsg-code"""
        utm_band = str((math.floor((lon + 180) / 6 ) % 60) + 1)
        if len(utm_band) == 1:
            utm_band = '0'+utm_band
        if lat >= 0:
            epsg_code = '326' + utm_band
            return epsg_code
        epsg_code = '327' + utm_band
        return epsg_code   
        
    def cut(self, line, distance):
        # Cuts a line in two at a distance from its starting point
        if distance <= 0.0 or distance >= line.length:
            return LineString(line)
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == distance:
                return LineString(coords[:i+1])
            if pd > distance:
                cp = line.interpolate(distance)
                return LineString(coords[:i] + [(cp.x, cp.y)])
            
    def cut_at_coord(self, line, distance):
        # Cuts a line in two at a point already within the line
        # that guarantees the length of the new line is greater than distance
        if distance <= 0.0 or distance >= line.length:
            return LineString(line)
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd >= distance:
                return LineString(coords[:i+1])
            
    def cut_line_with_point(self, line, splitter):
        """Split a LineString with a Point
        Code borrowed from shapely
        """

        # point is on line, get the distance from the first point on line
        distance_on_line = line.project(splitter)

        if distance_on_line == 0:
            return LineString([]), line.simplify(0)
        elif distance_on_line == line.length:
            return line.simplify(0), LineString([])

        coords = list(line.coords)
        # split the line at the point and create two new lines
        current_position = 0.0
        for i in range(len(coords)-1):
            point1 = coords[i]
            point2 = coords[i+1]
            dx = point1[0] - point2[0]
            dy = point1[1] - point2[1]
            segment_length = (dx ** 2 + dy ** 2) ** 0.5
            current_position += segment_length
            if distance_on_line == current_position:
                # splitter is exactly on a vertex

                # now check if the splitter is at the start of the line
                if len(coords[i+1:]) == 1:
                    return LineString(coords[:i+2]).simplify(0), LineString([])

                # now check if the splitter is at the end of the line
                if len(coords[:i+2]) == 1:
                    return LineString([]), LineString(coords[i+1:]).simplify(0)

                # otherwise it is normal split
                return LineString(coords[:i+2]).simplify(0), LineString(coords[i+1:]).simplify(0)
            elif distance_on_line < current_position:
                # splitter 
                # is between two vertices
                return LineString(coords[:i+1] + [splitter.coords[0]]).simplify(0), LineString([splitter.coords[0]] + coords[i+1:]).simplify(0)

    