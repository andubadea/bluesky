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
This plugin also logs the data.
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
        self.los_detected = []
        
        self.rough_geometries = []
        self.ac_leg_positions = []
        self.dlookaheads = []
        
        self.fo = []
        
        # Distance buffer for shapely
        self.precision = 0.0001 # metres
        
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
        super.reset()
        return
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self, ownship, intruder):
        # Update the geometry tree
        self.geom_tree = self.create_index()
        
        # Detect intersections
        self.confpairs, self.lospairs, self.inconf, self.dist_to_int, self.num_turns, \
        self.mean_turn_angle, self.qdr_mat, \
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
        confpairs_s, lospairs, inconf_s, tcpamax_s, qdr_s, \
            dist_s, dcpa_s, tcpa_s, tLOS_s, qdr_mat, dist_mat = \
                self.sb_detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)
        self.los_detected = lospairs
        inconf = np.array([False]*ownship.ntraf)
        
        if len(acidx_int_pairs) == 0:
            return [], [], inconf, [], [], [], qdr_mat, dist_mat, []
        
        # For each confpair, we need to get the distance of each aircraft to the intersection,
        # the number of turns until the intersection, and the mean turn angle until the intersection
        # First, initialize the arrays
        conf_pairs = []
        dist_to_int = []
        velocity_wrt_int = []
        num_turns = []
        mean_turn_angle = []
        intent_geom = []
        
        for j, pair in enumerate(acidx_int_pairs):
            idx1, idx2 = pair[0], pair[1]
            # Get info from the functions
            dist1, dist2, vel1, vel2, intent1, intent2, int_point, is_conf = self.intersection_info(idx1, idx2)
            
            if not is_conf and pair in confpairs_s:
                print(pair)
            if not is_conf:
                # Check if state-based detects a conflict
                if pair in confpairs_s:
                    # This is a statebased conflict, we can store normal statebased information
                    # We take the CPA as the intersection point
                    # Distance to intersection is just tcpa times the velocity of the aircraft
                    dist_to_int.append([tcpa_s[j] * bs.traf.gs[idx1], tcpa_s[j] * bs.traf.gs[idx2]])
                    # Number of turns is just 0
                    num_turns[j] = [0,0]
                    mean_turn_angle[j] = [0,0]
                    # Append the intent geometry why not
                    intent_geom.append([intent1, intent2])
                    # We still set inconf as true
                    inconf[idx1] = True
                    inconf[idx2] = True
                    # We also append the conf_pair
                    conf_pairs.append((bs.traf.id[pair[0]], bs.traf.id[pair[1]]))
                    continue
                else:
                    # Not a statebased conflict either, skip this pair.
                    continue
                
            else:
                # If we are here, then there is am intent-based conflict
                inconf[idx1] = True
                inconf[idx2] = True
                
                # There is a rare situation in which the aircraft are right on top of each other. Handle it here:
                ac1_pos = self.ac_leg_positions[idx1]
                ac2_pos = self.ac_leg_positions[idx2]
                if (ac1_pos.x, ac1_pos.y) == (ac2_pos.x, ac2_pos.y):
                    num_turns1, num_turns2, mean_turn_angle1, mean_turn_angle2 = 0, 0, 0, 0
                else:
                    num_turns1, num_turns2, mean_turn_angle1, mean_turn_angle2 = self.turn_info(idx1, idx2, intent1, intent2, dist1, dist2)

                # Check everything manually
                print('---------------------------------------------------------------')
                print(f'{bs.traf.id[idx1]} and {bs.traf.id[idx2]}')
                print('')
                print(f'Dist {bs.traf.id[idx1]}: {dist1}')
                print(f'Velo {bs.traf.id[idx1]}: {vel1}')
                print(f'Numt {bs.traf.id[idx1]}: {num_turns1}')
                print(f'Mang {bs.traf.id[idx1]}: {mean_turn_angle1}')
                print('')
                print(f'Dist {bs.traf.id[idx2]}: {dist2}')
                print(f'Velo {bs.traf.id[idx2]}: {vel2}')
                print(f'Numt {bs.traf.id[idx2]}: {num_turns2}')
                print(f'Mang {bs.traf.id[idx2]}: {mean_turn_angle2}')
                plt.figure('intersection', figsize=(8, 8))
                plt.plot(intent1.coords.xy[1], intent1.coords.xy[0], color = 'blue')
                plt.plot(intent2.coords.xy[1], intent2.coords.xy[0], color = 'red')
                plt.scatter(int_point.y, int_point.x, color = 'green', label = 'int')
                plt.scatter(self.ac_leg_positions[idx1].y,self.ac_leg_positions[idx1].x, marker = 'x', color = 'blue', label = bs.traf.id[idx1])
                plt.scatter(self.ac_leg_positions[idx2].y,self.ac_leg_positions[idx2].x, marker = 'x', color = 'red', label = bs.traf.id[idx2])
                rpz_circle_1 = self.ac_leg_positions[idx1].buffer(16).exterior
                rpz_circle_2 = self.ac_leg_positions[idx2].buffer(16).exterior
                plt.plot(rpz_circle_1.xy[1], rpz_circle_1.xy[0], color = 'blue')
                plt.plot(rpz_circle_2.xy[1], rpz_circle_2.xy[0], color = 'red')
                plt.legend()
                ax = plt.gca()
                ax.set_aspect('equal', adjustable = 'box')
                plt.show(block = True)
                
                # Assign the values
                dist_to_int.append([dist1, dist2])
                velocity_wrt_int.append([vel1, vel2])
                num_turns.append([num_turns1, num_turns2])
                mean_turn_angle.append([mean_turn_angle1, mean_turn_angle2])
                intent_geom.append([intent1, intent2])
                conf_pairs.append((bs.traf.id[pair[0]], bs.traf.id[pair[1]]))

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
            # When this happens
            # 1. The intents actually intersect in two points, for example in a roundabout where the back intents of
            # each aircraft intersect with the front intents of each aircraft - solvable by checkcing if the points
            # in the multipoint are either the first or last points of the intent lines.
            # 2. There is a proper intersection but also the back legs of the aircraft intersect.
            
            # Thus, check if the points are either the first or last points of the intent lines.
            for p in intersection.geoms:
                if (p.x, p.y) == intent1.coords[0] or \
                    (p.x, p.y) == intent2.coords[0] or \
                    (p.x, p.y) == intent1.coords[-1] or \
                    (p.x, p.y) == intent2.coords[-1]:
                    # Ignore this point
                    continue
                else:
                    # We found a proper intersection, handle it as point
                    return self.handle_point_intersection(idx1, idx2, intent1, intent2, p)
            # If we are out of the for loop, that means that this multipoint is not a conflict
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
        
        # We split the two rough intent lines into two geometries
        intent1_back, intent1_front = self.cut_line_with_point(intent1, self.ac_leg_positions[idx1])
        intent2_back, intent2_front = self.cut_line_with_point(intent2, self.ac_leg_positions[idx2])
        
        # Check if the intersection is in the back
        int_in_back1 = int_point.distance(intent1_back) < self.precision
        int_in_back2 = int_point.distance(intent2_back) < self.precision
        
        # Check if it's a type 1:
        if (int_point.x, int_point.y) == intent1.coords[0] or \
            (int_point.x, int_point.y) == intent2.coords[0] or \
            (int_point.x, int_point.y) == intent1.coords[-1] or \
            (int_point.x, int_point.y) == intent2.coords[-1]:
                return 0, 0, 0, 0, intent1, intent2, intersection, False
            
        elif int_in_back1 or int_in_back2:
            # We're already past the intersection point
            return 0, 0, 0, 0, intent1, intent2, intersection, False
        
        else:
            # This is a type 2 intersection. Let's process the information and the intents
            # into accurate ones.
            # There is no way in which this intersection is not in front of both aircraft.
            # If one of the aircraft would already be past the intersection, and this is not
            # caught by the previous if statement, then the intersection should be a linestring.
            # Thus, handle it as a front intersection.
            
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
        
        # In very rare cases, the aircraft are on the same point. We can tackle this here:
        if (ac1_pos.x, ac1_pos.y) == (ac2_pos.x, ac2_pos.y):
            # The aircraft are on the exact same point. Still a conflict, but nobody is in
            # front or in back. This is one of the rare cases where we return 0 for everything
            # but it is still a conflict.
            int_point = ac1_pos
            return 0, 0, 0, 0, intent1, intent2, int_point, True
        
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
        ac1_in_front = intent2_front.distance(self.ac_leg_positions[idx1]) < self.precision
        ac2_in_front = intent1_front.distance(self.ac_leg_positions[idx2]) < self.precision
            
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
    
            return dist1, dist2, vel1, vel2, intent1_front_cut, intent2_front_cut, int_point, True
        
        elif ac2_in_front:
            # Either they are perfectly equal (very unlikely), or ac2 is in front of ac1.
            # The intersection point is aircraft 2 itself. 
            int_point = ac2_pos
            
            # We need to cut the linestring of the aircraft in the back at the intersection point
            intent1_front_cut, _ = self.cut_line_with_point(intent1_front, int_point)
            
            # If the length of this guy is greater than the lookahead distance, we don't have a conflict yet
            if intent1_front_cut.length > self.dlookaheads[idx1]:
                # There can still be a LOS here if the intersection geometry is super weird.
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
            diverging = not intent1_front.intersects(intent2_front)
            
            # We can also check if the first point of the intersection line is in the back
            int_in_front1 = intersection.coords[0] in list(intent1_front.coords)
            int_in_front2 = intersection.coords[0] in list(intent2_front.coords)
            int_in_back1 = intersection.coords[-1] in list(intent1_back.coords)
            int_in_back2 = intersection.coords[-1] in list(intent2_back.coords)         
            
            if int_in_front1 and int_in_front2:
                # Intersection is just a point in front of both these aircraft, we can use the other function to
                # compute it.
                int_point = Point(intersection.coords[0])
                # Just run the point intersection function
                return self.handle_point_intersection(idx1, idx2, intent1, intent2, int_point)
            
            elif int_in_back1 and int_in_back2:
                # We are past the intersection point, so no conflict here
                # However, a LOS can still exist, we might need to apply state-based here.
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
    
    
    def turn_info(self, idx1, idx2, intent1, intent2, dist1, dist2):
        """Function that calculates the number of turns to the intersection between two aircraft.
        """
        # We get two intents and an intersection point. We need to determine how many turns
        # Each aircraft has until the intersection point. This intersection point can be:
        # - a point along the routes of each aircraft
        # - the exact position of one of the aircraft
        # - One aircraft is already past the intersection point.
        # We can determine this from the sign of the distance to the intersection point.
        # If both distances are positive, then we can get the number of turns for both.
        # If one of the distances is 0, then that aircraft itself is the intersection point and thus it has 0
        # turns to the intersection point, we only get the number of turns for the other aircraft.
        # Same if an aircraft has a negatie distance, it means it is already past the intersection point. 
        
        if dist1 > 0 and dist2 > 0:
            # Intersection point is in front of both aircraft.
            num_turns_1, avg_angle_1 = self.get_ac_turn_info(idx1, intent1)
            num_turns_2, avg_angle_2 = self.get_ac_turn_info(idx2, intent2)
            
        elif dist1 <= 0 and dist2 > 0:
            # Set stuff to 0 for aircraft 1 and get the correct stuff for aircraft 2
            num_turns_1 = 0
            avg_angle_1 = 0
            num_turns_2, avg_angle_2 = self.get_ac_turn_info(idx2, intent2)
        elif dist1 > 0 and dist2 <= 0:
            # Set stuff to 0 for aircraft 2 and get the correct stuff for aircraft 1
            num_turns_1, avg_angle_1 = self.get_ac_turn_info(idx1, intent1)
            num_turns_2 = 0
            avg_angle_2 = 0
        else:
            # We probably shouldn't be in this function at all then? Probably not a conflict.
            print('BBBBBBBBBBBB')
            print(dist1, dist2)
            num_turns_1 = 0
            avg_angle_1 = 0
            num_turns_2 = 0
            avg_angle_2 = 0
        
        # Return the info
        return num_turns_1, num_turns_2, avg_angle_1, avg_angle_2
    
    def get_ac_turn_info(self, acidx, intent):
        '''The intent this function uses needs to be clipped at the intersection point. It also assumes
        that the intersection point is in front of the aircraft.'''
        # Get route
        acrte = bs.traf.ap.route[acidx]
        # Get active waypoint
        act_wp = acrte.iactwp
        # Turn waypoints
        turnidx = np.where(acrte.wpflyturn)[0]
        # Get the route in UTM coordinate
        rte_lon_utm, rte_lat_utm = self.transform_coords.transform(acrte.wplon, acrte.wplat)
        rte_utm = list(zip(rte_lat_utm, rte_lon_utm))
        # Get the indices of the waypoints within the intent
        intent_turn_idx = []
        for intent_wp in intent.coords[1:-1]:
            if intent_wp in rte_utm and rte_utm.index(intent_wp) in turnidx:
                intent_turn_idx.append(rte_utm.index(intent_wp))
        
        
        # If the intex  of this waypoint is smaller or equal to the active waypoint index,
        # then there are no turns.
        if not intent_turn_idx:
            return 0, 0
        # Get the number of turns
        num_turns= len(intent_turn_idx)
        
        if num_turns == 0:
            return 0, 0
        
        # Now that we know the number of turns, calculate the mean turn angle
        # We need to for loop through all the turn waypoints we found
        angles = []
        for i_turn in intent_turn_idx:
            if not (i_turn < len(acrte.wplon)-1):
                continue
            # Get the needed stuff
            lat_cur, lon_cur   = acrte.wplat[i_turn], acrte.wplon[i_turn]
            lat_prev, lon_prev = acrte.wplat[i_turn-1], acrte.wplon[i_turn-1]
            lat_next, lon_next = acrte.wplat[i_turn+1], acrte.wplon[i_turn+1]
            
            # Get the angle
            d1=geo.kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=geo.kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                angles.append(angle)
            else:
                # Not a turn
                continue
            
        avg_angle = np.average(angles) if len(angles) > 0 else 0
        return num_turns, avg_angle
        
           
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
    
    def sb_detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' State-based detection.'''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------

        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.trk)
        intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        dvs = ownship.vs.reshape(1, ownship.ntraf) - \
            intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        # hPZ can differ per aircraft, get the largest value per aircraft pair
        hpz = np.asarray(np.maximum(np.asmatrix(hpz), np.asmatrix(hpz).transpose()))
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) *
                           np.asarray(tinconf < np.asmatrix(dtlookahead).T) * (1.0 - I), dtype=bool)

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return confpairs, lospairs, inconf, tcpamax, \
            qdr[swconfl], dist[swconfl], np.sqrt(dcpa2[swconfl]), \
                tcpa[swconfl], tinconf[swconfl], qdr, dist
        
    
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

# Plot set
# print(f'{bs.traf.id[idx1]} and {bs.traf.id[idx2]}')
# print(self.ac_leg_positions[idx1], self.ac_leg_positions[idx2])
# plt.figure('intersection')
# plt.plot(intent1_front.coords.xy[1], intent1_front.coords.xy[0], color = 'red')
# plt.plot(intent2_front.coords.xy[1], intent2_front.coords.xy[0], color = 'blue')
# plt.plot(intent1_back.coords.xy[1], intent1_back.coords.xy[0], color = 'red')
# plt.plot(intent2_back.coords.xy[1], intent2_back.coords.xy[0], color = 'blue')
# plt.scatter(int_point.y, int_point.x, color = 'green')
# plt.scatter(self.ac_leg_positions[idx1].y,self.ac_leg_positions[idx1].x, marker = 'x', color = 'red')
# plt.scatter(self.ac_leg_positions[idx2].y,self.ac_leg_positions[idx2].x, marker = 'x', color = 'blue')
# rpz_circle_1 = self.ac_leg_positions[idx1].buffer(16).exterior
# rpz_circle_2 = self.ac_leg_positions[idx2].buffer(16).exterior
# plt.plot(rpz_circle_1.xy[1], rpz_circle_1.xy[0], color = 'red')
# plt.plot(rpz_circle_2.xy[1], rpz_circle_2.xy[0], color = 'blue')
# ax = plt.gca()
# ax.set_aspect('equal', adjustable = 'box')
# plt.show(block = True)