import bluesky as bs
import numpy as np
import math
import pyproj
import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt
from bluesky.tools.geo import qdrpos

from shapely.geometry import LineString, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely.ops import split, nearest_points, linemerge, snap
from shapely.affinity import rotate
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection
from bluesky.tools import geo, datalog

"""
IntentCDV2 is bascially DefensiveCD but we assume we know the path of the intruder
"""

def init_plugin():
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'INTENTCD',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

confheader = \
    '#######################################################\n' + \
    'CONF LOG\n' + \
    'Conflict Statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Unique CONF ID [-]' + \
    'ACID1 [-],' + \
    'ACID2 [-],' + \
    'LAT1 [deg],' + \
    'LON1 [deg],' + \
    'ALT1 [ft],' + \
    'LAT2 [deg],' + \
    'LON2 [deg],' + \
    'ALT2 [ft]\n'

uniqueconflosheader = \
    '#######################################################\n' + \
    'Unique CONF LOS LOG\n' + \
    'Shows whether unique conflicts results in a LOS\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Unique CONF ID, ' + \
    'Resulted in LOS\n'

class IntentCD(ConflictDetection):
    def __init__(self):
        super().__init__()
        # Lookahead parameters
        self.lookahead_min = 100 #metres
        self.lookahead_max = 300 #metres
        
        # New detection parameters
        self.intent_geom = [] # Linestring of aircraft intent per pair
        self.dist_to_int = [] # Distance to intent intersections per pair
        self.int_coords = [] # Coordinates of the intersection point
        self.stopping_points = [] # Points at which the aircraft should stop before the intersection per pair
        self.dist_to_stop = []
        self.vel_rel_int = [] # Velocity relative to intent intersection per pair
        self.num_turns = [] # Number of turns per pair
        self.mean_turn_angle = [] # Mean turn angle per pair
        self.qdr_mat = np.array([]) # QDR for all aircraft
        self.dist_mat = np.array([]) # Distance for all aircraft
        self.los_detected = [] # If a LOS was detected or not
        
        self.rough_geometries = []
        self.ac_leg_positions = []
        self.dlookaheads = []
    
        # Distance buffer for shapely
        self.precision = 0.0001 # metres
        
        # Logging
        self.conflictlog = datalog.crelog('CDR_CONFLICTLOG', None, confheader)
        self.uniqueconfloslog = datalog.crelog('CDR_WASLOSLOG', None, uniqueconflosheader)
        
        # Conflict related
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.unique_conf_dict = dict()
        self.counter2id = dict() # Keep track of the other way around
        self.unique_conf_id_counter = 0 # Start from 0, go up
        self.confhold =  [] # array to keep track of the conflicts we are
        self.already_logged = [] #unique conflict IDs that have already been logged
        # keeping for an extra few seconds
        self.hold_time = 10 #seconds
        
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
        
    def reset(self):
        super().reset()
        # New detection parameters
        self.intent_geom = [] # Linestring of aircraft intent per pair
        self.dist_to_int = [] # Distance to intent intersections per pair
        self.stopping_points = [] # Points at which the aircraft should stop before the intersection per pair
        self.int_coords = [] # Coordinates of the intersection point
        self.dist_to_stop = []
        self.vel_rel_int = [] # Velocity relative to intent intersection per pair
        self.num_turns = [] # Number of turns per pair
        self.mean_turn_angle = [] # Mean turn angle per pair
        self.qdr_mat = [] # QDR for all aircraft
        self.dist_mat = [] # Distance for all aircraft
        self.los_detected = []
        
        self.rough_geometries = []
        self.ac_leg_positions = []
        self.dlookaheads = []
        
        # Conflict related
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.unique_conf_dict = dict()
        self.counter2id = dict() # Keep track of the other way around
        self.unique_conf_id_counter = 0 # Start from 0, go up
        self.confhold =  [] # array to keep track of the conflicts we are
        
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
        return
    
    def clearconfdb(self):
        return super().clearconfdb()
    
    def update(self, ownship, intruder):
        # Detect intersections
        self.confpairs, self.lospairs, self.inconf, self.dist_to_int, self.vel_rel_int, self.num_turns, \
            self.mean_turn_angle, self.qdr_mat, self.dist_mat, self.intent_geom , self.stopping_points, \
                self.dist_to_stop = self.detect(ownship, intruder)
                
        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique   
        
        # Update the logging
        self.update_log()
        return
    
    def detect(self, ownship, intruder):
        # Collect some useful vars
        G = bs.traf.TrafficSpawner.graph
        edges = bs.traf.TrafficSpawner.edges
        nodes = bs.traf.TrafficSpawner.nodes
        # Do state-based detection for good measure
        confpairs_s, lospairs, inconf_s, tcpamax_s, qdr_s, \
            dist_s, dcpa_s, tcpa_s, tLOS_s, qdr_mat, dist_mat = \
                self.sb_detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)
                
        # if lospairs:
        #     print(lospairs)
        
        # Get the current edges
        current_edges = self.get_current_edges()
        # Get the aircraft that share nodes within their routes
        acidx_int_pairs, problem_nodes = self.problem_aircraft_and_nodes(current_edges, dist_mat)
                
        self.los_detected = lospairs
        # if self.los_detected:
        #     print(self.los_detected)
            
        inconf = np.array([False]*ownship.ntraf)
        
        if len(acidx_int_pairs) == 0:
            return [], [], inconf, [], [], [], [], qdr_mat, dist_mat, [], [], []
        
        # For each confpair, we need to get the distance of each aircraft to the intersection,
        # the number of turns until the intersection, and the mean turn angle until the intersection
        # First, initialize the arrays
        conf_pairs = []
        dist_to_int = []
        velocity_wrt_int = []
        int_coords = []
        num_turns = []
        mean_turn_angle = []
        intent_geom = []
        stopping_points = []
        dist_to_stop = []
        
        # Iterate over the aircraft that have common nodes
        for i, pair in enumerate(acidx_int_pairs):
            # IDX1 is the ownship and IDX2 is the intruder
            idx1, idx2 = pair
                    
            # Get the problem nodes as well
            pair_nodes = problem_nodes[i]
            # Get the lookaheads
            dlookahead1 = bs.traf.gs[idx1] * self.dtlookahead_def
            dlookahead1 = max(min(dlookahead1, self.lookahead_max), self.lookahead_min)
            dlookahead2 = bs.traf.gs[idx2] * self.dtlookahead_def
            dlookahead2 = max(min(dlookahead2, self.lookahead_max), self.lookahead_min)
            # First of all, we can check if this pair is currently on the same path, and thus one is behind the other.
            # We can check this using the current edges of each aircraft.
            current_edge_1 = current_edges[idx1]
            route_edges_1 = bs.traf.TrafficSpawner.route_edges[idx1]
            current_edge_2 = current_edges[idx2]
            
            # Check if the current edge of ac2 is within the route of ac1
            if current_edge_2 in route_edges_1:
                # We can see who's in front and who's in the back by comparing the distances along the route
                # of the ownship.
                # We need the route of the ownship for this one in geometric form, so let's get it
                acrte_1 = bs.traf.ap.route[idx1]
                coords_1_utm_lon, coords_1_utm_lat = self.transform_coords.transform(acrte_1.wplon, acrte_1.wplat)
                route_geometry = LineString([*zip(coords_1_utm_lon, coords_1_utm_lat)])
                # Get the closest locations of these aircraft on the line
                ac1_lon_utm, ac1_lat_utm = self.transform_coords.transform(bs.traf.lon[idx1], bs.traf.lat[idx1])
                ac2_lon_utm, ac2_lat_utm = self.transform_coords.transform(bs.traf.lon[idx2], bs.traf.lat[idx2])
                p1, _ = nearest_points(route_geometry, Point(ac1_lon_utm, ac1_lat_utm))
                p2, _ = nearest_points(route_geometry, Point(ac2_lon_utm, ac2_lat_utm))
                
                distance1 = route_geometry.project(p1)
                distance2 = route_geometry.project(p2)
                
                if distance1 < distance2:
                    # Ownship is in the back, so distance to intersection of intruder is , and so is the velocity
                    dist2 = 0
                    vel2 = 0
                    # Intersection coords are simply the coords of ac2
                    int_latlon = [bs.traf.lat[idx2], bs.traf.lon[idx2]]
                    # For ownship, the velocity is just its own velocity, and the distance is just distance 2 - distance 1
                    dist1 = distance2 - distance1
                    vel1 = bs.traf.gs[idx1]
                else:
                    # Ownship is in the front
                    dist1 = 0
                    vel1 = 0
                    # Intersection coords are simply the coords of ac1
                    int_latlon = [bs.traf.lat[idx1], bs.traf.lon[idx1]]
                    dist2 = distance1 - distance2
                    vel2 = bs.traf.gs[idx2]
                    
                # Skip if any of the distances are greater than the lookahead distance
                if dist1 > dlookahead1 or dist2 > dlookahead2:
                    # not a conflict yet
                    continue
                    
                conf_pairs.append((bs.traf.id[idx1], bs.traf.id[idx2]))
                dist_to_int.append([dist1, dist2])
                velocity_wrt_int.append([vel1, vel2])
                 # In back-to-front conflicts these don't matter
                num_turns.append([0,0])
                mean_turn_angle.append([0,0])
                int_coords.append(int_latlon)
                intent_geom.append([None, None])
                stopping_points.append([None, None])
                dist_to_stop.append([0,0])
                inconf[idx1] = True
                inconf[idx2] = True
                continue
            else:
                # This means that they are not back to back, so we need to take each intersection point
                # and process it.
                # If this is the case, there might be more than one node and intersection, and thus dist_to_int, num_turns and mean_turn_angle
                # will have several entries for each aircraft pair.
                num_turns_list = []
                dist_to_int_list = []
                int_coords_list = []
                mean_turn_angle_list = []
                int_geom_list = []
                stopping_points_list = []
                dist_to_stop_list = []
                
                # We need to loop through the problem nodes
                for node in pair_nodes:
                    # We need to get the path to that node
                    if current_edge_1[0] == node or current_edge_2[0] == node:
                        # We're past the node, skip
                        continue
                    # First, get the next node, such that we have a complete route
                    next_node_own = current_edge_1[1]
                    next_node_int = current_edge_2[1]
                    # Compute the paths to that node for both aircraft
                    route_path_1 = nx.shortest_path(G, next_node_own, node)
                    route_path_2 = nx.shortest_path(G, next_node_int, node)
                    # Add the previous node of the aircraft to the route path
                    route_path_1.insert(0, current_edge_1[0])
                    route_path_2.insert(0, current_edge_2[0])
                    # Get the geometry
                    geom_path_1 = [edges.loc[(u, v, 0), 'geometry'] for u, v in zip(route_path_1[:-1], route_path_1[1:])]
                    geom_path_2 = [edges.loc[(u, v, 0), 'geometry'] for u, v in zip(route_path_2[:-1], route_path_2[1:])]
                    # Need to cut the geometry with the current position of the aircraft
                    own_cur_pos = Point(bs.traf.lon[idx1], bs.traf.lat[idx1])
                    int_cur_pos = Point(bs.traf.lon[idx2], bs.traf.lat[idx2])
                    # Only need the front line
                    # Merge the lines
                    merge1 = linemerge(geom_path_1)
                    merge2 = linemerge(geom_path_2)
                    
                    if isinstance(merge1, MultiLineString):
                        # Do the merge manually then
                        line_coords = []
                        for line in geom_path_1:
                            for coord in list(line.coords):
                                line_coords.append(coord)
                        merge1 = LineString(line_coords)
                        
                    if isinstance(merge2, MultiLineString):
                        # Do the merge manually then
                        line_coords = []
                        for line in geom_path_2:
                            for coord in list(line.coords):
                                line_coords.append(coord)
                        merge2 = LineString(line_coords)
                
                    _, intent_1 = self.cut_line_with_point(merge1, own_cur_pos)
                    _, intent_2 = self.cut_line_with_point(merge2, int_cur_pos)
                    # We intersect these two, we should get a point. If we get anything else than a point, then we convert it to a point.
                    # Basically, this should confirm that the node we are looking at is indeed an intersection. It could also be the case
                    # that the two aircraft have a certain portion of the path in common towards that node. In that case, we get a line
                    # intersection. We just take the first point of the line, and then check if we already added it to the intersection
                    # points. 
                    intersection = intent_1.intersection(intent_2)
                    
                    if intersection.is_empty:
                        # Skip
                        continue
                    
                    if isinstance(intersection, MultiLineString):
                        # Merge the line
                        intersection = linemerge(intersection)
                        # Take the first point
                        point_intersection = Point(intersection.coords.xy[0][0],intersection.coords.xy[1][0])
                    
                    elif isinstance(intersection, Point):
                        #Good then
                        point_intersection = intersection

                    elif isinstance(intersection, LineString):
                        # Take first point
                        point_intersection = Point(intersection.coords.xy[0][0],intersection.coords.xy[1][0])
                        
                    elif isinstance(intersection, MultiPoint):
                        #First point I guess
                        point_intersection = intersection.geoms[0]  
                    
                    else:
                        #uhh, dunno
                        #print('huh2', intersection)
                        point_intersection = None
                        continue
                    
                    # Check if the point is already in int_geom_list
                    if point_intersection in int_geom_list:
                        # We skip this one, it's already in the list
                        continue
                    
                    # Get the int_latlon
                    int_latlon = [point_intersection.y, point_intersection.x]

                    # Convert the intents and coordinates to UTM
                    coords_1_utm_lon, coords_1_utm_lat = self.transform_coords.transform(intent_1.coords.xy[0],intent_1.coords.xy[1])
                    coords_2_utm_lon, coords_2_utm_lat = self.transform_coords.transform(intent_2.coords.xy[0],intent_2.coords.xy[1])
                    intent_1_utm = LineString([*zip(coords_1_utm_lon, coords_1_utm_lat)])
                    intent_2_utm = LineString([*zip(coords_2_utm_lon, coords_2_utm_lat)])
                    point_intersection_utm = Point(self.transform_coords.transform(point_intersection.x, point_intersection.y))
                    
                    # Get the distance from the current aircraft position to the intersection
                    dist1 = intent_1_utm.project(point_intersection_utm)
                    dist2 = intent_2_utm.project(point_intersection_utm)
                    
                    # Skip if any of the distances are greater than the lookahead distance
                    if dist1 > dlookahead1 or dist2 > dlookahead2:
                        # not a conflict yet
                        continue
                    
                    # Let's create the stopping points. First, we cut the intents with the intersection point.
                    intent_1_utm_cut, _ = self.cut_line_with_point(intent_1_utm, point_intersection_utm)
                    intent_2_utm_cut, _ = self.cut_line_with_point(intent_2_utm, point_intersection_utm)
                    
                    # Then, create a buffer for each in function of the others' intent
                    buffer_rpz_1 = intent_1_utm.buffer(self.rpz_def * 1.1) #10% buffer to the buffer
                    buffer_rpz_2 = intent_2_utm.buffer(self.rpz_def * 1.1)
                    
                    # Now, the stopping points are the intersections with the intents
                    stopping_point_1 = intent_1_utm_cut.intersection(buffer_rpz_2.exterior)
                    stopping_point_2 = intent_2_utm_cut.intersection(buffer_rpz_1.exterior)
                    
                    # These might be multipoints. We want to take the point furthest away from the intersection.
                    if isinstance(stopping_point_1, MultiPoint):
                        distances_to_points = [intent_1_utm_cut.project(p) for p in stopping_point_1.geoms]
                        stopping_point_1 = stopping_point_1.geoms[distances_to_points.index(min(distances_to_points))]
                        
                    if isinstance(stopping_point_2, MultiPoint):
                        distances_to_points = [intent_2_utm_cut.project(p) for p in stopping_point_2.geoms]
                        stopping_point_2 = stopping_point_2.geoms[distances_to_points.index(min(distances_to_points))]
                    
                    num_turns1, avg_turn1 = self.get_ac_turn_info(idx1, intent_1)
                    num_turns2, avg_turn2 = self.get_ac_turn_info(idx2, intent_2)
                    
                    # Append things
                    num_turns_list.append([num_turns1, num_turns2])
                    dist_to_int_list.append([dist1, dist2])
                    mean_turn_angle_list.append([avg_turn1, avg_turn2])
                    int_geom_list.append(point_intersection)
                    int_coords_list.append(int_latlon)
                    stopping_points_list.append([stopping_point_1, stopping_point_2])
                    # Get the distance to stopping points
                    if stopping_point_1.is_empty:
                        dist_to_stop_1 = 0
                    else:
                        dist_to_stop_1 = intent_1_utm_cut.project(stopping_point_1)
                        
                    if stopping_point_2.is_empty:
                        dist_to_stop_2 = 0
                    else:
                        dist_to_stop_2 = intent_2_utm_cut.project(stopping_point_2)
                    dist_to_stop_list.append([dist_to_stop_1, dist_to_stop_2])
            
                if len(dist_to_int_list)>0 and len(int_geom_list)>0:    
                    # Append the values to the big lists
                    conf_pairs.append((bs.traf.id[idx1], bs.traf.id[idx2]))
                    dist_to_int.append(dist_to_int_list)
                    int_coords.append(int_coords_list[0]) # only append the first value for this one
                    velocity_wrt_int.append([bs.traf.gs[idx1],bs.traf.gs[idx2]])
                    # In back-to-front conflicts these don't matter
                    num_turns.append(num_turns_list)
                    mean_turn_angle.append(mean_turn_angle_list)
                    intent_geom.append(int_geom_list)
                    stopping_points.append(stopping_points_list)
                    dist_to_stop.append(dist_to_stop_list)
                    inconf[idx1] = True
                    inconf[idx2] = True
                
        # Also check the pairs that are in statebased pairs but not in the intersection pairs
        for j, pair in enumerate(confpairs_s):
            # First, skip the pair if it's already in confpairs
            if pair in conf_pairs:
                # They are going to solve it defensively
                continue
            
            # Append state-based stuff
            inconf[idx1] = True
            inconf[idx2] = True
            dist_to_int.append([tcpa_s[j] * bs.traf.gs[idx1], tcpa_s[j] * bs.traf.gs[idx2]])
            velocity_wrt_int.append([bs.traf.gs[idx1], bs.traf.gs[idx2]])
            int_coords.append(qdrpos(bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.hdg[idx1], dcpa_s[confpairs_s.index(pair)]/nm))
            num_turns.append([0,0])
            mean_turn_angle.append([0,0])
            conf_pairs.append((pair[0], pair[1]))
            intent_geom.append(['statebased'])
            stopping_points.append([None, None])
            dist_to_stop.append([0,0])

        return conf_pairs, lospairs, inconf, dist_to_int, velocity_wrt_int, num_turns, mean_turn_angle, qdr_mat, dist_mat, intent_geom, stopping_points, dist_to_stop
    
    def get_current_edges(self):
        # Get all the edges for all aircraft
        edges_trafarray = bs.traf.TrafficSpawner.route_edges
        #Initialise the edges
        current_edge = [None] * bs.traf.ntraf
        # We basically need to loop through all aircraft routes
        for acidx, acrte in enumerate(bs.traf.ap.route):
            # Skip aircraft if it doesn't have a route
            if len(acrte.wplat) == 0:
                continue
            
            current_edge[acidx] = edges_trafarray[acidx][acrte.iactwp]
        return current_edge
    
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
        
        
        # If the index of this waypoint is smaller or equal to the active waypoint index,
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
    
    def problem_aircraft_and_nodes(self, current_edge, dist_mat):
        '''The point of this function should be to output data such that it can be
        processed in an intent-based manner, but with several intents as possibilities.
        '''
        # Get all the aircraft pairs that are within 300m of each other
        pairs = np.array(np.where(dist_mat<self.lookahead_max)).T
        acidx_int_pairs = []
        problem_nodes = []
        for pair in pairs:
            acidx1 = pair[0]
            acidx2 = pair[1]
            # Get the lookahead for this aircraft pair
            dlookahead1 = bs.traf.gs[acidx1] * self.dtlookahead_def
            dlookahead1 = max(min(dlookahead1, self.lookahead_max), self.lookahead_min)
            dlookahead2 = bs.traf.gs[acidx2] * self.dtlookahead_def
            dlookahead2 = max(min(dlookahead2, self.lookahead_max), self.lookahead_min)
            # Get the current node of the ownship and intruder
            ac_edge1 = current_edge[acidx1]
            ac_edge2 = current_edge[acidx2]
            ac_node2 = ac_edge2[1]
            # Get the future nodes of the ownship
            current_wpt_id_1 = bs.traf.ap.route[acidx1].iactwp
            # Get the nodes in its route within the lookahead distance
            total_length = 0
            i = current_wpt_id_1
            prev_u, prev_v = current_edge[acidx1]
            nodes_to_check_1 = [prev_v] # Add the first V by default.
            while total_length < dlookahead1 and i<len(bs.traf.TrafficSpawner.route_edges[acidx1]):
                u,v = bs.traf.TrafficSpawner.route_edges[acidx1][i]
                if u==prev_u and v==prev_v:
                    # Next waypoint belongs to the same edge, skip
                    i += 1
                    continue
                total_length += bs.traf.TrafficSpawner.edges.loc[(u,v,0), 'length']
                nodes_to_check_1.append(v)
                prev_u = u
                prev_v = v
                i += 1
                
            # Now get the nodes for intruder
            #_, nodes_can_be_reached2 = self.get_graph_points_within_distance(ac_node2, dlookahead2)
            #-----------------------------------------------------------------
            # INTENT MODIFICATION, get nodes from the intruder route
            # Get the future nodes of the intruder based on actual route (intent)
            current_wpt_id_2 = bs.traf.ap.route[acidx2].iactwp
            # Get the nodes in its route within the lookahead distance
            total_length = 0
            i = current_wpt_id_2
            prev_u, prev_v = current_edge[acidx2]
            nodes_can_be_reached2 = [prev_v] # Add the first V by default.
            while total_length < dlookahead2 and i<len(bs.traf.TrafficSpawner.route_edges[acidx2]):
                u,v = bs.traf.TrafficSpawner.route_edges[acidx2][i]
                if u==prev_u and v==prev_v:
                    # Next waypoint belongs to the same edge, skip
                    i += 1
                    continue
                total_length += bs.traf.TrafficSpawner.edges.loc[(u,v,0), 'length']
                nodes_can_be_reached2.append(v)
                prev_u = u
                prev_v = v
                i += 1
            #-----------------------------------------------------------------
                
            # Now get the possible conflict nodes, basically the nodes that are common between these two
            conflict_nodes = set([node for node in nodes_to_check_1 if node in nodes_can_be_reached2])
            if conflict_nodes:
                acidx_int_pairs.append(pair)
                problem_nodes.append(list(conflict_nodes))             
        return acidx_int_pairs, problem_nodes
    
    def get_graph_points_within_distance(self, source_id, lookahead_distance):
        '''Returns the nodes of the edges that are within the given distance.
        Note that nodes will be further than the distance, as the edge needs to be
        included.
        '''
        # Get needed things
        G = bs.traf.TrafficSpawner.graph
        nodes = bs.traf.TrafficSpawner.nodes
        
        if source_id not in G.nodes:
            raise nx.NetworkXError(f"The node {source_id} is not in the graph.")
        # List of nodes
        nodes_within_distance = set()
        # List of geometries
        points = []
        # Initial depth of search
        depth = 1
        while True:
            # Assume all nodes will be added
            all_nodes_added = True
            # Do the search for descendants
            descendants = nx.descendants_at_distance(G, source_id, depth)
            # # From these, we want to eliminate the children of nodes we already visited
            for node in nodes_within_distance:
                for child in nx.descendants_at_distance(G, node, 1):
                    if child in descendants:
                        descendants.remove(child)
                        
            # Get the distance for each node
            for node in descendants:
                # Get the path distance to the node
                path_distance = nx.shortest_path_length(G, source_id, node, weight='length')
                if path_distance > lookahead_distance:
                    # Add this node to the set of nodes within distance
                    nodes_within_distance.add(node)
                    # Also add the nodes to reach this route
                    route_nodes = nx.shortest_path(G, source_id, node)
                    for route_node in route_nodes:
                        nodes_within_distance.add(route_node)
                else:
                    # All nodes were not added this iteration
                    all_nodes_added = False
                    
            if all_nodes_added:
                break
            else:
                depth += 1
        points = [nodes.loc[x, 'geometry'] for x in nodes_within_distance]
        return MultiPoint(points), nodes_within_distance
    
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
            
    def update_log(self):
        '''Here, we are logging the information for current conflicts as well as
        whether these conflicts resulted in a LOS or not.'''
        confpairs_new = list(set(self.confpairs) - self.prevconfpairs) # New confpairs
        confpairs_out = list(self.prevconfpairs - set(self.confpairs)) # Pairs that are no longer in conflict
        lospairs_new = list(set(self.lospairs) - self.prevlospairs) # New lospairs
        
        # First of all, add the new conflicts to the unique dict tracker
        for confpair in confpairs_new:
            # The dict is of the following format:
            # lower_number_acidx_newer_number_acidx : [unique_id, was it a LOS or not]
            # First, get the aircraft IDX
            idx1 = bs.traf.id.index(confpair[0])
            idx2 = bs.traf.id.index(confpair[1])
            # Create dictionary entry
            if idx1 < idx2:
                dictkey = confpair[0] + confpair[1]
            else:
                dictkey = confpair[1] + confpair[0]
                
            if dictkey in self.unique_conf_dict:
                # Pair already in there
                continue
            else:
                self.unique_conf_dict[dictkey] = [self.unique_conf_id_counter, False]
                self.counter2id[self.unique_conf_id_counter] = dictkey 
                self.unique_conf_id_counter += 1
                
                self.conflictlog.log(
                self.unique_conf_dict[dictkey][0],
                confpair[0],
                confpair[1],
                bs.traf.lat[idx1],
                bs.traf.lon[idx1],
                bs.traf.alt[idx1],
                bs.traf.lat[idx2],
                bs.traf.lon[idx2],
                bs.traf.alt[idx2]
            )   
            
        # Now check the new LOS
        done_pairs = []
        for lospair in lospairs_new:
            # Set the los flag of these in the unique dict tracker
            idx1 = bs.traf.id.index(lospair[0])
            idx2 = bs.traf.id.index(lospair[1])
            if idx1 < idx2:
                dictkey = lospair[0] + lospair[1]
            else:
                dictkey = lospair[1] + lospair[0]
                
            if dictkey in done_pairs:
                # Already done, continue
                continue
            
            done_pairs.append(dictkey)
                
            # Set the bool as true
            if dictkey in self.unique_conf_dict:
                self.unique_conf_dict[dictkey][1] = True
            else:
                # This LOS was not detected, but it is usually because of weird geometry
                #print(dictkey)
                continue
                
            
        # Now handle aircraft that are no longer in confpairs
        done_pairs = []
        for confpair in confpairs_out:
            # There is a possibility that one aircraft thinks it is still in a conflict while the other
            # doesn't. If this confpair is still in confpairs but inverted, skip it
            if (confpair[1], confpair[0]) in self.confpairs:
                continue
            # Log these in the uniqueconfloslog
            if confpair[0] not in bs.traf.id or confpair[1] not in bs.traf.id:
                # One of these aircraft was deleted, so just log them and done.
                if confpair[0] + confpair[1] in self.unique_conf_dict:
                    dictkey = confpair[0] + confpair[1]
                elif confpair[1] + confpair[0] in self.unique_conf_dict:
                    dictkey = confpair[1] + confpair[0]
                else:
                    # Absolutely no clue, continue I guess
                    #print('huh')
                    continue
                
                self.uniqueconfloslog.log(
                self.unique_conf_dict[dictkey][0],
                str(self.unique_conf_dict[dictkey][1])
                )
                self.unique_conf_dict.pop(dictkey)
                continue
                    
                
            idx1 = bs.traf.id.index(confpair[0])
            idx2 = bs.traf.id.index(confpair[1])
            
            if idx1 < idx2:
                dictkey = confpair[0] + confpair[1]
            else:
                dictkey = confpair[1] + confpair[0]
                
            if dictkey in done_pairs:
                # Already done, continue
                continue
            
            done_pairs.append(dictkey)
            
            # We want to keep this entry for a few extra seconds and see what happens
            # Get the conflict info and log it, then delete the entry
            self.uniqueconfloslog.log(
                self.unique_conf_dict[dictkey][0],
                str(self.unique_conf_dict[dictkey][1])
            )
            self.unique_conf_dict.pop(dictkey)
        
        self.prevconfpairs = set(self.confpairs)
        self.prevlospairs = set(self.lospairs)
        
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