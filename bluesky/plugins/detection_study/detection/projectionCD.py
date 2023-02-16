import bluesky as bs
import numpy as np
import shapely as sp
import math
import pyproj
import geopandas as gpd
import matplotlib.pyplot as plt

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
        
        # Distance buffer for shapely
        self.precision = 0.01 # metres
        
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
        self.intent_geometries = []
        
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
        tree_query = self.geom_tree.query(self.intent_geometries, predicate = 'intersects')
        all_intersections = np.transpose(tree_query)
        
        # Problem is, the query contains self intersections
        # We want the indices where column 0 is not equal to column 1
        acidx_int_pairs = all_intersections[all_intersections[:, 0] != all_intersections[:, 1]]
        
        # Get lospairs and inconf bools
        lospairs, qdr_mat, dist_mat = self.los_detect(ownship, intruder)
        inconf = np.array([False]*ownship.ntraf)
        
        if len(acidx_int_pairs) == 0:
            return [], [], inconf, [], [], [], qdr_mat, dist_mat, []
        
        inconf[acidx_int_pairs[0,:]] = True
        
        # For each confpair, we need to get the distance of each aircraft to the intersection,
        # the number of turns until the intersection, and the mean turn angle until the intersection
        # First, initialize the arrays

        dist_to_int = [-999,-999] * len(acidx_int_pairs)
        num_turns = [-999,-999] * len(acidx_int_pairs)
        mean_turn_angle = [-999,-999] * len(acidx_int_pairs)
        intent_geom = [-999,-999] * len(acidx_int_pairs)
        
        for j, pair in enumerate(acidx_int_pairs):
            idx1, idx2 = pair[0], pair[1]
            # Get info from the functions
            dist1, dist2, vel1, vel2, split1, split2, int_point = self.intersection_info(idx1, idx2)
            num_turns1, num_turns2, mean_turn_angle1, mean_turn_angle2 = self.turn_info(idx1, idx2, split1, split2)
            
            # Assign the values
            dist_to_int[j] = [dist1, dist2]
            num_turns[j] = [num_turns1, num_turns2]
            mean_turn_angle[j] = [mean_turn_angle1, mean_turn_angle2]
            intent_geom[j] = [split1, split2]
            
            if isinstance(int_point, MultiPoint):
                print('----------------------------------------------------------------')
                print(bs.traf.id[idx1], bs.traf.id[idx2])
                print(dist1, dist2)
                print(vel1, vel2)
                print(num_turns1, num_turns2)
                print(mean_turn_angle1, mean_turn_angle2)
                plt.plot(split1.coords.xy[1], split1.coords.xy[0], color = 'red')
                plt.plot(split2.coords.xy[1], split2.coords.xy[0], color = 'blue')
                for i, p in enumerate(int_point.geoms):
                    plt.scatter(p.y, p.x, color = 'green')
                plt.scatter(split1.coords.xy[1][1], split1.coords.xy[0][1], marker = 'x', color = 'red')
                plt.scatter(split2.coords.xy[1][1], split2.coords.xy[0][1], marker = 'x', color = 'blue')
                ax = plt.gca()
                ax.set_aspect('equal', adjustable = 'box')
                plt.show(block = True)
        
        # Change to strings
        acidx_int_pairs = [(bs.traf.id[pair[0]], bs.traf.id[pair[1]]) for pair in acidx_int_pairs]
        
        return acidx_int_pairs, lospairs, inconf, dist_to_int, num_turns, mean_turn_angle, qdr_mat, dist_mat, intent_geom
    
    def intersection_info(self, idx1, idx2):
        """Function that outputs information about the intersection between the intents of two
        aircraft, depending on the type of intersection.
        
        We have several types of possible intersections:
        1. Type 1: Classic intersection
            - Intersection is a point
            - Both aircraft are heading towards the intersection
            - Intersection point is ahead of both aircraft
        2. Type 2: Past intersection
            - Intersection is a point
            - One aircraft is already past the intersection point
            - The intersection point lies on the back leg of the intent of one aircraft
        3. Type 3: Merge intersection
            - Intersection is a linestring
            - Aircraft are coming from different streets and merging into the same one
            - The intersection itself does not contain any of the aircraft
        4. Type 4: Same-intent intersection
            - Intersection is a linestring
            - Aircraft have the same path, one is behind, one in front
            - The intersection itself is a linestring
            - The intersection contains one of the aircraft
        5. Type 5: Other things
            - Some weird cases where the intersection is a multipoint or a geometry collection
            - These need to be handled differently
            - If intersection is a MultiPoint, do checks for types 1 and 2
            - If intersection is a MultiLineString, do checks for types 3 and 4
            - If intersection is a Geometry Collection, unpack it, and then do checks for all types
        """
        
        # TODO: Take care of all geometries
        # TODO: Check if the current geometries are fine
        # Get the geometries of the aircraft
        intent1 = self.intent_geometries[idx1]
        intent2 = self.intent_geometries[idx2]
        
        # Get the intersection point between the two intents
        intersection = intent1.intersection(intent2)
        
        if isinstance(intersection, Point):
            # This can be either a type 1 or 2
            # Get the back lines of both intents
            int_point = intersection # Rename
            back_line1 = LineString([intent1.coords[0], intent1.coords[1]])
            back_line2 = LineString([intent2.coords[0], intent2.coords[1]])
            
            # Check if intersection point is within any of these two
            if back_line1.contains(intersection):
                # This is a type 2 intersection, and aircraft 1 is past the intersection point
                # Thus, we return negative velocity and distance value for aircraft 1
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                vel1 = -bs.traf.gs[idx1]
                vel2 = bs.traf.gs[idx2]
                dist1 = -intent1_split.length - self.rpz_def/2
                dist2 = intent2_split.length - self.rpz_def/2
                
            elif back_line2.contains(intersection):
                # This is a type 2 intersection, and aircraft 2 is past the intersection point
                # Thus, we return negative velocity and distance value for aircraft 2
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                vel1 = bs.traf.gs[idx1]
                vel2 = -bs.traf.gs[idx2]
                dist1 = intent1_split.length - self.rpz_def/2
                dist2 = -intent2_split.length - self.rpz_def/2
                
            else:
                # This is a type 1 intersection
                # Everyone has positive values                
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                vel1 = bs.traf.gs[idx1]
                vel2 = bs.traf.gs[idx2]
                dist1 = intent1_split.length - self.rpz_def/2
                dist2 = intent2_split.length - self.rpz_def/2

        if isinstance(intersection, LineString):
            # This can happen if the intersection is a type 3 or 4
            # If it's a type 4, then one of the intents contains the other aircraft
            # Get the aircraft positions, they should always be the second point in the intent
            ac_point1 = Point(intent1.coords[1])
            ac_point2 = Point(intent2.coords[1])
            if intent1.contains(ac_point2):
                # This means that this is a type 4 intersection, 
                # and aircraft 2 is the "moving intersection point"
                int_point = ac_point2
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                # With respect to the intersection point, aircraft 2 is not moving
                # The distance 
                vel1 = bs.traf.gs[idx1] - bs.traf.gs[idx2]
                vel2 = 0
                dist1 = intent1_split.length - self.rpz_def/2
                dist2 = 0
                
            elif intent2.contains(ac_point1):
                # The other way around, aircraft 1 is the "moving intersection point"
                int_point = ac_point1
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                # With respect to the intersection point, aircraft 2 is not moving
                # The distance 
                vel1 = 0
                vel2 = bs.traf.gs[idx2] - bs.traf.gs[idx1]
                dist1 = 0
                dist2 = intent2_split.length - self.rpz_def/2
                
            else:
                # Then this is a type 3 intersection, take the intersection point
                # as the first point in the linestring
                int_point = Point(intersection.coords[0])
                intent1_split = split(intent1, int_point).geoms[0]
                intent2_split = split(intent2, int_point).geoms[0]
                # With respect to the intersection point, aircraft 2 is not moving
                # The distance 
                vel1 = 0
                vel2 = bs.traf.gs[idx2]
                dist1 = 0
                dist2 = intent2_split.length - self.rpz_def/2
            
        if isinstance(intersection, MultiLineString):
            # I wanna see this case
            print('MULTILINESTRING')
            intersection = linemerge(intersection)
            # Return bogus values and the intents
            return 0, 0, 0, 0, intent1, intent2, intersection
        
        if isinstance(intersection, MultiPoint):
            print('MULTIPOINT')
            return 0, 0, 0, 0, intent1, intent2, intersection

        if isinstance(intersection, GeometryCollection):
            print('GEOMETRY COLLECTION')
            return 0, 0, 0, 0, intent1, intent2, intersection
    
        # Return information
        return dist1, dist2, vel1, vel2, intent1_split, intent2_split, int_point
    
    def turn_info(self, idx1, idx2, split1, split2):
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
        # Initialise the list of geometries
        self.intent_geometries = []
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
                # Change the precision of the linestring
                current_leg = sp.set_precision(current_leg, self.precision)
                # Get the position of the aircraft on the currrent leg
                ac_leg_pos, _ = nearest_points(current_leg, Point([ac_lon_utm, ac_lat_utm]))
                ac_leg_pos = snap(ac_leg_pos, current_leg, 1)
            else:
                # We're before the first waypoint
                ac_lon_utm, ac_lat_utm = self.transform_coords.transform(bs.traf.lon[acidx],bs.traf.lat[acidx])
                ac_leg_pos = Point([ac_lon_utm, ac_lat_utm])
                
            print(ac_leg_pos.within(current_leg))
                
            # Convert the route coordinates we need to UTM
            # Convert the coordinates of the route to UTM
            rte_utm_lon,rte_utm_lat = self.transform_coords.transform(acrte.wplon[act_wp:], acrte.wplat[act_wp:])
            
                        # -------------- FROM HERE ONWARDS WE DO LAT/LON -------------------
            # Create a simplified LineString with these coords
            rte_simplified = sp.set_precision(LineString(zip(rte_utm_lat, rte_utm_lon)), self.precision)
            # Get the coords back
            rte_simplified_lat = rte_simplified.coords.xy[0]
            rte_simplified_lon = rte_simplified.coords.xy[1]
            
            # Concatenate the coords
            rte_lat = np.concatenate(([ac_leg_pos.y], rte_simplified_lat))
            rte_lon = np.concatenate(([ac_leg_pos.x], rte_simplified_lon))
            
            # Add the aircraft point to the simplified LineString
            rte_linestring = LineString(zip(rte_lat, rte_lon))
            
            #print(rte_linestring)
            
            # Cut the linestring in function of the lookahead time
            cut_dist = bs.traf.gs[acidx] + self.dtlookahead[acidx]
            cut_dist = max(min(cut_dist, self.lookahead_max), self.lookahead_min)
            
            cut_rte = self.cut(rte_linestring, cut_dist)
            
            # In order to avoid aircraft intruding from behind or the front when route is too short,
            # we add a line to the back (and if needed to the front).
            # Get first line segment
            first_seg = LineString([cut_rte.coords[0], cut_rte.coords[1]])
            
            # Buffer the current position of the aircraft with 16m so we get a circle around it
            ac_rpz_circle = Point(cut_rte.coords[0]).buffer(self.rpz_def / 2)
            
            # The nearest point on this circle to the line should be a nice front line to use
            front_point, _ = nearest_points(ac_rpz_circle.exterior, first_seg)
            
            # Rotating this point around the aircraft gives us a good back line
            back_point = rotate(front_point, 180, origin = cut_rte.coords[0])
            
            # We want to add the front point to cut_rte only if cut_rte is too short
            if cut_rte.length < self.rpz_def / 2:
                # The cut_rte is now just composed of the back point, aircraft, and front point
                intent_geom = LineString([[back_point.x, back_point.y], 
                                      cut_rte.coords[0], 
                                      [front_point.x, front_point.y]])
                
            else:
                # We simply add the back point to cut_rte
                intent_geom = LineString([[back_point.x, back_point.y], *cut_rte.coords])
            
            # Add the route to the list of geometries
            self.intent_geometries.append(intent_geom)
            
        # Create the index
        return STRtree(self.intent_geometries)
    
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
    