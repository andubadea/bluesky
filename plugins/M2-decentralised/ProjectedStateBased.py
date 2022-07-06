''' State-based conflict detection. '''
import numpy as np
from shapely.geometry import LineString, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely.ops import nearest_points, split, transform, linemerge
import geopandas as gpd
# from rich import inspect
from shapely.affinity  import affine_transform, scale, translate
from pyproj import Transformer

from bluesky import stack
import bluesky as bs
from bluesky.tools import geo
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection
from time import time

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'PROJECTEDSTATEBASED',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config


class ProjectedBased(ConflictDetection):
    def __init__(self):
        super().__init__()
        self.dist_mat = np.array([])
        self.qdr_mat = np.array([])
        self.rpz_actual = 32 #m
        self.rpz_buffered = 40 #m
        self.hpz_actual = 7.62 #m
        self.dtlookahead_actual = 10 #s

        # create some transformers
        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:32633")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:32633", "EPSG:4326")
        return
        
    def clearconfdb(self):
        ''' Clear conflict database. '''
        self.confpairs_unique.clear()
        self.lospairs_unique.clear()
        self.confpairs.clear()
        self.lospairs.clear()
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.inconf = np.zeros(bs.traf.ntraf)
        self.tcpamax = np.zeros(bs.traf.ntraf)
        self.dist_mat = np.array([])
        self.qdr_mat = np.array([])
        self.projected_lats = np.array([])
        self.projected_lons = np.array([])
        self.conftrks = np.array([])
        return
        
    def update(self, ownship, intruder):
        ''' Perform an update step of the Conflict Detection implementation. '''
        self.confpairs, self.inconf, self.tcpamax, self.qdr, \
            self.dist, self.dcpa, self.tcpa, self.tLOS, self.projected_lats, self.projected_lons, \
                self.conftrks= self.detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)
                

        # Check LOS the normal way
        self.lospairs, self.qdr_mat, self.dist_mat = self.detect_los(ownship, intruder, self.rpz, self.hpz)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique    
        
    def detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' Conflict detection between ownship (traf) and intruder (traf/adsb).'''

        ############### START PROJECTION ########################

        # t1 = time()
        # here find the position along route of ownship
        routes = ownship.ap.route
        
        # intialize the geo_dict
        geo_dict = {'geometry': [], 'acid': [], 'current_point': []}
        
        actual_intersections = []

        if ownship.ntraf >= 1:

            confpairs, qdr_conf, dist_conf, dcpa_conf, tcpa_conf, tLOS_conf = [], [], [], [], [], []
            projected_lons, projected_lats = [], []
            conftrks = []
            # return empty things if there are no intersections
            inconfs = np.full(ownship.ntraf, False, dtype=np.bool)
            tcpamaxs = np.full(ownship.ntraf, 0)
            
            # TODO: assert geometries
            # TODO: create routes just once outside of loop
            # TODO: vectorize this
            # TODO: NUMPYFY THE RETURN VALUES
            # TODO: run once per pair instead of twice
            # TODO: fix front and back of route when it is less than 32 meters
            # solution just extend the route_line outside BlueSky
            for idx, route in enumerate(routes):
                
                if not route.wplat:
                    continue

                # get the current location
                current_loc = Point(self.transformer_to_utm.transform(ownship.lat[idx], ownship.lon[idx]))
                
                # get the lookahead distance
                look_ahead_dist = ownship.selspd[idx] * dtlookahead[idx]

                # add lon lat to shapely linestring
                route_line = gpd.GeoSeries(LineString(zip(route.wplon, route.wplat)), crs='epsg:4326')
                route_line = route_line.to_crs(epsg=32633)

                # extend the route_line 40 meters in front and behind
                # get last two points of front and back
                end_extension =  LineString(route_line.geometry.values[0].coords[-2:])
                end_sf = (end_extension.length + 400) / end_extension.length
                end_extension = scale(end_extension, xfact=end_sf, yfact=end_sf, origin=end_extension.coords[0])
                p_end = route_line.geometry.values[0].coords[-2]
                end_extension = LineString([p_end, end_extension.coords[-1]])

                start_extension  =  LineString(route_line.geometry.values[0].coords[:2])
                start_sf = (start_extension.length + 400) / start_extension.length
                start_extension = scale(start_extension, xfact=-start_sf, yfact=-start_sf, origin=start_extension.coords[0])
                start_extension = reverse_geom(start_extension)

                # now ensure that values are the same so merging becomes a linestring
                p_start = route_line.geometry.values[0].coords[1]
                start_extension = LineString([start_extension.coords[0], p_start])

                # merge lines
                route_merged = MultiLineString([start_extension.coords, route_line.geometry.values[0].coords[1:-1], end_extension.coords])
                route_merged = linemerge(route_merged)
                
                route_line = gpd.GeoSeries(route_merged, crs='epsg:32633')

                # find closest point to linestring
                p1, _ = nearest_points(route_line.geometry.values[0], current_loc)

                # now split the line to remove eveything before current position
                back_line, front_line = split_line_with_point(route_line.geometry.values[0], p1)

                # now interpolate along the line
                if front_line.length < rpz[0]:

                    if front_line.length == 0:
                        # get the last two points of route and extend 32 meters
                        # In reality, aircraft should be deleted at last waypoint so this
                        # is a safety so bluesky doesn't crash
                        dummy_line = LineString(route_line.geometry.values[0].coords[-2:])
                        sf = rpz[0] / dummy_line.length
                        look_ahead_line = scale(dummy_line, xfact=sf, yfact=sf, origin=route_line.geometry.values[0].coords[-1])

                    else:
                        sf = rpz[0] / front_line.length
                        look_ahead_line = scale(front_line, xfact=sf, yfact=sf, origin=p1)
                        try:
                            l1 = LineString([p1, look_ahead_line.coords[1]])
                            l2 = LineString(look_ahead_line.coords[1:])
                        except ValueError:
                            print('---------------------')
                            print(bs.stack.get_scenname())
                            print('---------------------')
                            raise ValueError('Error in projecting lookahead line')

                        look_ahead_line = MultiLineString([l1, l2])
                        look_ahead_line = linemerge(multi_line)

                else:
                    
                    # normal lookahead extension
                    look_ahead_dist = look_ahead_dist + rpz[0]
                    look_ahead_dist = 100 if look_ahead_dist < 100 else look_ahead_dist
                    end_point = front_line.interpolate(look_ahead_dist)

                    # now split line again to get line with a lookahead tine
                    # bs.stack.get_scenname()
                    look_ahead_line, _ = split_line_with_point(front_line, end_point)


                # now also extend the line with back_line 32 meters back
                back_line = reverse_geom(back_line)

                # if near the start of the line then just extend the line so it is 32 meters
                if back_line.length < rpz[0]:

                    if back_line.length == 0:
                        # get the first two points of route and extend 32 meters
                        # In reality, aircraft should be deleted at last waypoint so this
                        # is a safety so bluesky doesn't crash
                        dummy_line = LineString(route_line.geometry.values[0].coords[:2])
                        sf = rpz[0] / dummy_line.length
                        look_back_line = scale(dummy_line, xfact=-sf, yfact=-sf, origin=p1)
                        # ensure that rounding error is removed
                        look_back_line = LineString([p1, look_back_line.coords[1]])

                    
                    else:
                        sf = rpz[0] / back_line.length
                        look_back_line = scale(back_line, xfact=sf, yfact=sf, origin=back_line.coords[0])
                        look_back_line = LineString([back_line.coords[0], look_back_line.coords[-1]])
                    
                else:
                    # interpolate with route geometry if larger than 32 meters
                    start_point = back_line.interpolate(rpz[0])
                
                    # now split line again to get line that extends 32 meters back from aircraft
                    look_back_line, _ = split_line_with_point(back_line, start_point)

                # reverse the line again before merging with look_ahead_line
                look_back_line = reverse_geom(look_back_line)

                # merge lines
                multi_line = MultiLineString([look_back_line, look_ahead_line])
                merged_line = linemerge(multi_line)

                if not isinstance(merged_line, LineString):
                    raise ValueError('Merged line is not a LineString')
                    
                # fill the geo_dict
                geo_dict['geometry'].append(merged_line)
                geo_dict['acid'].append(ownship.id[idx])
                geo_dict['current_point'].append(p1)

            # t2 = time()
            # print('Time to extrapolate: ', t2-t1)

            # t1 = time()
            # create geopandas geoseries
            geo_series = gpd.GeoSeries(geo_dict['geometry'], crs='epsg:32633', index=geo_dict['acid'])

            # check if look_ahead_lines_intersect
            own_inter, int_inter = geo_series.sindex.query_bulk(geo_series, predicate="intersects")

            # note that all intersect with themselves so you must check if there are any unique intersections
            # This happens when there are more intersections than aircraft
            if len(own_inter) > ownship.ntraf:

                # Get all unique intersections since there are more intersections than aircraft
                # Also because query_bulk returns also self-intersections
                uniq_arr, counts = np.unique(own_inter, axis=0, return_counts=True)

                # select indices from own_inter that correspond to unique values with a count greater than 1
                potential_intersections = np.arange(len(own_inter))[~np.in1d(own_inter, uniq_arr[counts == 1])]

                # stack the ownship and intruder intersection vertically (nx2) array
                own_int_array = np.column_stack((own_inter[potential_intersections], int_inter[potential_intersections]))

                # check rows and if the columns are equal delete that row
                actual_intersections = own_int_array[own_int_array[:,0] != own_int_array[:,1]]

            # t2 = time()
            # print("Time to check intersection: ", t2-t1)

            # t3 = time()

            # if they intersect rebuild intruder.lat and intuder.lon, intruder.trk so that state based works normally
            for intersection in actual_intersections:

                curr_ownship = intersection[0]
                ownship_id = ownship.id[curr_ownship]

                curr_intruder = intersection[1]
                intruder_id = intruder.id[curr_intruder]

                # find intersection point between ownship and intruder using geo_series
                own_line = geo_series[ownship_id]
                int_line = geo_series[intruder_id]

                p_own = geo_dict['current_point'][curr_ownship]
                p_int = geo_dict['current_point'][curr_intruder]

                # get the intersection point
                p_inter = own_line.intersection(int_line)

                # check if p_inter is a linestring 
                if isinstance(p_inter, LineString):
                    # if so, get the first point
                    p_inter = Point(p_inter.coords[0])

                # check if p_inter is a multilinestring (this means multiple intersections)
                if isinstance(p_inter,MultiLineString):
                    p_inter = Point(p_inter[0].coords[0])
                
                # can also be a multipoint so select first intersection
                if isinstance(p_inter, MultiPoint):
                    p_inter = p_inter[0]

                if isinstance(p_inter, GeometryCollection):
                    # get first geometry of collection
                    p_inter = p_inter[0]

                    # if linestring choose a point
                    if isinstance(p_inter, LineString):
                        p_inter = Point(p_inter.coords[0])
                    
                    # if point just use it
                    if isinstance(p_inter, Point):
                        pass
                
                # do a final sanity check to make sure p_inter is a point
                if not isinstance(p_inter, Point):
                    raise ValueError('p_inter is not a point')

                if not isinstance(own_line, LineString):
                    raise ValueError('own_line is not a linestring')

                if not isinstance(int_line, LineString):
                    raise ValueError('int_line is not a linestring')

                # now split ownship and intruder lines with interseciton point (back and front)
                s_own_back, s_own_front = split_line_with_point(own_line, p_inter)
                s_int_back, s_int_front = split_line_with_point(int_line, p_inter)

                # check if the intersection is in front or back of ownship and intruder

                if s_own_back.contains(p_own) and s_int_back.contains(p_int):
                    
                    # Case 1: Interection is in front of ownship and intruder
                    # this means that the p_own is in s_own_back
                    # and intuder is in s_int_back
                    # print('CASE 1: THERE IS AN INTERSECTION')                
                    
                    # remove back part behind ownship and intruder
                    _, s_own = split_line_with_point(s_own_back, p_own)
                    _, s_int = split_line_with_point(s_int_back, p_int)

                    # first step is to project the ownship and intruder line
                    p1 = Point([s_own.xy[0][-2],  s_own.xy[1][-2]])
                    p2 = Point([s_int.xy[0][-2],  s_int.xy[1][-2]])

                    s_own_end = LineString([p1, p_inter])
                    s_int_end = LineString([p2, p_inter])
                    
                    own_scale_factor = s_own.length/s_own_end.length
                    int_scale_factor = s_int.length/s_int_end.length

                    lpr_own = scale(s_own_end, xfact=own_scale_factor, yfact=own_scale_factor, origin=p_inter)
                    lpr_int = scale(s_int_end, xfact=int_scale_factor, yfact=int_scale_factor, origin=p_inter)

                    pr_own = Point([lpr_own.xy[0][0], lpr_own.xy[1][0]])
                    pr_int = Point([lpr_int.xy[0][0], lpr_int.xy[1][0]])

                    # convert to lat lon from utm of intruder
                    intruderlat, intruderlon = self.transformer_to_latlon.transform(pr_int.x, pr_int.y)
                    ownshiplat, ownshiplon = self.transformer_to_latlon.transform(pr_own.x, pr_own.y)
                    inter_point_lat, inter_point_lon = self.transformer_to_latlon.transform(p_inter.x, p_inter.y)

                    # assign intruder.lat and intruder.lon with int_x and int_y
                    inttrk, *_ = geo.qdrdist(intruderlat, intruderlon, inter_point_lat, inter_point_lon)                   
                    ownntrk, *_ = geo.qdrdist(ownshiplat, ownshiplon, inter_point_lat, inter_point_lon)
                    
                elif s_own_front.contains(p_own) and s_int_back.contains(p_int):

                    # Case 2: Interection is behind ownship
                    # and in front of intruder
                    # this means that p_own is in s_own_front
                    # and p_int is in s_int_back
                    # print('CASE 2: THERE IS AN INTERSECTION')
                    
                    # keep back part behind ownship and front part from intruder
                    s_own, _ = split_line_with_point(s_own_front, p_own)
                    _, s_int = split_line_with_point(s_int_back, p_int)

                    # first step is to project the ownship and intruder line
                    p1 = Point([s_own.xy[0][1],  s_own.xy[1][1]])
                    p2 = Point([s_int.xy[0][-2],  s_int.xy[1][-2]])

                    s_own_end = LineString([p_inter, p1])
                    s_int_end = LineString([p2, p_inter])
                    
                    own_scale_factor = s_own.length/s_own_end.length
                    int_scale_factor = s_int.length/s_int_end.length

                    lpr_own = scale(s_own_end, xfact=own_scale_factor, yfact=own_scale_factor, origin=p_inter)
                    lpr_int = scale(s_int_end, xfact=int_scale_factor, yfact=int_scale_factor, origin=p_inter)

                    pr_own = Point([lpr_own.xy[0][-1], lpr_own.xy[1][-1]])
                    pr_int = Point([lpr_int.xy[0][0], lpr_int.xy[1][0]])

                    # convert to lat lon from utm of intruder
                    intruderlat, intruderlon = self.transformer_to_latlon.transform(pr_int.x, pr_int.y)
                    ownshiplat, ownshiplon = self.transformer_to_latlon.transform(pr_own.x, pr_own.y)
                    inter_point_lat, inter_point_lon = self.transformer_to_latlon.transform(p_inter.x, p_inter.y)

                    # assign intruder.lat and intruder.lon with int_x and int_y
                    inttrk, *_ = geo.qdrdist(intruderlat, intruderlon, inter_point_lat, inter_point_lon)                   
                    ownntrk, *_ = geo.qdrdist(inter_point_lat, inter_point_lon, ownshiplat, ownshiplon)

                elif s_own_back.contains(p_own) and s_int_front.contains(p_int):

                    # Case 3: Intersection is in front of ownship
                    # and in back of intruder
                    # this means that p_own is in s_own_front
                    # and p_int is in s_int_back
                    # print('CASE 3: THERE IS AN INTERSECTION')     

                    # keep back part behind ownship and front part from intruder
                    _, s_own = split_line_with_point(s_own_back, p_own)
                    s_int, _ = split_line_with_point(s_int_front, p_int)

                    # first step is to project the ownship and intruder line
                    p1 = Point([s_own.xy[0][-2],  s_own.xy[1][-2]])
                    p2 = Point([s_int.xy[0][1],  s_int.xy[1][1]])

                    s_own_end = LineString([p1, p_inter])
                    s_int_end = LineString([p_inter, p2])
                    
                    own_scale_factor = s_own.length/s_own_end.length
                    int_scale_factor = s_int.length/s_int_end.length

                    lpr_own = scale(s_own_end, xfact=own_scale_factor, yfact=own_scale_factor, origin=p_inter)
                    lpr_int = scale(s_int_end, xfact=int_scale_factor, yfact=int_scale_factor, origin=p_inter)

                    pr_own = Point([lpr_own.xy[0][0], lpr_own.xy[1][0]])
                    pr_int = Point([lpr_int.xy[0][-1], lpr_int.xy[1][-1]])

                    # convert to lat lon from utm of intruder
                    intruderlat, intruderlon = self.transformer_to_latlon.transform(pr_int.x, pr_int.y)
                    ownshiplat, ownshiplon = self.transformer_to_latlon.transform(pr_own.x, pr_own.y)
                    inter_point_lat, inter_point_lon = self.transformer_to_latlon.transform(p_inter.x, p_inter.y)

                    # get trk of intruder and ownship
                    inttrk, *_ = geo.qdrdist(inter_point_lat, inter_point_lon, intruderlat, intruderlon)                   
                    ownntrk, *_ = geo.qdrdist(ownshiplat, ownshiplon, inter_point_lat, inter_point_lon)
                
                else:
                    # ignore if intersection is behind both intruder and ownship
                    continue
                
                # plot_things(p_own, p_int, own_line, int_line, s_own, s_int, p_inter, lpr_own, lpr_int, pr_own, pr_int)

                # check if intersecting pair is in a conflict
                ownshiplats = np.array([ownshiplat, intruderlat])
                ownshiplons = np.array([ownshiplon, intruderlon])
                ownshiptrks = np.array([ownntrk, inttrk])

                intruderlats = np.array([ownshiplat, intruderlat])
                intruderlons = np.array([ownshiplon, intruderlon])
                intrudertrks = np.array([ownntrk, inttrk])

                ownshipids = np.array([ownship_id, intruder_id])
                ownshipgs = np.array([ownship.gs[curr_ownship], intruder.gs[curr_intruder]])
                ownshipalts = np.array([ownship.alt[curr_ownship], intruder.alt[curr_intruder]])
                ownshipvs = np.array([ownship.vs[curr_ownship], intruder.vs[curr_intruder]])

                intruderids = np.array([ownship_id, intruder_id])
                intrudergs = np.array([ownship.gs[curr_ownship], intruder.gs[curr_intruder]])
                intruderalts = np.array([ownship.alt[curr_ownship], intruder.alt[curr_intruder]])
                intrudervs = np.array([ownship.vs[curr_ownship], intruder.vs[curr_intruder]])
            
                # use statebased method if there are intersections
                ntraf_intersecting = 2
                rpz = np.zeros(ntraf_intersecting) + self.rpz_actual
                hpz = np.zeros(ntraf_intersecting) + self.hpz_actual
                dtlookahead = np.zeros(ntraf_intersecting) + self.dtlookahead_actual
                # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
                I = np.eye(ntraf_intersecting)

                # Horizontal conflict ------------------------------------------------------

                qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownshiplats), np.asmatrix(ownshiplons),
                                    np.asmatrix(intruderlats), np.asmatrix(intruderlons))

                # Convert back to array to allow element-wise array multiplications later on
                # Convert to meters and add large value to own/own pairs
                qdr = np.asarray(qdr)
                dist = np.asarray(dist) * nm + 1e9 * I

                # Calculate horizontal closest point of approach (CPA)
                qdrrad = np.radians(qdr)
                dx = dist * np.sin(qdrrad)  # is pos j rel to i
                dy = dist * np.cos(qdrrad)  # is pos j rel to i

                # Ownship track angle and speed
                owntrkrad = np.radians(ownshiptrks)
                ownu = ownshipgs * np.sin(owntrkrad).reshape((1, ntraf_intersecting))  # m/s
                ownv = ownshipgs * np.cos(owntrkrad).reshape((1, ntraf_intersecting))  # m/s

                # Intruder track angle and speed
                inttrkrad = np.radians(intrudertrks)
                intu = intrudergs * np.sin(inttrkrad).reshape((1, ntraf_intersecting))  # m/s
                intv = intrudergs * np.cos(inttrkrad).reshape((1, ntraf_intersecting))  # m/s

                du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
                dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

                dv2 = du * du + dv * dv
                dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
                vrel = np.sqrt(dv2)

                tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

                # Calculate distance^2 at CPA (minimum distance^2)
                dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

                # Check for horizontal conflict
                R2 = rpz * rpz
                swhorconf = dcpa2 < R2  # conflict or not

                # Calculate times of entering and leaving horizontal conflict
                dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
                dtinhor = dxinhor / vrel

                tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
                touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

                # Vertical conflict --------------------------------------------------------

                # Vertical crossing of disk (-dh,+dh)

                dalt = ownshipalts.reshape((1, ntraf_intersecting)) - \
                      intruderalts.reshape((1, ntraf_intersecting)).T  + 1e9 * I

                dvs = ownshipvs.reshape(1, ntraf_intersecting) - \
                    intrudervs.reshape(1, ntraf_intersecting).T
                dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

                # Check for passing through each others zone
                tcrosshi = (dalt + hpz) / -dvs
                tcrosslo = (dalt - hpz) / -dvs
                tinver = np.minimum(tcrosshi, tcrosslo)
                toutver = np.maximum(tcrosshi, tcrosslo)

                # Combine vertical and horizontal conflict----------------------------------
                tinconf = np.maximum(tinver, tinhor)
                toutconf = np.minimum(toutver, touthor)

                swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) * \
                    (tinconf < dtlookahead) * (1.0 - I), dtype=np.bool)

                # --------------------------------------------------------------------------
                # Update conflict lists
                # --------------------------------------------------------------------------
                # Ownship conflict flag and max tCPA
                inconf = np.any(swconfl, 1)
                tcpamax = np.max(tcpa * swconfl, 1)

                # Select conflicting pairs: each a/c gets their own record
                confpair = [(ownshipids[i], ownshipids[j]) for i, j in zip(*np.where(swconfl))]

                # extend the return lists
                # TODO: NUMPYFY THEM

                if len(confpair) > 0:
                    confpairs.append(confpair[0])
                    inconfs[curr_ownship] = inconf[0]
                    tcpamaxs[curr_ownship] = tcpamax[0]
                    qdr_conf.append(qdr[swconfl][0])
                    dist_conf.append(dist[swconfl][0])
                    dcpa_conf.append(np.sqrt(dcpa2[swconfl][0]))
                    tcpa_conf.append(tcpa[swconfl][0])
                    tLOS_conf.append(tinconf[swconfl][0])
                    projected_lats.append(ownshiplats)
                    projected_lons.append(ownshiplons)
                    conftrks.append(ownshiptrks)

            
            return confpairs, inconfs, tcpamaxs, qdr_conf, dist_conf, dcpa_conf, tcpa_conf, \
                tLOS_conf, projected_lats, projected_lons, conftrks

        if len(actual_intersections) > 1:

            # return empty things if there are no intersections
            inconfs = np.full(ownship.ntraf, False, dtype=np.bool)
            tcpamax = np.full(ownship.ntraf, 0)

            return [], inconfs, tcpamaxs, [], [], [], [], [], [], [], []

    def detect_los(self, ownship, intruder, rpz, hpz):
        ''' Conflict detection between ownship (traf) and intruder (traf/adsb).'''

        # Calculate everything using the buffered RPZ
        rpz = np.zeros(len(rpz)) + self.rpz_buffered
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------

        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        dist = np.asarray(dist) * nm + 1e9 * I

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I


        # --------------------------------------------------------------------------
        # Update LOS lists
        # --------------------------------------------------------------------------

        # It's a LOS if the actual RPZ of 32m is violated.
        swlos = (dist < (np.zeros(len(rpz)) + self.rpz_actual)) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]


        return lospairs, qdr, dist



def plot_things(p_own, p_int, own_line, int_line, s_own, s_int, p_inter, lpr_own, lpr_int, pr_own, pr_int):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    p_own = gpd.GeoSeries([
                            p_own, # current point of ownship
                            ], crs='epsg:32633')

    p_own.plot(marker='o', color='blue', ax=ax)

    p_int = gpd.GeoSeries([
                            p_int, # current point of ownship
                            ], crs='epsg:32633')

    p_int.plot(marker='o', color='red', ax=ax)


    s_own = gpd.GeoSeries([
                            s_own, # current point of ownship
                            ], crs='epsg:32633')
    own_line = gpd.GeoSeries([
            own_line, # current point of ownship
            ], crs='epsg:32633')
    int_line = gpd.GeoSeries([
            int_line, # current point of ownship
            ], crs='epsg:32633')

    own_line.plot(color='blue', ax=ax,  linestyle='--')
    int_line.plot(color='red', ax=ax, linestyle='--')

    s_int = gpd.GeoSeries([
                            s_int, # current point of ownship
                            ], crs='epsg:32633')


    
    p_inter = gpd.GeoSeries([
                            p_inter, # current point of ownship
                            ], crs='epsg:32633')

    p_inter.plot(marker='x', color='black', ax=ax)

    lpr_own = gpd.GeoSeries([
                            lpr_own, # current point of ownship
                            ], crs='epsg:32633')
    lpr_int = gpd.GeoSeries([
                            lpr_int, # current point of ownship
                            ], crs='epsg:32633')
    pr_own = gpd.GeoSeries([
                            pr_own, # current point of ownship
                            ], crs='epsg:32633')
    pr_int = gpd.GeoSeries([
                            pr_int, # current point of ownship
                            ], crs='epsg:32633')

    pr_own.plot(marker='*', color='blue', ax=ax)
    pr_int.plot(marker='*', color='red', ax=ax)

    lpr_own.plot(color='blue', ax=ax,  linestyle='-')
    lpr_int.plot(color='red', ax=ax, linestyle='-')

    # funny stuff happening from second part of for loop check
    plt.show()

def split_line_with_point(line, splitter):
    """Split a LineString with a Point
    Code borrowed from shapely
    """

    # point is on line, get the distance from the first point on line
    distance_on_line = line.project(splitter)

    if distance_on_line == 0:
        return LineString([]), line.simplify(0)
    

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

def reverse_geom(geom) -> LineString:
    def _reverse(x, y, z=None):
        if z:
            return x[::-1], y[::-1], z[::-1]
        return x[::-1], y[::-1]

    return transform(_reverse, geom)