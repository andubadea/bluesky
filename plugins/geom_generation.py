import geopandas as gpd
import shapely.geometry as geom


##### TUNING PARAMETERS #####
# create a point where the center is at
origin_lat = 0
origin_lon = 0

# set the width and depth of a rectangle meters
depth = 500 # meters
width = 4000 # meters

# now set the origin of the aircraft to be a certain distance from the border of rectangle
dist_origin_x = 0 # meters
dist_origin_y = 4000 # meters

# distance from the border of top of rectangle
dist_destination_x = 0 # meters
dist_destination_y = 4000 # meters

##### END TUNING PARAMETERS #####

# create geopandas dataframe with point origin
point_df = gpd.GeoDataFrame(geometry=[geom.Point(origin_lon, origin_lat)], crs="EPSG:4326")

# convert to crs 3857 to work with meters
point_df = point_df.to_crs(epsg=3857)

# get origin point
origin_x = point_df.geometry.x.values[0] 
origin_y  = point_df.geometry.y.values[0] - depth/2 - dist_origin_y
origin_df = gpd.GeoDataFrame(geometry=[geom.Point(origin_x, origin_y)], crs="EPSG:3857")

# get destination point
destination_x = point_df.geometry.x.values[0]
destination_y = point_df.geometry.y.values[0] + depth/2 + dist_destination_y
destination_df = gpd.GeoDataFrame(geometry=[geom.Point(destination_x, destination_y)], crs="EPSG:3857")

# create a rectangle centered at point_df with depth and width
rectangle = geom.box(point_df.geometry.x.values[0] - width/2, point_df.geometry.y.values[0] - depth/2, point_df.geometry.x.values[0] + width/2, point_df.geometry.y.values[0] + depth/2)
rectangle_df = gpd.GeoDataFrame(geometry=[rectangle], crs="EPSG:3857")

# convert everything to lat lon
origin_df = origin_df.to_crs(epsg=4326)
destination_df = destination_df.to_crs(epsg=4326)
rectangle_df = rectangle_df.to_crs(epsg=4326)

# create geodence command which is a sequence of lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4
xy_values = rectangle_df.geometry.values[0].exterior.coords.xy
lat_lon = [f'{lat}, {lon}' for lon, lat in zip(xy_values[0], xy_values[1])]
geo_fence = '00:00:00>GEOFENCE,AIGEO,25000,0, ' + ','.join(lat_lon)

# create an aircraft
cre_command = f'00:00:00>CRE AI01 B744 {origin_df.geometry.y.values[0]} {origin_df.geometry.x.values[0]} 0 FL250 200'

# add a waypoint
addwpt_command = f'00:00:00>ADDWPT AI01 {destination_df.geometry.y.values[0]} {destination_df.geometry.x.values[0]}'

# create a scenario write three lines to the file
with open('scenarios.scn', 'w') as f:
    f.write(geo_fence + '\n')
    f.write(cre_command + '\n')
    f.write(addwpt_command + '\n')
