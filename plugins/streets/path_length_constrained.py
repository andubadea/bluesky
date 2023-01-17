# %%
import dill
import plugins.streets.agent_path_planning
import plugins.streets.flow_control
import os
import json
import geopandas as gpd
import numpy as np
from pyproj import Transformer
import json
from shapely.geometry import LineString, MultiLineString
from os import path

transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:32633")
transformer_to_latlon = Transformer.from_crs("EPSG:32633", "EPSG:4326")

graph=dill.load(open("graph_data/M2-graph/Flow_control.dill", "rb"))

list_dills = os.listdir('path_plan_dills/')

# constrained_airspca polyogn
constrained_gdf = gpd.read_file('updated_constrained_airspace.gpkg')

constrained_poly = constrained_gdf.geometry[0]

# dictionary to save information
dict_lengths = {}
for dill_to_load in list_dills:
    
    try:
        dill_name, _ = path.splitext(dill_to_load)[0].split('_')
    except ValueError:
        print(dill_to_load)
        continue

    
    path_file = f'path_plan_dills/{dill_to_load}'

    loaded_dill = dill.load(open(path_file, 'rb'),ignore=True)
    loaded_dill.flow_graph=graph

    edges_changes=[]
            
    route,turns,edges,next_turn,groups,in_constrained,turn_speeds=loaded_dill.replan_spawned(edges_changes,
                                                loaded_dill.start_index_previous,loaded_dill.start_index,
                                                loaded_dill.start_point.y,loaded_dill.start_point.x)
    # get coordinates for route in utm
    utm_x = []
    utm_y = []
    

    for j, rte in enumerate(route):
        lat = rte[1] # deg
        lon = rte[0] # deg

        # convert to utm #TODO: check x y in correct order
        x,y = transformer_to_utm.transform(lat, lon)
        utm_x.append(x)
        utm_y.append(y)
    
    lon_origin, lat_origin = route[0]
    lon_dest, lat_dest = route[-1]
    key_orig_dest = f'{lat_origin}-{lon_origin}-{lat_dest}-{lat_origin}'

    if key_orig_dest in dict_lengths:
        continue

    # now create a linestring
    original_route = LineString(list(zip(utm_x, utm_y))).simplify(0.0001)

    # check total length of intersection)
    intersection = original_route.intersection(constrained_poly)

    
    # Check the type of the intersection
    if isinstance(intersection, LineString):
        # Get the total length of the overlapping LineString
        total_length = intersection.length
        dict_lengths[key_orig_dest] = total_length
    elif isinstance(intersection, MultiLineString):
        total_length = 0
        for ls in intersection:
          total_length += ls.length
        dict_lengths[key_orig_dest] = total_length

with open("constrained_path_lengths.json", "w") as json_file:
    json.dump(dict_lengths, json_file, indent=4)
