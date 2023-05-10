import osmnx as ox
import pickle
import numpy as np
import networkx as nx
from shapely.ops import linemerge
from multiprocessing import Pool
import random
import os
from os.path import exists
import tqdm

#Steal kiwkqdrdist function from Bluesky
def kwikqdrdist(lata, lona, latb, lonb):
    """Gives quick and dirty qdr[deg] and dist [nm]
       from lat/lon. (note: does not work well close to poles)"""

    re      = 6371000.  # radius earth [m]
    dlat    = np.radians(latb - lata)
    dlon    = np.radians(((lonb - lona)+180)%360-180)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle  = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist    = re * dangle

    qdr     = np.degrees(np.arctan2(dlon * cavelat, dlat)) % 360

    return qdr, dist

# City we are using
city = 'Vienna'
path = f'{city}'

# Path requirements
min_dist = 1000 # Metres

# Load the graph for that city
G = ox.load_graphml(f'{path}/streets.graphml')
nodes, edges = ox.graph_to_gdfs(G)

# Let's make some origin and destinations from this graph
nodes_already_added = []
attempts = 0
random.seed(0)
while attempts < 100 and len(nodes_already_added)<200:
    node = random.choice(list(G.nodes))
    node_lat = G.nodes[node]['y']
    node_lon = G.nodes[node]['x']
    node_too_close = False
    for existing_node in nodes_already_added:
        existing_node_lat = G.nodes[existing_node]['y']
        existing_node_lon = G.nodes[existing_node]['x']
        _, dist = kwikqdrdist(node_lat, node_lon, existing_node_lat, existing_node_lon)
        if dist < 300:
            # Node too close
            node_too_close = True
            break
    if not node_too_close:
        nodes_already_added.append(node)
        attempts = 0
    else:
        attempts += 1

# Load some helper dictionaries to convert node IDs to OSMIDs and back
with open(f'{path}/id2osm.pickle', 'rb') as f:         
    id2osm = pickle.load(f)

with open(f'{path}/osm2id.pickle', 'rb') as f:    
    osm2id = pickle.load(f)

# Load the spawning points for that city, convert em to simple IDs
spawn_nodes_osm = np.genfromtxt(f'{path}/spawn_points.txt', dtype = np.int64)
orig_nodes = nodes_already_added
orig_nodes_new = []

# Compile the list of destination nodes
dest_nodes = [x for x in G.nodes if x not in orig_nodes]
dest_nodes_new = []
            
# Make the input array by combining all origin nodes with destination nodes
input_arr = []
for origin in orig_nodes:
    for destination in dest_nodes:
        input_arr.append([origin, destination])
        
# Function that creates the route pickle
def make_route_pickle(inp):
    '''Creates a route pickle. 
    This consists in a list that has the following elements:
    Lattitude
    Longitude
    Edge
    Turn WPT Bool'''
    # Parse input
    orig_node, dest_node = inp
    
    # Check if file already exists
    if exists(f'{path}/pickles/{orig_node}-{dest_node}.pkl'):
        return
    
    # Compute distance between the two waypoints
    _, dist = kwikqdrdist(G.nodes[orig_node]['y'], G.nodes[orig_node]['x'], 
                    G.nodes[dest_node]['y'], G.nodes[dest_node]['x'])
    
    if dist > min_dist:
        # Create the path for these two nodes
        route = nx.shortest_path(G, orig_node, dest_node)
        # Extract the path geometry
        geoms = [edges.loc[(u, v, 0), 'geometry'] for u, v in zip(route[:-1], route[1:])]
        line = linemerge(geoms)
        
        # Prepare the edges is a very dumb way.
        point_edges = []
        i = 0
        for geom, u, v in zip(geoms, route[:-1], route[1:]):
            if i == 0:
                # First edge, also take the first waypoint
                for coord in geom.coords:
                    point_edges.append([u,v])
            else:
                first = True
                for coord in geom.coords:
                    if first:
                        first = False
                        continue
                    point_edges.append([u,v])
            i += 1
        
        # Also prepare the turns
        latlons = list(zip(line.xy[1], line.xy[0]))
        turns = [True] # Always make first wpt a turn
        i = 1
        for lat_cur, lon_cur in latlons[1:-1]:
            # Get the needed stuff
            lat_prev, lon_prev = latlons[i-1]
            lat_next, lon_next = latlons[i+1]
            
            # Get the angle
            d1=kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
            d2=kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
            angle=abs(d2[0]-d1[0])

            if angle>180:
                angle=360-angle
                
            # This is a turn if angle is greater than 25
            if angle > 25:
                turns.append(True)
            else:
                turns.append(False)
                
            i+= 1
                
        #Last waypoint is always a turn one.        
        turns.append(True)
        # Pack everything up
        route_pickle = list(zip(line.xy[1], line.xy[0], point_edges, turns))

    else:
        # Return to not create a pickle if path is too short.
        return
        
    with open(f'{path}/pickles/{orig_node}-{dest_node}.pkl' , 'wb') as f:
        pickle.dump(route_pickle, f)
    return route_pickle

def main():
    print(f'Found {len(nodes_already_added)} spawn points.')
    with Pool(8) as p:
        _ = list(tqdm.tqdm(p.imap(make_route_pickle, input_arr), total = len(input_arr)))
        p.close()
    pass
    
if __name__ == '__main__':
    main()

orig_dest_dict = dict()
files_that_exist = os.listdir(f'{path}/pickles')
for filename in files_that_exist:
    # If pkl not in file, skip
    if 'pkl' not in filename:
        continue
    # First is origin, second is destination
    split_filename = filename.replace('.pkl', '').split('-')
    orig = int(split_filename[0])
    dest = int(split_filename[1])
    if orig not in orig_dest_dict:
        orig_dest_dict[orig] = []
        
    orig_dest_dict[orig].append(dest)

# Save orig_nodes and dest_nodes to a file
with open(f'{path}/orig_dest_dict.pickle', 'wb') as f:
    pickle.dump(orig_dest_dict, f)
