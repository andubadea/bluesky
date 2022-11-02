import osmnx as ox
import pickle
import numpy as np
import networkx as nx
from shapely.ops import linemerge
from multiprocessing import Pool as ThreadPool
import traceback

def kwikdist(lata, lona, latb, lonb):
    """
    Quick and dirty dist [nm]
    In:
        lat/lon, lat/lon [deg]
    Out:
        dist [nm]
    """

    re      = 6371000.  # radius earth [m]
    dlat    = np.radians(latb - lata)
    dlon    = np.radians(((lonb - lona)+180)%360-180)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle  = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist    = re * dangle

    return dist

# City we are using
city = 'Vienna'
path = f'plugins/scenario_maker/{city}'

# Path requirements
min_dist = 1000 # Metres

# Load the graph for that city
G = ox.load_graphml(f'{path}/streets.graphml')
nodes, edges = ox.graph_to_gdfs(G)


# Load some helper dictionaries to convert node IDs to OSMIDs and back
with open(f'{path}/id2osm.pickle', 'rb') as f:         
    id2osm = pickle.load(f)

with open(f'{path}/osm2id.pickle', 'rb') as f:    
    osm2id = pickle.load(f)

# Load the spawning points for that city, convert em to simple IDs
spawn_nodes_osm = np.genfromtxt(f'{path}/spawn_points.txt', dtype = np.int64)
orig_nodes = [osm2id[x] for x in spawn_nodes_osm]

# Compile the list of destination nodes
dest_nodes = [x for x in G.nodes if x not in orig_nodes]

# Make the input array
input_arr = []
for origin in orig_nodes:
    for destination in dest_nodes:
        input_arr.append([origin, destination])
    
def make_route_pickle(inp):
    orig_node, dest_node = inp
    # Compute distance between these two
    dist = kwikdist(G.nodes[orig_node]['y'], G.nodes[orig_node]['x'], 
                    G.nodes[dest_node]['y'], G.nodes[dest_node]['x'])
    
    if dist > min_dist:
        # Create the path for these two nodes
        route = nx.shortest_path(G, orig_node, dest_node)
        geoms = [edges.loc[(u, v, 0), 'geometry'] for u, v in zip(route[:-1], route[1:])]
        line = linemerge(geoms)
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
        
        route_pickle = list(zip(line.xy[1], line.xy[0], point_edges))
        
        # We also need to create the list of edges
    else:
        return
        
    with open(f'{path}/pickles/{orig_node}-{dest_node}.pkl' , 'wb') as f:
        pickle.dump(route_pickle, f)
        
    return route_pickle

def main():
    pool = ThreadPool(16)
    try:
        _ = pool.map(make_route_pickle, input_arr)
    except:
        pool.close()
        traceback.print_exc()
    pool.close()
    
if __name__ == '__main__':
    main()


