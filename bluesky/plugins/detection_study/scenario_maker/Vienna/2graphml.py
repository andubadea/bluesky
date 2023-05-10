import geopandas as gpd
import osmnx as ox
import networkx as nx
import shutil

# read gpkgs that are
nodes = gpd.read_file('streets.gpkg', layer='nodes')
edges = gpd.read_file('streets.gpkg', layer='edges')

# set the indices 
edges.set_index(['u', 'v', 'key'], inplace=True)
nodes.set_index(['osmid'], inplace=True)

# ensure that it has the correct value
nodes['x'] = nodes['geometry'].apply(lambda x: x.x)
nodes['y'] = nodes['geometry'].apply(lambda x: x.y)

G = ox.graph_from_gdfs(nodes, edges)
if nx.is_strongly_connected(G):
    print(f'Graph is fully connected.')
    # save grapml
    ox.save_graphml(G, 'streets.graphml')