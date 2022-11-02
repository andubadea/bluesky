import osmnx as ox

city = 'Vienna'

G = ox.load_graphml(f'{city}/streets.graphml')

