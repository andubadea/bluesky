''' Detects geofences in the vicinity of each aircraft. Detection here does
not mean aircraft and geofence are in conflict. '''
import numpy as np
import bluesky as bs
from bluesky.traffic import traffic
from shapely.ops import nearest_points
from shapely.geometry.polygon import Polygon, Point
from bluesky.tools import geo
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'GFCD',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config
class Tile:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
class GeofenceDetection():
    def __init__(self):
        super().__init__()
        self.geoconfs = []
        return
                
    # This function is actually called from within detect. The geofence resoltion is also
    # updated from this function
    def detect(self,ownship, intruder, rpz, hpz, dtlookahead):
        # Retrieve geofence plugin instance
        geofenceplugin = bs.core.varexplorer.varlist['geofence'][0]
        # Get geofence data from plugin
        geofenceTileData = geofenceplugin.geofenceTileData
        geofenceData = geofenceplugin.geofences
        # Get zoom level
        z = geofenceTileData.z
        
        # For now, detection is a bit unoptimised as in we need to loop over all aircraft and
        # see if there are geofences in their tiles.
        ntraf = ownship.ntraf
        
        # Retain conflicts between aircraft and geofences: (ACID, AC_IDX, GEOFENCE NAME)
        self.geoconfs = []
        
        for i in np.arange(ntraf):
            # Get aircraft position and tile
            aclat, aclon = ownship.lat[i], ownship.lon[i]
            actileX, actileY = self.tileXY(aclat, aclon, z)
            
            # Get adjacent tiles depending on zoom level
            # TODO: actually make it depend on zoom level
            geofence_names = set()
            tiles = self.getAdjacentTiles(actileX, actileY)
            
            # Retrieve names of geofences in considered tiles
            for tile in tiles:
                tile = (tile.x, tile.y)
                if tile in geofenceTileData.tiledictionary:
                    geofence_names = geofence_names.union(geofenceTileData.tiledictionary[tile])

            if geofence_names:
                for geofence_name in geofence_names:
                    # Get the geofence itself
                    geofence = geofenceData[geofence_name]
                    # Create shapely polygon
                    geofencepoly = Polygon(geofence.getPointArray())
                    # Find closest point on this polygon w.r.t aircraft
                    p1, p2 = nearest_points(geofencepoly, Point(aclat, aclon))
                    # Get closest point Coordinates
                    plat, plon = p1.x, p1.y
                    # Get distance between aircraft and geofence
                    distance = geo.latlondist(aclat, aclon, plat, plon)
                    print(ownship.id[i], geofence_name, distance)
                    if distance < 10 * 1852:
                        self.geoconfs.append((ownship.id[i], i, geofence_name))
        return
                
        
        
        
        
    # Helper functions   
    def getAdjacentTiles(self, tileX, tileY):
        tilelist = []
        tilelist.append(Tile(tileX, tileY))
        return tilelist
    
    def bbox_corner_tiles(self, lat1, lon1, lat2, lon2, z):
        tile_nw = self.tileXY(lat1, lon1, z)
        tile_ne = self.tileXY(lat1, lon2, z)
        tile_sw = self.tileXY(lat2, lon1, z)
        tile_se = self.tileXY(lat2, lon2, z)
        return tile_nw, tile_ne, tile_sw, tile_se

    def tile_corners(self, x, y, z):
        lat2, lon1, lat1, lon2 = self.tileEdges(x, y, z)
        corners = (lat2, lon2), (lat2, lon1), (lat1, lon1), (lat1, lon2)
        # order is SE, SW, NW, NE
        return corners

    def tileEdges(self, x, y, z):
        lat1, lat2 = self.latEdges(y, z)
        lon1, lon2 = self.lonEdges(x, z)
        return lat2, lon1, lat1, lon2  # S,W,N,E

    def tileXY(self, lat, lon, z):
        x, y = self.latlon2xy(lat, lon, z)
        return int(x), int(y)

    def latlon2xy(self, lat, lon, z):
        n = self.numTiles(z)
        x, y = self.latlon2relativeXY(lat, lon)
        return n * x, n * y

    def latEdges(self, y, z):
        n = self.numTiles(z)
        unit = 1 / n
        relY1 = y * unit
        relY2 = relY1 + unit
        lat1 = self.mercatorToLat(np.pi * (1 - 2 * relY1))
        lat2 = self.mercatorToLat(np.pi * (1 - 2 * relY2))
        return lat1, lat2

    def lonEdges(self, x, z):
        n = self.numTiles(z)
        unit = 360 / n
        lon1 = -180 + x * unit
        lon2 = lon1 + unit
        return lon1, lon2

    def xy2latlon(self, x, y, z):
        n = self.numTiles(z)
        relY = y / n
        lat = self.mercatorToLat(np.pi * (1 - 2 * relY))
        lon = -180.0 + 360.0 * x / n
        return lat, lon

    def latlon2relativeXY(self, lat, lon):
        x = (lon + 180) / 360
        y = (1 - np.log(np.tan(np.radians(lat)) + self.sec(np.radians(lat))) / np.pi) / 2
        return x, y

    @staticmethod
    def numTiles(z):
        return pow(2, z)

    @staticmethod
    def mercatorToLat(mercatorY):
        return np.degrees(np.arctan(np.sinh(mercatorY)))

    @staticmethod
    def sec(x):
        return 1 / np.cos(x)
        