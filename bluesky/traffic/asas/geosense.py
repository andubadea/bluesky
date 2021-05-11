''' This file serves to "sense" geofences around aircraft, it does
not output geofences that are in conflict with aircraft, but it servers
as an input to such a file. It was created to avoid for loops over all
geofences when detecting them. It also keeps track whether an aircraft is
inside a geofence or outside using a dictionary. The condition for this is
that aircraft NEED to have a starting position outside ANY geofence.'''
import numpy as np
import bluesky as bs
from bluesky.traffic import traffic
from shapely.ops import nearest_points
from shapely.geometry.polygon import Polygon, Point
from bluesky.tools import geo
from bluesky.tools.aero import nm

class Tile:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
class GeofenceSense():
    def __init__(self):
        super().__init__()
        # Dictionaries to keep track of whether aircraft are inside or
        # outside geofences. The only reason these exist is that, if the geofence is large
        # enough (or tile zoom is large enough) and an aircraft is inside it, because 
        # of the tile-based optimisation, it will not be detected. For this situation 
        # specifically, we need to keep track of aircraft that are inside a geofence. 

        # First dictionary can be used to check if an aircraft is inside any geofences.
        # If aircraft ID is not present, then aircraft is not inside any geofence. Same if
        # entry for ACID is empty. 
        self.acingeofence = dict()

        # Second dictionary checks whether a geofence has aircraft inside it. If geofence name
        # is not present, there are no aircraft inside it. Same if entry for geofence name is
        # empty. 
        self.geofencehasac = dict()

        # Dictionary to find the geofences in vicinity of aircraft for further use in detecting
        # conflicts. Dictionary is shaped like this: 
        # ACID : {geofence objects}
        self.geoinvicinity = dict()
        return
    
    # This function is called from within traffic
    def update(self, ownship):
        self.detect(ownship)   
        
    def reset(self):
        self.acingeofence = dict()
        self.geofencehasac = dict()
        self.geoinvicinity = dict()
        
    def delgeofence(self, geofencename):
        '''Delete geofence from the dictionaries in this class.'''
        # First do the easy bit, simply remove geofence entry
        if geofencename in self.geofencehasac:
            self.geofencehasac.pop(geofencename)
            
        # Then the harder part. For loop over dictionary
        for key in self.acingeofence:
            if geofencename in self.acingeofence[key]:
                self.acingeofence[key].remove(geofencename)
        
    def delaircraft(self, acid):
        '''Delete acid from the dictionaries in this class.'''
        # Easy bit, remove ac entry
        if acid in self.acingeofence:
            self.acingeofence.pop(acid)
            
        # Loop over geofence dictionary
        for key in self.geofencehasac:
            if acid in self.geofencehasac[key]:
                self.geofencehasac[key].remove(acid)
                
    def detect(self, ownship):
        # Check if geofence plugin is enabled
        if 'geofence' not in bs.core.varexplorer.varlist:
            return 'Geofence plugin not loaded.'
        # Retrieve geofence plugin instance
        geofenceplugin = bs.core.varexplorer.varlist['geofence'][0]
        
        # Get geofence data from plugin
        geofenceTileData = geofenceplugin.TileData
        geofenceData = geofenceplugin.geofences
        
        # If there are no geofences then there is no point in continuing
        if not geofenceData:
            return
        
        # Get zoom level
        z = geofenceTileData.z
        
        # For now, detection is a bit unoptimised as in we need to loop over all aircraft and
        # see if there are geofences in their tiles. But as long as the number of aircraft is
        # not exaggerated (like, 10k at once), then this is fine. 
        ntraf = ownship.ntraf
        
        # Retain conflicts between aircraft and geofences: (ACID, AC_IDX, GEOFENCE NAME)
        self.geoinvicinity = dict()
        for i in np.arange(ntraf):
            acid = ownship.id[i]
            # Get aircraft position and tile
            aclat, aclon = ownship.lat[i], ownship.lon[i]
            actileX, actileY = self.tileXY(aclat, aclon, z)
            
            # This is a set to automatically avoid duplicate geofence names
            geofence_names = set()
            
            # Get adjacent tiles depending on lookahead distance or time
            tiles = self.getAdjacentTiles(actileX, actileY, z)
            
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
                    # Find closest point on this polygon w.r.t aircraft, p2 unused
                    # When geofences are very big, this becomes inaccurate, as this does
                    # not take into account the curvature of the Earth. 
                    # TODO: find a solution for this to follow curvature of Earth. 
                    p1, p2 = nearest_points(geofencepoly, Point(aclat, aclon))
                    # Get closest point Coordinates
                    plat, plon = p1.x, p1.y
                    # Get distance between aircraft and geofence
                    distance = geo.latlondist(aclat, aclon, plat, plon)
                    if distance < bs.settings.geofence_dlookahead:
                        # Geofence in vicinity, add the object itself to dictionary
                        if acid not in self.geoinvicinity:
                            self.geoinvicinity[acid] = set()
                        
                        self.geoinvicinity[acid].add(geofence)
                        
                    
                    # Inside check
                    if distance == 0:
                        # We are inside the geofence. First, create entry for aircraft
                        if acid not in self.acingeofence:
                            self.acingeofence[acid] = list()
                        
                        # Append geofence name to the list of geofences if not already there
                        if geofence_name not in self.acingeofence[acid]:
                            self.acingeofence[acid].append(geofence_name)
                        
                        # Do the same for the other dictionary
                        if geofence_name not in self.geofencehasac:
                            self.geofencehasac[geofence_name] = list()
                            
                        # Append acid to geofence name entry
                        if acid not in self.geofencehasac[geofence_name]:
                            self.geofencehasac[geofence_name].append(acid)
                    
                    else:
                        # We are outside the geofence
                        # If entry for geofence name or acid do not exist in the dictionaries, then
                        # we do nothing
                        if acid not in self.acingeofence or geofence_name not in self.geofencehasac:
                            continue
                        
                        # Remove geofence name from acid dictionary, and vice versa
                        if geofence_name in self.acingeofence[acid]:
                            self.acingeofence[acid].remove(geofence_name)
                            
                        if acid in self.geofencehasac[geofence_name]:
                            self.geofencehasac[geofence_name].remove(acid)
                            
                        # Remove the keys entirely if list is now empty
                        if not self.acingeofence[acid]:
                            self.acingeofence.pop(acid)
                            
                        if not self.geofencehasac[geofence_name]:
                            self.geofencehasac.pop(geofence_name)   
        return
                
    # Helper functions   
    def getAdjacentTiles(self, tileX, tileY, z):
        # We want to look inside a 3x3 grid of tiles around the aircraft
        tilelist = []
        for x in [tileX-1, tileX, tileX + 1]:
            # If x is negative, or larger than the greatest possible x, skip
            if x < 0 or x > (2**z)-1:
                continue
            
            for y in [tileY-1, tileY, tileY +1]:
                # Do the same check as before
                if y < 0 or y > (2**z)-1:
                    continue
                
                # Append tile
                tilelist.append(Tile(x, y))
        
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