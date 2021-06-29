""" Detection of geofence conflicts."""
import numpy as np
import bluesky as bs
from bluesky.core import Entity
from shapely.ops import nearest_points
from shapely.geometry.polygon import Polygon, Point, LineString
from bluesky.tools import geo
from bluesky.tools.aero import nm

class Tile:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
class GeofenceDetection(Entity):
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
        
        # Conflict detection
        self.geoconfs = dict() # Stores current conflicts
        self.geobreaches = dict() # Stores current breaches
        
        # Array to store if an aircraft is in conflict with a geofence
        with self.settrafarrays():
            self.active = np.array([]) 
        
        # Historic lists
        self.allgeobreaches = [] # Stores all pastgeofence breaches 
        self.allgeoconfs = [] # Stores all past geofence conflicts
        return
    
    # This function is called from within traffic
    def update(self, ownship):
        self.Geodetect(ownship)
        return
        
    def reset(self):
        self.acingeofence = dict()
        self.geofencehasac = dict()
        self.geoconfs = dict()
        self.geobreaches = dict()
        self.allgeobreaches = []
        self.allgeoconfs = []
        self.active = np.array([]) 
        return
        
    def delgeofence(self, geofencename):
        '''Delete geofence from the dictionaries in this class.'''
        # First do the easy bit, simply remove geofence entry
        if geofencename in self.geofencehasac:
            self.geofencehasac.pop(geofencename)
            
        # Then the harder part. For loop over dictionary
        for key in self.acingeofence:
            if geofencename in self.acingeofence[key]:
                self.acingeofence[key].remove(geofencename)
        return
        
    def delaircraft(self, acid):
        '''Delete acid from the dictionaries in this class.'''
        # Easy bit, remove ac entry
        if acid in self.acingeofence:
            self.acingeofence.pop(acid)
            
        # Loop over geofence dictionary
        for key in self.geofencehasac:
            if acid in self.geofencehasac[key]:
                self.geofencehasac[key].remove(acid)
        return
    
    def Geodetect(self, ownship):
        ''' This detection method uses spacial indexing and bounding boxes. It is only accurate
        on a small scale as it basically assumes the world is flat.'''
        self.geoconfs = dict()

        # Check if geofence plugin is enabled
        if 'geofence' not in bs.core.varexplorer.varlist:
            return 'Geofence plugin not loaded.'
        
        # Load the plugin
        geofenceplugin = bs.core.varexplorer.varlist['geofence'][0]
        
        # Get the necessary data from the plugin
        geofenceData = geofenceplugin.geofences
        geoidx = geofenceplugin.geoidx
        geoidx_idtogeo = geofenceplugin.geoidx_idtogeo
        
        # Skip everything if no geofences are defined.
        if not geofenceData:
            return
        
        # Create the dict
        self.geoinvicinity = dict()
        ntraf = ownship.ntraf
        dlookahead = bs.settings.geofence_dlookahead
        
        for i in range(ntraf):
            geoinvicinity = []
            acid = ownship.id[i]
            # Get aircraft position and tile
            pos_ac = np.array([ownship.lat[i], ownship.lon[i], ownship.alt[i]])
            hdg_ac = ownship.trk[i]
            
            # Project the current position of the aircraft in the direction of the heading
            pos_next = geo.qdrpos(pos_ac[0], pos_ac[1], hdg_ac, dlookahead / nm)
            
            # Create a line for convenience, as line.bounds gives the correct format for the rtree
            trajectory = LineString(np.array([pos_ac[:2], pos_next]))
            
            # Find the list of potentially intersecting geofences
            intersectionlist = list(geoidx.intersection(trajectory.bounds))
            
            for id in intersectionlist:
                geofencename = geoidx_idtogeo[id]
                geoinvicinity.append(geofenceData[geofencename])
                
            # Detect conflicts with geofences
            self.GeoconfDetect(acid, i, trajectory, geoinvicinity)
        return
            
    def GeoconfDetect(self, acid, idx_ac, trajectory, geoinvicinity):
        ''' Detects conflicts between an aircraft an the geofences in its vicinity.
        Assumes flat earth.'''
        # Iterate over geofence objects and check if there is an intersection
        for geofence in geoinvicinity:
            # First do horizontal check. We check using shapely if projected line intersects polygon.
            geopoly = geofence.getPoly()
            if trajectory.intersects(geopoly):
                # We have a conflict, add conflict to dictionary
                if acid not in self.geoconfs:
                    self.geoconfs[acid] = set()
                
                # Add geofence to this set
                self.geoconfs[acid].add(geofence)
                self.active[idx_ac] = True 
        
        print(self.geoconfs)
        
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