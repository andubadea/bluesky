''' Plugin that creates and keeps track of geofences.'''
# Geofences need shapely
import bluesky as bs
import numpy as np

def init_plugin():
    # Create new geofences dictionary
    reset()
    # Create new point search MATRIX
    
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name'      : 'GEOFENCE',
        'plugin_type'      : 'sim',
        'update_interval'  :  1.0,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        # 'update':          update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        # 'preupdate':       preupdate,

        # Reset all geofences
        'reset':         reset
        }

    # Add two commands: GEOFENCE to define a geofance
    stackfunctions = {
        # Defining a geofence
        'GEOFENCE': [
            'GEOFENCE Name, [lat,lon,lat,lon...]',
            'txt,[latlon,...]',
            defgeofencepoly,
            'Define a poly geofence']
        ,
        'GFTILEZOOM': [
            'GFTILEZOOM Int between 1-12',
            'int',
            setZ,
            'Change tile zoom level.']
        ,
        # Delete a geofance
        'DELGEOFENCE': [
            'DELGEOFENCE name',
            'txt',
            delgeofence,
            'Remove geofence']
        ,
        'RESETGEOFENCES': [
            'RESETGEOFENCES',
            '',
            reset,
            'Delete all geofences']
        }
    # init_plugin() should always return these two dicts.
    return config, stackfunctions

#------------------------------ Helper Classes -----------------------------------------
class Point:
    '''A point [latlon] that has a geofence name attached to it.'''
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        
class Line:
    def __init__(self, Point1, Point2):
        self.Point1 = Point1
        self.Point2 = Point2
        
class Geofence:
    ''' Geofence specification class. '''
    def __init__(self, name, coordinates, topalt=1e9, bottomalt=-1e9):
        self.name = name
        self.coordinates = coordinates
        self.points = self.getPoints()
        self.edges = self.getEdges()
        self.topalt = topalt
        self.bottomalt = bottomalt
        
    def getEdges(self):
        ''' Returns a list of the edges of the geofence as a list of lines.'''
        edges = []
        for i,point in enumerate(self.points):
            edge = Line(self.points[i-1], self.points[i])
            edges.append(edge)
        return edges
    
    def getPoints(self):
        ''' Returns a list of points created from the latlon coord list.'''
        pointsarr = self.getPointArray()
        points = []
        for point in pointsarr:
            points.append(Point(point[0], point[1]))
        return points
    
    def getPointArray(self):
        '''Returns the points not as point objects but as an array.'''
        lats = self.coordinates[::2]
        lons = self.coordinates[1::2]
        pointsarr = np.array([lats, lons])
        return pointsarr.T
        
# Class for tile data container
class GeofenceTileData():
    ''' Stores tile data for geofence, make searching for a geofence
    on the map really easy and fast as long as you know the tile the aircraft is
    in. Tiles are defined by map tile coordinates: 
    https://developers.google.com/maps/documentation/javascript/coordinates?hl=ko
    Use 'tiledictionary' to look up the names of the geofence SIDES that
    cross a certain tile, and then retrieve those geofences from
    the global 'geofences' dictionary.
    '''
    def __init__(self):
        ''' Z is the zoom level.'''
        self.z = 10 # Default zoom level
        self.size = self.numTiles(self.z)
        # Dictionary that links tiles to geofence names. This is the one that
        # should be used when detecting geofences in range of an aircraft.
        # {tile: [geofence names]}
        self.tiledictionary = dict()
        # Dictionary that links geofence names to tiles. This is used to
        # make geofence deletion faster from the other dictionary.
        # {geofence name: [tiles]}
        self.geodictionary = dict()
        
    def setZ(self, z):
        self.z = z
        
    def getGeofenceTiles(self, Geofence):
        ''' Retrieves the tiles spanned by the sides of a geofence.'''
        GeofenceTiles = []
        edges = Geofence.edges
        for edge in edges:
            Tiles = self.getVertexTiles(edge.Point1.lat, edge.Point1.lon, 
                                         edge.Point2.lat, edge.Point2.lon)
            GeofenceTiles.extend(Tiles)
            
        return list(dict.fromkeys(GeofenceTiles))
    
    def addGeofence(self, Geofence):
        '''Adds a geofence name to the respective tiles in the dictionary.'''
        name = Geofence.name
        tiles = self.getGeofenceTiles(Geofence)
        for tile in tiles:
            # Handle tiledictionary
            if tile in self.tiledictionary:
                self.tiledictionary[tile].add(name)
            else:
                # Create entry
                self.tiledictionary[tile] = set([name])
                
            # Handle geodictionary
            if name in self.geodictionary:
                # append tile
                self.geodictionary[name].add(tile)
            else:
                # create entry
                self.geodictionary[name] = set([tile])
        print(self.geodictionary)
        
    def removeGeofence(self, GeofenceName):
        ''' Removes a geofence from the tile dictionary. Only uses
        geofence name.'''
        name = GeofenceName
        # Remove name from tiles
        for tile in self.geodictionary[name]:
            # Delete geofence name from tile entry
            self.tiledictionary[tile].remove(name)
            # Delete entry alltogether if tile only contained one geofence and
            # set is empty now
            if not self.tiledictionary[tile]:
                self.tiledictionary.pop(tile)
        # Finally, remove entry from geodictionary
        self.geodictionary.pop(name)
        
    def clear(self):
        self.tiledictionary.clear()
        self.geodictionary.clear()
    
    def getVertexTiles(self, lat1, lon1, lat2, lon2):
        '''Returns the tiles crossed by the geofence vertex.'''
        z = self.z
        # Get the start and end tile coordinates
        startTileX, startTileY = self.tileXY(lat1, lon1, z)
        endTileX, endTileY   = self.tileXY(lat2, lon2, z)
        
        #print('startend', (startTile.x, startTile.y), (endTile.x, endTile.y))
        
        # Get the brearing of the two points
        brg = self.kwikqdr(lat1, lon1, lat2, lon2)
        
        # Get the directions we're moving in (-1 or 1)
        latdir, londir = self.getLatLonSigns(lat1, lon1, lat2, lon2)
        
       # print('latlondir', latdir, londir)
        
        # Tiles use different coordinate system with origin in top left corner 
        # of world map
        xtiledir = londir
        ytiledir = -latdir
        
        #print('xydir', xtiledir, ytiledir)
        
        # Get tile point index
        itp = self.getCornerIndex(latdir, londir)
        
        #print('itp', itp)
        
        # Initialize vars
        tiles = []
        
        # Append first tile
        tiles.append((startTileX, startTileY))
        # Initialize current tile
        currentTileX, currentTileY = startTileX, startTileY
        
        while True:
            # Stop if we are in final tile
            if (currentTileX, currentTileY) == (endTileX, endTileY):
                return tiles
            # Take corner of current tile
            corner = self.getTileCorners(currentTileX, currentTileY, z)[itp]
            
            # Get bearing with respect of point 1
            crnBrg = self.kwikqdr(lat1, lon1, corner[0], corner[1])
            
            # Get bearing difference
            brgDff = self.getBearingDifference(brg, crnBrg)
            
            # Bearing difference depends on which direction we're going into. If
            # we're going either SW or NE, if bearing difference is positive, it
            # means the line intersects the bottom/top (lattitude) edge of the tile.
            # The opposite happens if we're going in SE or NW direction.
            if latdir * londir > 0:
                if brgDff >= 0:
                    # Move in y direction to next tile
                    nextTileX, nextTileY = currentTileX, currentTileY + ytiledir
                else:
                    # Move in x direction to next tile
                    nextTileX, nextTileY = currentTileX + xtiledir, currentTileY
            else:
                # Do the opposite of above
                if brgDff < 0:
                    # Move in y direction to next tile
                    nextTileX, nextTileY = currentTileX, currentTileY + ytiledir
                else:
                    # Move in x direction to next tile
                    nextTileX, nextTileY = currentTileX + xtiledir, currentTileY
            
            # Add to tile list
            tiles.append((int(nextTileX), int(nextTileY)))
            
            # Move to next tile
            currentTileX, currentTileY = nextTileX, nextTileY
            
            # Failsafe
            if currentTileX < 0 or currentTileY < 0 or currentTileX > self.numTiles(z) or currentTileY > self.numTiles(z):
                print('fail', currentTileX, currentTileY, self.numTiles(z), z)
                return tiles
                    
        return tiles
        
        
    # Helper functions   
    
    def getBearingDifference(self, b1, b2):
    	r = (b2 - b1) % 360.0
    	# Python modulus has same sign as divisor, which is positive here,
    	# so no need to consider negative case
    	if r >= 180.0:
    		r -= 360.0
    	return r
    
    
    def kwikqdr(self, lata, lona, latb, lonb):
        """Gives quick and dirty qdr[deg]
           from lat/lon. (note: does not work well close to poles)"""
    
        dlat    = np.radians(latb - lata)
        dlon    = np.radians(lonb - lona)
        cavelat = np.cos(np.radians(lata + latb) * 0.5)
    
        qdr     = np.degrees(np.arctan2(dlon * cavelat, dlat)) % 360.
    
        return qdr
    
    def getLatLonSigns(self, lata, lona, latb, lonb):
        """ Gives the direction in which we're moving in lat/lon. """
        difflat = latb - lata
        difflon = lonb - lona
        return difflat/abs(difflat), difflon/abs(difflon)
    
    def getCornerIndex(self, latdir, londir):
        '''Gets the tile corner index that is going to be used for tile calculation'''
        if latdir < 0 and londir < 0:
            # We're going in negative directions of both, take SW corner
            return 1
        if latdir < 0 and londir > 0:
            # Negative lat direction, positive lon direction, take SE
            return 0
        if latdir > 0 and londir > 0:
            # Positive direction both, take NE
            return 3
        if latdir > 0 and londir < 0:
            # NW
            return 2
    
    def getTileCorners(self, x, y, z):
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
    
    def numTiles(self, z):
        return np.power(2, z)
    
    def mercatorToLat(self, mercatorY):
        return np.degrees(np.arctan(np.sinh(mercatorY)))
    
    def sec(self, x):
        return 1 / np.cos(x)
    
# ----------------------------- End of Helper Classes -----------------------------

# ----------------------------- Plugin Functions ----------------------------------
# A dictionary of geofences {name : Geofence object}
geofences = dict()
# A container that links tile data and geofence data
geofenceTileData = GeofenceTileData()

def preupdate():
    return

### Periodic update functions that are called by the simulation. You can replace
### this by anything, so long as you communicate this in init_plugin
def update(): # Not used
    return

def reset():
    for geofencename in geofences:
        bs.scr.objappend('', geofencename, None)
    geofences.clear()
    geofenceTileData.clear()
    return

def setZ(z):
    ''' Change the tile zoom level for the geofence dictionary. 
    If this is done, we need to reset the geofenceTileData dictionaries
    and add all geofences one by one again. This is a lengty process
    if there are a lot of geofences and/or the zoom level is high.
    The zoom level is capped at 12 for now.'''
    # Cap the zoom level between 1 and 12
    if z>12:
        z = 12
    if z<1:
        z = 1
        
    # Set the new zoom level
    geofenceTileData.setZ(z)
    # Clear dictionaries
    geofenceTileData.clear()
    # Add geofences from dictionary one by one
    for geofence in geofences.values():
        geofenceTileData.addGeofence(geofence)

    return True, f'Zoom successfully changed to level {z}.'

### Other functions of your plug-in
def hasGeofence(geofence_name):
    """Check if area with name 'areaname' exists."""
    return geofence_name in geofences

# Polygon geofence
def defgeofencepoly(*args):
    '''Create poly geofence.'''
    # Check if input was correct
    if not args or len(args) < 5:
        return True, 'GEOFENCEPOLY Name, [lat,lon,lat,lon...]'
    
    if isinstance(args[0], str):
        if hasGeofence('GEOFENCE:'+args[0]):
            return False, f'Geofence {args[0]} already exists.'
    else:
        return False, 'Syntax error. \nGEOFENCEPOLY Name, [lat,lon,lat,lon...]'    
    
    # Create geofence naming to not be confused with another area
    geofencename = 'GF:'+args[0]
    coordinates = args[1:]
    top=1e9 
    bottom=-1e9
    # Add geofence to dictionary of geofences
    myGeofence = Geofence(geofencename, coordinates, top, bottom)
    geofences[geofencename] = myGeofence
    
    # Create shape on screen
    bs.scr.objappend('POLY', geofencename, coordinates)
    
    # Make geofence red
    data = dict(color=(255, 0, 0))
    data['polyid'] = geofencename
    bs.net.send_event(b'COLOR', data)
    
    # Add geofence to tile data container
    geofenceTileData.addGeofence(myGeofence)
    
    return True, f'Created geofence {args[0]}.'

# Delete geofence
def delgeofence(name=""):
    geofencename = 'GF:'+name
    # Delete geofence if name exists
    found = geofences.pop(geofencename)
    if not found:
        return False, f'No geofence found for {name}'
    # Remove geofence from screen
    bs.scr.objappend('', geofencename, None)
    
    # Remove geofence from all dictionaries
    geofenceTileData.removeGeofence(geofencename)
    
    return True, f'Deleted geofence {name}'

def retrievegeofences():
    return geofences

def bounds(coordinates):
    ''' Returns tuple (minlat, minlon, maxlat, maxlon)'''
    # Coordinates are latlon, extract each
    lats = coordinates[::2]
    lons = coordinates[1::2]
    return (min(lats), min(lons), max(lats), max(lons))