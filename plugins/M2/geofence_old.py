''' Plugin that creates and keeps track of geofences.'''
# Geofences need shapely
import bluesky as bs
import shapely
import numpy as np
import pickle
import os
from rtree import index

def init_plugin():
    # Create new geofences dictionary
    reset()
    # Create new point search MATRIX
    
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name'      : 'GEOFENCEOLD',
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
        ,
        'LOADGEOFENCES': [
            'LOADGEOFENCES name, graphics? [Y/N] ',
            '[txt,txt]',
            loadFromFile,
            'Load geofence data from specified .pkl file.']
        ,
        'SAVEGEOFENCES': [
            'SAVEGEOFENCES name',
            'txt',
            saveToFile,
            'Save geofence data to specified .pkl file.']
        ,
        'GEODETECTMETHOD': [
            'GEODETECTMETHOD name',
            'txt',
            detectmethod,
            'Set the geofence detection method. TILE is slower but more accurate, RTREE is faster but less accurate.']
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
        self.lats = self.coordinates[::2]
        self.lons = self.coordinates[1::2]
        self.pointsarr = self.getPointArray()
        self.points = self.getPoints()
        self.edges = self.getEdges()
        self.topalt = topalt
        self.bottomalt = bottomalt
        self.poly = shapely.geometry.Polygon(self.getPointArray())
        self.bbox = self.poly.bounds
        
    def getEdges(self):
        ''' Returns a list of the edges of the geofence as a list of lines.'''
        edges = []
        for i,point in enumerate(self.points):
            edge = Line(self.points[i-1], self.points[i])
            edges.append(edge)
        return edges
    
    def getPoints(self):
        ''' Returns a list of points created from the latlon coord list.'''
        points = []
        for point in self.pointsarr:
            points.append(Point(point[0], point[1]))
        return points
    
    def getPointArray(self):
        '''Returns the points not as point objects but as an array.'''
        lats = self.coordinates[::2]
        lons = self.coordinates[1::2]
        pointsarr = np.array([lats, lons])
        return pointsarr.T
    
    def getPoly(self):
        return self.poly
        
        
        
        
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
        # Dictionary that links tiles to geofence names. This is the one that
        # should be used when detecting geofences in range of an aircraft.
        # {tile: [geofence names]}
        self.tiledictionary = dict()
        # Dictionary that links geofence names to tiles. This is used to
        # make geofence deletion faster from the other dictionary.
        # {geofence name: [tiles]}
        self.geodictionary = dict()
        # Zoom level related
        self.calcZ(bs.settings.geofence_dlookahead) # Default zoom level
        self.size = self.numTiles(self.z)
        
    def setZ(self, z):
        # Cap the zoom level between 1 and 18
        if z>18:
            z = 18
        if z<1:
            z = 1
        self.z = z
        self.tiledictionary['ZOOM'] = self.z
        self.geodictionary['ZOOM'] = self.z
        
    def calcZ(self, dlookahead):
        # Calculate the zoom level in function of required lookahead distance
        # Tiles don't all have the same edge distance, so we need to assume an
        # average case (i.e., tile is at latitude 45 deg). 
        # Aircraft will be looking in a 3x3 tile area. We need to make sure that the
        # edge of this area is at least twice the lookahead time. This means that the
        # edge of a tile needs to be 2/3 of dlookahead time at minimum in length. 
        # So, minimum edge length:
        min_edge_length = dlookahead * 2 / 3 # dlookahead in meters
        circumference_45 = 28305000 # meters
        max_numtiles = circumference_45 / min_edge_length # Keep it as float for now
        
        # Ok so now we have the maximum amount of numtiles, we need to find the
        # closest power of two to this number, as the number of tiles spanning
        # the equator is equal to 2 ** z
        power = self.power_of_two(max_numtiles)
        # This power is basically the zoom level we need. But, to account for big erros
        # for higher lattitudes, subtract 1 from the zoom level.
        self.setZ(power-1)
        return
        
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
        return
        
    def addGeofenceAndTiles(self, Geofence, tiles):
        '''Adds a geofence and assumes the tiles are already known, thus avoiding the calculation of tiles.'''
        name = Geofence.name
        # Tiles need to be a list of tuples, e.g., [(1,1), (1,2)]
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
        return
        
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
        self.tiledictionary['ZOOM'] = self.z
        self.geodictionary['ZOOM'] = self.z
    
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
            
            if xtiledir != 0 and ytiledir != 0:
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
                
            elif ytiledir == 0:
                # We're going in the x direction only
                nextTileX, nextTileY = currentTileX + xtiledir, currentTileY
                
            elif xtiledir == 0:
                # We're moving in the y direction only
                nextTileX, nextTileY = currentTileX, currentTileY + ytiledir
                
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
        if difflat != 0 and difflon != 0:
            return difflat/abs(difflat), difflon/abs(difflon)
        
        if difflat == 0:
            return 0, difflon/abs(difflon)
        
        if difflon == 0:
            return difflat/abs(difflat), 0
    
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
    
    def power_of_two(self, target):
        # Returns the power of two that is that closest, but smaller than 
        # target. 
        if target > 1:
            i = 0
            while True:
                if (2 ** i >= target):
                    return (i - 1)
                i += 1
        else:
            return 1
    
# ----------------------------- End of Helper Classes -----------------------------

# ----------------------------- Plugin Functions ----------------------------------

# A dictionary of geofences {name : Geofence object}
geofences = dict()
# A container that links tile data and geofence data
TileData = GeofenceTileData()
# geofence rtree
geoidx = index.Index()
# Dict to keep track of IDs, format: ID : {Geofence Name}
geoidx_idtogeo = dict()
geoidx_geotoid = dict()
# First ID. 
geoidx_id = 0


def detectmethod(name):
    if name in ['TILE', 'tile']:
        bs.traf.geod.method = 'TILE'
    elif name in ['RTREE', 'rtree']:
        bs.traf.geod.method = 'RTREE'
    else:
        bs.scr.echo('Available geofence detection methods: TILE, RTREE.')
    return

def reset():
    ''' Resets everything geofence related. '''
    for geofencename in geofences:
        bs.scr.objappend('', geofencename, None)
    geofences.clear()
    TileData.clear()
    bs.traf.geod.reset()
    geoidx_idtogeo.clear()
    geoidx_geotoid.clear()
    
    global geoidx, geoidx_id
    geoidx = index.Index()
    geoidx_id = 0
    return

def setZ(z):
    ''' Change the tile zoom level for the geofence dictionary. 
    If this is done, we need to reset the geofenceTileData dictionaries
    and add all geofences one by one again. This is a lengty process
    if there are a lot of geofences and/or the zoom level is high.
    The zoom level is capped at 18 for now.'''
    # Cap the zoom level between 1 and 18
    if z>18:
        z = 18
    if z<1:
        z = 1
        
    # Set the new zoom level
    TileData.setZ(z)
    # Clear dictionaries
    TileData.clear()
    # Add geofences from dictionary one by one
    for geofence in geofences.values():
        TileData.addGeofence(geofence)

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
    TileData.addGeofence(myGeofence)
    
    global geoidx_id
    # Add geofence bounds to rtree
    geoidx.insert(geoidx_id, myGeofence.bbox)
    
    # Register this in geoid dict
    geoidx_idtogeo[geoidx_id] = geofencename
    geoidx_geotoid[geofencename] = geoidx_id
    
    # Increment ID
    geoidx_id += 1
    
    return True, f'Created geofence {args[0]}.'

# Delete geofence
def delgeofence(name=""):
    geofencename = 'GF:'+name
  
    try:
        myGeofence = geofences[geofencename]
    except: 
        return False, f'No geofence found for {name}'
        
    # Delete geofence if name exists
    found = geofences.pop(geofencename)
    if not found:
        return False, f'No geofence found for {name}'
    
    # Remove geofence from screen
    bs.scr.objappend('', geofencename, None)
    
    # Remove geofence from all dictionaries
    TileData.removeGeofence(geofencename)
    
    # Remove geofence from geodetect
    bs.traf.geod.delgeofence(geofencename)
    
    # Remove geofence from rtree
    geoidx.delete(geoidx_geotoid[geofencename], myGeofence.bbox)
    
    return True, f'Deleted geofence {name}'

def retrievegeofences():
    return geofences

def bounds(coordinates):
    ''' Returns tuple (minlat, minlon, maxlat, maxlon)'''
    # Coordinates are latlon, extract each
    lats = coordinates[::2]
    lons = coordinates[1::2]
    return (min(lats), min(lons), max(lats), max(lons))

def loadFromFile(*args):
    # Clear previous geofences
    reset()
    # Loads geofences and their tile data from .pkl file
    filename = args[0]
    graphicsYN = args[1]
    
    if graphicsYN in ['Y', 'y']:
        loadGraphicsBool = True
    elif graphicsYN in ['N', 'n']:
        loadGraphicsBool = False
    else:
        return 'Could not parse command. Use Y or N. '
    
    if filename[-4:] != '.pkl':
        filename = filename + '.pkl'
        
    # Check if file exists
    if not os.path.exists('data/geofence/' + filename):
        bs.scr.echo('File does not exist.')
        return
    
    bs.scr.echo('Loading pickle file...')
    
    # We need to import the 3 dictionaries, and send geofences to screen if requested
    with open('data/geofence/' + filename, 'rb') as f:
        GlobalDict = pickle.load(f)  
    
    # Take the global instance of variables
    global geofences
    global TileData
    global geoidx_id
    
    # Process the geofences
    geodict = GlobalDict['geodict']
    for geofencename in geodict:
        data = geodict[geofencename]
        myGeofence = Geofence(geofencename, data['coordinates'], data['topalt'], data['bottomalt'])
        geofences[geofencename] = myGeofence
        geoidx.insert(geoidx_id, myGeofence.bbox)
    
        # Register this in geoid dicts
        geoidx_idtogeo[geoidx_id] = geofencename
        geoidx_geotoid[geofencename] = geoidx_id
        
        # Increment
        geoidx_id += 1
           
    # Set them as such
    TileData.tiledictionary = GlobalDict['tiledictionary']
    TileData.geodictionary = GlobalDict['geodictionary']
    
    # Set the zoom level to match the one in TileData
    #try:
    TileData.setZ(GlobalDict['tiledictionary']['ZOOM'])
    #except:
        #bs.scr.echo('File is missing zoom level data. Load failed.')
        #reset()
        #return
    
    
    if loadGraphicsBool == True:
        bs.scr.echo('Loading graphics...')
        # Also send the geofences to be drawn
        for geofence in geofences.values():
            geofencename = geofence.name
            coordinates = geofence.coordinates
            # Create shape on screen
            bs.scr.objappend('POLY', geofencename, coordinates)
            
            # Make geofence red
            data = dict(color=(255, 0, 0))
            data['polyid'] = geofencename
            bs.net.send_event(b'COLOR', data)
    
    bs.scr.echo('Load from file successful.')
    return

def saveToFile(filename):
    # Saves geofences and their tile data to json file
    # We need to store: Geofence name, coords, topalt, bottomalt, associated tiles
    # Process command
    if filename[-4:] != '.pkl':
        filename = filename + '.pkl'

    bs.scr.echo('Saving to pickle file...')
    
    # Prepare dictionary. We have an overall dictionary that will serve as a
    # collection of dictionaries. 
    GlobalDict = dict()
    
    # First store geofences dict. We create some temp dictionaries for this
    geodict = dict()
    for geofencename in geofences:
        geofence = geofences[geofencename]
        # secondary dict with data
        data = dict()
        # We basically need to extract data from the geofence objects
        data['coordinates'] = geofence.coordinates
        data['topalt'] = geofence.topalt
        data['bottomalt'] = geofence.bottomalt
        geodict[geofence.name] = data
        
    # Save the geodict
    GlobalDict['geodict'] = geodict
    
    # Then store the two dictionaries from GeofenceTileData
    GlobalDict['tiledictionary'] = TileData.tiledictionary
    GlobalDict['geodictionary'] = TileData.geodictionary
    
    # Check if geofence folder exists in data
    dirname = "data/geofence"
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    
    # Now save this GlobalDict to a .pkl file
    with open('data/geofence/' + filename, 'wb') as f:
        pickle.dump(GlobalDict, f, pickle.HIGHEST_PROTOCOL)
    
    bs.scr.echo('Save to file successful.')
    
    return 'Save to file successful.'