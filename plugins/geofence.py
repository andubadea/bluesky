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
        }
    # init_plugin() should always return these two dicts.
    return config, stackfunctions

#------------------------------ Helper Classes ----------------------------------------   
class Geofence:
    ''' Geofence specification class. '''
    def __init__(self, name, coordinates, topalt=1e9, bottomalt=-1e9):
        self.name = name
        self.coordinates = coordinates
        lats = self.coordinates[::2]
        lons = self.coordinates[1::2]
        self.topalt = topalt
        self.bottomalt = bottomalt
        self.poly = shapely.geometry.Polygon(np.array([lats, lons]).T)
        self.bbox = self.poly.bounds

    def getPointArray(self):
        '''Returns the points not as point objects but as an array.'''
        lats = self.coordinates[::2]
        lons = self.coordinates[1::2]
        pointsarr = np.array([lats, lons])
        return pointsarr.T
    
    def getPoly(self):
        return self.poly


# ----------------------------- Plugin Functions ----------------------------------

# A dictionary of geofences {name : Geofence object}
geofences = dict()
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
    bs.traf.geod.reset()
    geoidx_idtogeo.clear()
    geoidx_geotoid.clear()
    
    global geoidx, geoidx_id
    geoidx = index.Index()
    geoidx_id = 0
    return

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
    
    global geoidx_id
    # Add geofence bounds to rtree
    geoidx.insert(geoidx_id, myGeofence.bbox)
    print(myGeofence.bbox)
    
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
    
    # Check if geofence folder exists in data
    dirname = "data/geofence"
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    
    # Now save this GlobalDict to a .pkl file
    with open('data/geofence/' + filename, 'wb') as f:
        pickle.dump(GlobalDict, f, pickle.HIGHEST_PROTOCOL)
    
    bs.scr.echo('Save to file successful.')
    
    return 'Save to file successful.'