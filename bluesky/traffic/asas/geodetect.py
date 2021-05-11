import numpy as np

import bluesky as bs
import shapely
from bluesky.core import Entity
from bluesky.tools.aero import ft, nm
from bluesky.tools import geo

class GeofenceDetection(Entity):
    '''Geofence crossing detection.'''
    def __init__(self):
        super().__init__()
        self.geoconfs = dict() # Stores current conflicts
        self.geobreaches = dict() # Stores current breaches
        # Array to store if an aircraft is in conflict with a geofence
        with self.settrafarrays():
            self.active = np.array([]) 
        
        # Historic lists
        self.allgeobreaches = [] # Stores all pastgeofence breaches 
        self.allgeoconfs = [] # Stores all past geofence conflicts
        
    def reset(self):
        ''' Reset data lists. '''
        self.allgeobreaches = []
        self.allgeoconfs = []
        self.active = np.array([]) 
        
    def update(self, ownship):
        ''' This function is called from within traffic.'''
        # First check if geofencing plugin is active
        if 'geofence' not in bs.core.varexplorer.varlist:
            return 'Geofence plugin not loaded.'
        geoinvicinity = bs.traf.geos.geoinvicinity
        # Do the detection
        self.geoconfs = self.detect(ownship, geoinvicinity)
        
    def detect(self, ownship, geoinvicinity):
        geoconfs = dict()
        # Keep stuff as false by default
        self.active = np.array([False] * bs.traf.ntraf) 
        
        # Geoinvicinity is a dictionary taken from Geosense that basically shows the
        # geofences in vicinity for each aircraft, and is shaped like this:
        # ACID : {geofence objects}
        # So we can iterate over all these aircraft and check whether they are on a
        # conflict trajectory
        for acid in geoinvicinity:
            # First of all, get aircraft idx
            idx_ac = ownship.id.index(acid)
            
            # We're interested in the current velocity vector
            vel_ac = np.array([ownship.gseast[idx_ac], ownship.gsnorth[idx_ac], ownship.vs[idx_ac]])
            pos_ac = np.array([ownship.lat[idx_ac], ownship.lon[idx_ac], ownship.alt[idx_ac]])
            hdg_ac = ownship.trk[idx_ac]
            pos_ac_2d = pos_ac[:2]
            
            # Use dlookahead to find second point for trajectory segment
            # TODO: Might want to make this in function of time instead of a distance
            dlookahead = bs.settings.geofence_dlookahead
            pos_next = geo.qdrpos(pos_ac_2d[0], pos_ac_2d[1], hdg_ac, dlookahead / nm)
            print(pos_next)
            
            # Create line
            line = shapely.geometry.LineString(np.array([pos_ac[:2], pos_next]))
            
            # Then, iterate over geofence objects and check if there is an intersection
            for geofence in geoinvicinity[acid]:
                # First do horizontal check. We check using shapely if projected line intersects polygon.
                geopoly = geofence.getPoly()
                if line.intersects(geopoly):
                    # We have a conflict, add conflict to dictionary
                    if acid not in geoconfs:
                        geoconfs[acid] = set()
                    
                    # Add geofence to this set
                    geoconfs[acid].add(geofence)
                    self.active[idx_ac] = True 
        
        print(self.active)
        print(geoconfs)
        return geoconfs