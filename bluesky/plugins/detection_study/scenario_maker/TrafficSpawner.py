import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools.geo import kwikqdrdist, kwikdist_matrix, qdrpos, kwikdist
from bluesky.tools.aero import kts, ft, fpm, nm
from bluesky.traffic import Route
from bluesky.tools.misc import degto180
import numpy as np
import geopandas as gpd
import osmnx as ox
import os
import pickle
import random

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICSPAWNER',
        'plugin_type': 'sim',
        'reset': reset
    }
    # Put TrafficSpawner in bs.traf
    bs.traf.TrafficSpawner = TrafficSpawner()
    return config

def reset():
    bs.traf.TrafficSpawner.reset()

class TrafficSpawner(Entity):
    def __init__(self):
        super().__init__()
        self.target_ntraf = 200
        # Load default city
        self.graph, self.edges, self.nodes = self.loadcity('Vienna')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        # When to stop simulating
        self.stop_time = 7*24*60*60
        self.stop_time_enable = True
        # Number of conflicts when to stop simulating
        self.stop_conf = 10000
        self.stop_conf_enable = False
        # Turn ASAS on
        stack.stack('ASAS ON')
        # Set a default seed
        stack.stack('SEED 12345')
        
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()
        
        with self.settrafarrays():
            self.route_edges = []
            # Metrics
            self.distance2D = np.array([])
            self.distance3D = np.array([])
            self.distancealt = np.array([])
            self.create_time = np.array([])
        return
    
    def create(self, n=1):
        super().create(n)
        # Store creation time of new aircraft
        self.route_edges[-n:] = [0]*n # Default edge
        self.distance2D[-n:] = [0]*n
        self.distance3D[-n:] = [0]*n
        self.distancealt[-n:] = [0]*n
        self.create_time[-n:] = [0]*n
    
    def reset(self):
        self.target_ntraf = 50
        # Load default city
        self.graph, self.edges, self.nodes = self.loadcity('Vienna')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        # When to stop simulating
        self.stop_time = 7*24*60*60
        self.stop_time_enable = True
        # Number of conflicts when to stop simulating
        self.stop_conf = 10000
        self.stop_conf_enable = False
        # Turn ASAS on
        stack.stack('ASAS ON')
        # Set a default seed
        stack.stack('SEED 12345')
        
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()
    
    @command
    def loadcity(self, city = None):
        # Get a list of available cities
        list_of_cities = [x for x in os.listdir(f'bluesky/plugins/detection_study/scenario_maker/') if '.py' not in x]
        if city == None or city not in list_of_cities:
            bs.scr.echo(f'The following cities are available: {list_of_cities}.')
            return
        self.city = city
        self.path = f'bluesky/plugins/detection_study/scenario_maker/{self.city}'
        self.load_origins_destinations()
        # Set the origin point of the city
        with open(f'{self.path}/centre.txt', 'r') as f:
            coords = f.readlines()
        self.city_centre_coords = [float(coords[0]), float(coords[1])]
        # Load the graph for the city
        # read gpkgs that are
        nodes = gpd.read_file(f'{self.path}/streets.gpkg', layer='nodes')
        edges = gpd.read_file(f'{self.path}/streets.gpkg', layer='edges')

        # set the indices 
        edges.set_index(['u', 'v', 'key'], inplace=True)
        nodes.set_index(['osmid'], inplace=True)

        # ensure that it has the correct value
        nodes['x'] = nodes['geometry'].apply(lambda x: x.x)
        nodes['y'] = nodes['geometry'].apply(lambda x: x.y)

        G = ox.graph_from_gdfs(nodes, edges)
        
        bs.stack.stack(f'SCHEDULE 00:00:00 PAN {self.city_centre_coords[0]},{self.city_centre_coords[1]}')
        bs.stack.stack(f'SCHEDULE 00:00:00 ZOOM 15')
        # bs.stack.stack(f'SCHEDULE 00:00:01 CDMETHOD INTENTCD')
        # bs.stack.stack(f'SCHEDULE 00:00:01 RESO INTENTCR')
        bs.stack.stack(f'SCHEDULE 00:00:00 CDMETHOD DEFENSIVECD')
        bs.stack.stack(f'SCHEDULE 00:00:00 RESO DEFENSIVECR')
        bs.stack.stack(f'SCHEDULE 00:00:00 STARTLOGS')
        bs.stack.stack(f'SCHEDULE 00:00:00 STARTCDRLOGS')
        bs.stack.stack(f'HOLD')
        return G, edges, nodes
    
    @command
    def trafficnumber(self, target_ntraf = 50):
        self.target_ntraf = int(target_ntraf)
        bs.scr.echo(f'The target traffic number was set to {target_ntraf}.')
        return
    
    @command
    def stopsimt(self, time):
        # This will be the time at which we stop and quit.
        self.stop_time = int(time)
        self.stop_time_enable = True
        self.stop_conf_enable = False
        
    @command
    def stopconf(self, confno):
        # This will be the number of conflicts at which we stop and quit.
        self.stop_conf = int(confno)
        self.stop_conf_enable = True
        self.stop_time_enable = False
    
    def load_origins_destinations(self):
        with open(f'{self.path}/orig_dest_dict.pickle', 'rb') as f:
            self.orig_dest_dict = pickle.load(f)
        return
    
    @timed_function(dt = 1)
    def spawn_traffic(self):
        '''Function to spawn traffic to maintain a traffic level equal to ntraf.'''
        attempts = 0
        while bs.traf.ntraf < self.target_ntraf and attempts < 20:
            # Choose a random origin and destination
            origin = random.choice(list(self.orig_dest_dict.keys()))
            destination = random.choice(self.orig_dest_dict[origin])
            
            # Load the pickle file for that
            with open(f'{self.path}/pickles/{origin}-{destination}.pkl', 'rb') as f:
                pickled_route = pickle.load(f)
                
            # This pickle route has LAT, LON, EDGE, TURN. Unpack em
            lats, lons, edges, turns = list(zip(*pickled_route))
            
            # Check if any other aircraft is too close to the origin
            dist = kwikdist_matrix(np.array([lats[0]]), np.array([lons[0]]), bs.traf.lat, bs.traf.lon)
    
            # Second check, if distance is smaller than rpz * 4?
            if np.any(dist<(bs.settings.asas_pzr*2)):
                # Try again with another random aircraft
                attempts += 1
                continue
            
            # We are successful
            attempts = 0
            
            # Obtain required data for aircraft
            acid = f'D{self.traf_id}'
            self.traf_id += 1
            actype = 'M600'
            achdg, _ = kwikqdrdist(lats[0], lons[0], lats[1], lons[1])
            
            # Let's create the aircraft
            bs.traf.cre(acid, actype, lats[0], lons[0], achdg, self.alt, 5)
            
            # Get more info
            acrte = Route._routes.get(acid)
            acidx = bs.traf.id.index(acid)
            
            # Add the edges to this guy
            self.route_edges[acidx] = edges
            
            # Start adding waypoints
            for lat, lon, turn in zip(lats, lons, turns):
                if turn:
                    acrte.turnspd = 5 * kts
                    acrte.swflyby = False
                    acrte.swflyturn = True
                else:
                    acrte.swflyby = True
                    acrte.swflyturn = False
                    
                wptype  = Route.wplatlon
                acrte.addwpt_simple(acidx, acid, wptype, lat, lon, self.alt, self.spd)
            
            # Calculate the flight plan
            acrte.calcfp()
            # Turn lnav on for this aircraft
            stack.stack(f'LNAV {acid} ON')
            stack.stack(f'VNAV {acid} ON')
            # save the create time
            self.create_time[acidx] = bs.sim.simt
    
    @timed_function(dt = 0.5)
    def delete_aircraft(self):
        # Delete aircraft that have LNAV off and have gone past the last waypoint.
        # Also added logging in here because why not.
        lnav_on = bs.traf.swlnav
        still_going_to_dest = np.logical_and(abs(degto180(bs.traf.trk - bs.traf.ap.qdr2wp)) < 10.0, 
                                       bs.traf.ap.dist2wp > 5)
        delete_array = np.logical_and.reduce((np.logical_not(lnav_on), 
                                         bs.traf.actwp.swlastwp,
                                         np.logical_not(still_going_to_dest)))
        
        if np.any(delete_array):
            # Get the ACIDs of the aircraft to delete
            acids_to_delete = np.array(bs.traf.id)[delete_array]
            for acid in acids_to_delete:
                # Log the stuff for this aircraft in the flstlog
                idx = bs.traf.id.index(acid)
                bs.traf.CDRLogger.flst.log(
                    acid,
                    self.create_time[idx],
                    bs.sim.simt - self.create_time[idx],
                    (self.distance2D[idx]),
                    (self.distance3D[idx]),
                    (self.distancealt[idx]),
                    bs.traf.lat[idx],
                    bs.traf.lon[idx],
                    bs.traf.alt[idx]/ft,
                    bs.traf.tas[idx]/kts,
                    bs.traf.vs[idx]/fpm,
                    bs.traf.hdg[idx],
                    bs.traf.cr.active[idx],
                    bs.traf.aporasas.alt[idx]/ft,
                    bs.traf.aporasas.tas[idx]/kts,
                    bs.traf.aporasas.vs[idx]/fpm,
                    bs.traf.aporasas.hdg[idx])
                stack.stack(f'DEL {acid}')
                
        if (self.stop_time_enable and bs.sim.simt > self.stop_time) or \
            (self.stop_conf_enable and len(bs.traf.cd.confpairs_all) > self.stop_conf):
            stack.stack(f'HOLD')
            stack.stack(f'DELETEALL')
            stack.stack(f'RESET')
            
        # Increment the distance metrics
        resultantspd = np.sqrt(bs.traf.gs * bs.traf.gs + bs.traf.vs * bs.traf.vs)
        self.distance2D += bs.sim.simdt * abs(bs.traf.gs)
        self.distance3D += bs.sim.simdt * resultantspd
        self.distancealt += bs.sim.simdt * abs(bs.traf.vs)
        
        # Now let's do the CONF and LOS logs
        confpairs_new = list(set(bs.traf.cd.confpairs) - self.prevconfpairs)
        if confpairs_new:
            done_pairs = []
            for pair in set(confpairs_new):
                # Check if the aircraft still exist
                if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                    # Get the two aircraft
                    idx1 = bs.traf.id.index(pair[0])
                    idx2 = bs.traf.id.index(pair[1])
                    done_pairs.append((idx1,idx2))
                    if (idx2,idx1) in done_pairs:
                        continue
                        
                    bs.traf.CDRLogger.conflog.log(pair[0], pair[1],
                                    bs.traf.lat[idx1], bs.traf.lon[idx1],bs.traf.alt[idx1],
                                    bs.traf.lat[idx2], bs.traf.lon[idx2],bs.traf.alt[idx2])
                
        self.prevconfpairs = set(bs.traf.cd.confpairs)
        
        # Losses of separation as well
        # We want to track the LOS, and log the minimum distance and altitude between these two aircraft.
        # This gives us the lospairs that were here previously but aren't anymore
        lospairs_out = list(self.prevlospairs - set(bs.traf.cd.lospairs))
        
        # Attempt to calculate current distance for all current lospairs, and store it in the dictionary
        # if entry doesn't exist yet or if calculated distance is smaller.
        for pair in bs.traf.cd.lospairs:
            # Check if the aircraft still exist
            if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                idx1 = bs.traf.id.index(pair[0])
                idx2 = bs.traf.id.index(pair[1])
                # Calculate current distance between them [m]
                losdistance = kwikdist(bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.lat[idx2], bs.traf.lon[idx2])*nm
                # To avoid repeats, the dictionary entry is DxDy, where x<y. So D32 and D564 would be D32D564
                dictkey = pair[0]+pair[1] if int(pair[0][1:]) < int(pair[1][1:]) else pair[1]+pair[0]
                if dictkey not in self.losmindist:
                    # Set the entry
                    self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, bs.sim.simt]
                    # This guy here                             ^ is the LOS start time
                else:
                    # Entry exists, check if calculated is smaller
                    if self.losmindist[dictkey][0] > losdistance:
                        # It's smaller. Make sure to keep the LOS start time
                        self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, self.losmindist[dictkey][8]]
        
        # Log data if there are aircraft that are no longer in LOS
        if lospairs_out:
            done_pairs = []
            for pair in set(lospairs_out):
                # Get their dictkey
                dictkey = pair[0]+pair[1] if int(pair[0][1:]) < int(pair[1][1:]) else pair[1]+pair[0]
                # Is this pair in the dictionary?
                if dictkey not in self.losmindist:
                    # Pair was already logged, continue
                    continue
                losdata = self.losmindist[dictkey]
                # Remove this aircraft pair from losmindist
                self.losmindist.pop(dictkey)
                #Log the LOS
                bs.traf.CDRLogger.loslog.log(losdata[8], losdata[7], pair[0], pair[1],
                                losdata[1], losdata[2],losdata[3],
                                losdata[4], losdata[5],losdata[6],
                                losdata[0])
                
        
        self.prevlospairs = set(bs.traf.cd.lospairs)
        
    @command
    def deleteall(self):
        '''Deletes all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return