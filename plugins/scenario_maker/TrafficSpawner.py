import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools.geo import kwikqdrdist
from bluesky.tools.aero import kts, ft
from bluesky.traffic import Route
from bluesky.tools.misc import degto180
import numpy as np
import os
import pickle
import random

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICSPAWNER',
        'plugin_type': 'sim',
    }
    foo = TrafficSpawner()
    return config

class TrafficSpawner(Entity):
    def __init__(self):
        super().__init__()
        self.target_ntraf = 50
        # Load default city
        self.loadcity('Vienna')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        # When to stop simulating
        self.stop_time = 600
        # Start the logs
        bs.traf.conflog.start()
        bs.traf.loslog.start()
        # Turn ASAS on
        stack.stack('ASAS ON')
        return
    
    def reset(self):
        self.__init__()
    
    @command
    def loadcity(self, city = None):
        list_of_cities = [x for x in os.listdir(f'plugins/scenario_maker/') if '.py' not in x]
        if city == None or city not in list_of_cities:
            bs.scr.echo(f'The following cities are available: {list_of_cities}.')
            return
        self.city = city
        self.path = f'plugins/scenario_maker/{self.city}'
        self.load_origins_destinations()
        return
    
    @command
    def trafficnumber(self, target_ntraf = 50):
        self.target_ntraf = target_ntraf
        bs.scr.echo(f'The target traffic number was set to {target_ntraf}.')
        return
    
    @command
    def stopsimt(self, time):
        # This will be the time at which we stop and quit.
        self.stop_time = time
    
    def load_origins_destinations(self):
        with open(f'{self.path}/orig_dest_dict.pickle', 'rb') as f:
            self.orig_dest_dict = pickle.load(f)
        return
    
    @timed_function(dt = 1)
    def spawn_traffic(self):
        '''Function to spawn traffic to maintain a traffic level equal to ntraf.'''
        while bs.traf.ntraf < self.target_ntraf:
            # Choose a random origin and destination
            origin = random.choice(list(self.orig_dest_dict.keys()))
            destination = random.choice(self.orig_dest_dict[origin])
            
            # Load the pickle file for that
            with open(f'{self.path}/pickles/{origin}-{destination}.pkl', 'rb') as f:
                pickled_route = pickle.load(f)
                
            # This pickle route has LAT, LON, EDGE, TURN. Unpack em
            lats, lons, edges, turns = list(zip(*pickled_route))
            
            # Obtain required data for aircraft
            acid = f'D{self.traf_id}'
            self.traf_id += 1
            actype = 'M600'
            achdg, _ = kwikqdrdist(lats[0], lons[0], lats[1], lons[1])
            
            # Let's create the aircraft
            bs.traf.cre(acid, actype, lats[0], lons[0], achdg, 0, 5)
            
            # Get more info
            acrte = Route._routes.get(acid)
            acidx = bs.traf.id.index(acid)
            
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
    
    @timed_function(dt = 0.5)
    def delete_aircraft(self):
        # Delete aircraft that have LNAV off and have gone past the last waypoint.
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
                stack.stack(f'DEL {acid}')
                
        if bs.sim.simt > self.stop_time:
            stack.stack(f'HOLD')
            stack.stack(f'DELETEALL')
            stack.stack(f'QUIT')
            
    @command
    def deleteall(self):
        '''Deletes all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return