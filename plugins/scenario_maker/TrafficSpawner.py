import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools.geo import kwikqdrdist
from bluesky.tools.aero import kts, ft
from bluesky.traffic import Route
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
        self.target_ntraf = 1
        # Load default city
        self.loadcity('Vienna')
        # Traffic ID increment
        self.traf_id = 1
        #default alt and speed
        self.alt = 100 * ft
        self.spd = 20 * kts
        return
    
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
    def trafficnumber(self, target_ntraf = 100):
        self.target_ntraf = target_ntraf
        bs.scr.echo(f'The target traffic number was set to {target_ntraf}.')
        return
    
    def load_origins_destinations(self):
        with open(f'{self.path}/orig_dest_dict.pickle', 'rb') as f:
            self.orig_dest_dict = pickle.load(f)
        return
    
    @timed_function(dt = 10)
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
            
            print(turns)
            
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
            
    @command
    def DELETEALL(self):
        '''Deletes all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return