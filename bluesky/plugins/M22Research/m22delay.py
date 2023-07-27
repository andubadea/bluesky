import bluesky as bs
import numpy as np
import random

from bluesky.tools.geo import kwikdist_matrix
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.traffic import Route
from bluesky.tools.aero import nm, kts, ft

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22DELAY',
        'plugin_type': 'sim',
        'reset': reset
    }
    bs.traf.m22delay = M22Delay()
    return config

def reset():
    bs.traf.m22delay.reset()

class M22Delay(Entity):
    def __init__(self):
        super().__init__()
        self.mean = 0 # Mean delay
        self.delay_probability = 0 # probability of delay
        
        self.aircraft_buffer = dict()
        
        with self.settrafarrays():
            self.cre_time = []
        
    def reset(self):
        self.mean = 0
        self.delay_probability = 0
        self.aircraft_buffer = dict()
        
    def create(self, n=1):
        super().create(n)
        self.cre_time[-n:] = [bs.sim.simt]*n
        
    @stack.command
    def M22cre(self, acid:'txt', actype:'txt', aclat:'lat', aclon:'lon', achdg:'hdg', acalt:'alt', acspd:'spd', *wpt_data):
        """The function to attempt the creation of an aircraft. Delay can be introduced here.
        wpt_data is in the following repeating sequence:
        lat, lon, alt, spd,rta, FLYTURN/FLYBY/FLYOVER, street_number
        """
        if len(wpt_data)%7 !=0:
            bs.scr.echo('You missed a waypoint value, arguement number must be a multiple of 7.')
            return
        # Reshape the wp args
        wpt_data = np.reshape(wpt_data, (int(len(wpt_data)/7), 7))
        # First of all, do a delay roll, and see if we delay this aircraft
        delay = 0
        roll = random.random()
        if roll < self.delay_probability:
            delay = self.get_delay()
            
        if delay > 0:
            # Add aircraft to buffer
            self.aircraft_buffer[acid] = [bs.sim.simt, delay, acid, actype, aclat, aclon, achdg, acalt, acspd, wpt_data]
            return
        
        # We want to spawn the aircraft, but we gotta check whether that is possible
        can_spawn_aircraft = self.proximity_check(aclat, aclon, acalt)

        if can_spawn_aircraft:
            # Spawn it then
            bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)
            # And now get its idx
            acidx = bs.traf.id.index(acid)
            ## Add route
            # Set the default cruise speed, turn speed, and rate
            bs.traf.ap.cruisespd[acidx] = acspd
            bs.traf.ap.route[acidx].addwptMode(acidx, 'TURNBANK', 25*ft)
            bs.traf.ap.route[acidx].addwptMode(acidx, 'TURNRAD', 0.00216*ft)
            # Extract the street number and RTA info
            bs.traf.TrafficHandler.street_numbers[acidx] = wpt_data[:,6]
            rta_info = wpt_data[:,4]
            # Get rid of the street info, we will keep that in traffic handler
            wpt_data_stripped = np.delete(wpt_data[:,0:6], 4,1).flatten()
            # Now add the waypoints for this aircraft
            Route.addwaypoints(acidx, *wpt_data_stripped)
            # Add the RTA to the route
            for wpidx, rta_point in enumerate(rta_info):
                if rta_point:
                    acrte = Route._routes.get(acid)
                    acrte.wprta[wpidx] = float(rta_point)
            # Some more commands to get it going
            bs.traf.ap.setLNAV(acidx, True)
            bs.traf.ap.setVNAV(acidx, True)
            return
        else:
            # Add it to buffer
            self.aircraft_buffer[acid] = [bs.sim.simt, delay, acid, actype, aclat, aclon, achdg, acalt, acspd, wpt_data]
            return
        
    @timed_function(name='spawncheck', dt = 1)
    def attempt_create(self):
        '''Attempts to create aircraft that were not created when the M22CRE command was
        issued, either because they have a delay, or because they couldn't.'''
        # Create a copy of the dict
        temp = self.aircraft_buffer.copy()
        for acid in temp:
            # Check if the time requirement is fulfilled
            create_time = temp[acid][0]
            delay = temp[acid][1]
            now = bs.sim.simt
            if now - create_time < delay:
                # Skip this aircraft, not delayed enough yet
                continue
            
            # Okay so it is delayed enough, now to check the spacial requirement
            can_spawn_aircraft = self.proximity_check(temp[acid][4], temp[acid][5], temp[acid][7])
            if not can_spawn_aircraft:
                # Again, continue, we'll try again next step when it is clear to spawn
                continue
            # Otherwise, spawn the aircraft
            _, _, _, actype, aclat, aclon, achdg, acalt, acspd, wpt_data = temp[acid]
            # Spawn it then
            bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)
            # And now get its idx and route
            acidx = bs.traf.id.index(acid)
            ## Add route
            # Set the default cruise speed, turn speed, and rate
            bs.traf.ap.cruisespd[acidx] = acspd
            bs.traf.ap.route[acidx].addwptMode(acidx, 'TURNBANK', 25*ft)
            bs.traf.ap.route[acidx].addwptMode(acidx, 'TURNRAD', 0.00216*ft)
            # Extract the street number and RTA info
            bs.traf.TrafficHandler.street_numbers[acidx] = wpt_data[:,6]
            rta_info = wpt_data[:,4]
            # Get rid of the street info, we will keep that in traffic handler
            wpt_data_stripped = np.delete(wpt_data[:,0:6], 4,1).flatten()
            # Now add the waypoints for this aircraft
            Route.addwaypoints(acidx, *wpt_data_stripped)
            # Add the RTA to the route
            for wpidx, rta_point in enumerate(rta_info):
                if rta_point:
                    acrte = Route._routes.get(acid)
                    acrte.wprta[wpidx] = float(rta_point)
            # Some more commands to get it going
            bs.traf.ap.setLNAV(acidx, True)
            bs.traf.ap.setVNAV(acidx, True)
            # Remove the aircraft from the dictionary
            self.aircraft_buffer.pop(acid)
            
        
    def proximity_check(self, aclat, aclon, acalt):
        '''Checks whether an aircraft is safe to spawn at the specified location.
        Returns true if we can spawn, false if we cannot.'''
        layer_diff = bs.traf.TrafficHandler.cruiselayerdiff
        # First of all, get all the aircraft that are within the altitude tolerance.
        ac_close_alt = np.logical_and(acalt - layer_diff < bs.traf.alt, 
                                    bs.traf.alt < acalt + layer_diff)
        
        # Out of the aircraft that are within the altitude layer
        lats = bs.traf.lat[ac_close_alt]
        lons = bs.traf.lon[ac_close_alt]
        
        dists = kwikdist_matrix(np.array([aclat]), np.array([aclon]), lats, lons) * nm
        
        dist_not_ok = np.any(dists < bs.traf.cd.rpz_def * 2)
        
        if dist_not_ok:
            return False
        else:
            return True
    
    def get_delay(self):
        """Return a random delay for this aircraft. Distribution is exponential
        """
        return np.random.default_rng().exponential(self.mean)
        
    @stack.command
    def setm22delay(self, mean:float=0, probability:float=0):
        """Set the average and standard deviation of the delay.
        """
        self.mean = mean
        self.delay_probability = probability