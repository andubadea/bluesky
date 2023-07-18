import bluesky as bs
from bluesky import stack
from bluesky.core import Entity, timed_function
from bluesky.tools.misc import degto180
from bluesky.core.simtime import timed_function
import numpy as np


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICHANDLER',
        'plugin_type': 'sim'
    }
    # Put TrafficSpawner in bs.traf
    bs.traf.TrafficHandler = TrafficHandler()
    return config

class TrafficHandler(Entity):
    def __init__(self):
        super().__init__()
        # When to stop simulating
        self.stop_time = 2*60*60 #seconds
        self.stop_time_enable = True
        
        with self.settrafarrays():
            self.allocated_alt = []
            
    def create(self, n=1):
        super().create(n)
        # Save the starting altitude
        self.allocated_alt[-n:] = bs.traf.alt[-n:]

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
                
        if (self.stop_time_enable and bs.sim.simt > self.stop_time):
            stack.stack(f'HOLD')
            stack.stack(f'DELETEALL')
            stack.stack(f'RESET')
            
    @stack.command
    def deleteall(self):
        '''Deletes all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return
    
    @timed_function(name='cruisespd', dt = 0.5)
    def speed_control(self):
        '''Set the cruise speed of all aircraft.'''
        # First, some checks
        in_turn = np.logical_or(bs.traf.ap.inturn, bs.traf.ap.dist2turn < 50)  # Are aircraft in a turn?
        cr_active = bs.traf.cd.inconf # Are aircraft doing CR?
        in_vert_man = np.abs(bs.traf.vs) > 0 # Are aircraft performing a vertical maneuver?
        speed_zero = np.array(bs.traf.selspd) == 0 # The selected speed is 0, so we're at our destination and landing
        lnav_on = bs.traf.swlnav
        
        # Set the speed of all aircraft that meed the conditions to 30
        set_cruise_speed = np.logical_and.reduce((lnav_on,
                                                  np.logical_not(in_turn),
                                                  np.logical_not(cr_active),
                                                  np.logical_not(in_vert_man),
                                                  np.logical_not(speed_zero)))
        
        bs.traf.selspd = np.where(set_cruise_speed, bs.traf.cr.cruise_spd, bs.traf.selspd)