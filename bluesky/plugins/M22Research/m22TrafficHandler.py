import bluesky as bs
from bluesky import stack
from bluesky.core import Entity, timed_function
from bluesky.tools.misc import degto180
import numpy as np


def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICHANDLER',
        'plugin_type': 'sim'
    }
    # Put TrafficSpawner in bs.traf
    bs.traf.TrafficSpawner = TrafficHandler()
    return config

class TrafficHandler(Entity):
    def __init__(self):
        super().__init__()
        # When to stop simulating
        self.stop_time = 2*60*60 #seconds
        self.stop_time_enable = True

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