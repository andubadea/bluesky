''' A collection of functions that one can use to
dynamically change the route of an aircraft while
the simulation is running. '''

import bluesky as bs
import numpy as np
from bluesky.tools.aero import ft, kts, nm

def changeWptCons(acid, wpt, alt = None, spd = None):
    '''Changes the altitude and speed constraints of aircraft "acid" at waypoint "wpt".
    If only altitude or speed is wanted, set alt or spd to None.
    acid - either aircraft name or aircraft idx
    wpt - either waypoint name or waypoint idx in route
    alt - altitude in meters 
    spd = speed in m/s'''

    # Check if acid is an index or a string, get index
    if isinstance(acid, str):
        #ACID is a string, determine ACID index
        try:
            idx_ac = bs.traf.id2idx(acid)
        except:
            return f'Cannot parse ACID.'
        id_ac = acid
    elif isinstance(acid, int):
        #ACID is an int, just take it as such
        idx_ac = acid
        try:
            id_ac = bs.traf.id[idx_ac]
        except:
            return f'AC index out of range.'
    else:
        # Just take whatever came through and try making it an int
        try:
            idx_ac = int(acid)
        except:
            print('Cannot parse ACID.')
            
    # Create a copy of the aircraft route
    # Maybe it's best to work with the original route where needed to ensure the changes are applied
    route = bs.traf.ap.route[idx_ac]
            
    if isinstance(wpt, str):
        #WPT is a string, determine WPT index
        try:
            idx_wpt = route.wpname.index(wpt)
        except:
            return f'Waypoint not in route of {id_ac}.'
        name_wpt = wpt
    elif isinstance(acid, int):
        #WPT is an int, just take it as such
        idx_wpt = wpt
        try:
            name_wpt = route.wpname[idx_wpt]
        except:
            return f'Waypoint index is out of range.'
    else:
        # Just take whatever came through and try making it an int
        try:
            idx_wpt = int(wpt)
            name_wpt = route.wpname[idx_wpt]
        except:
            return 'Cannot parse waypoint.'
    
    idx_active_wpt = int(bs.traf.actwp.idx[idx_ac])
    
    # If IDX_active_wpt is negative, there is no active route for this aircraft
    if idx_active_wpt < 0:
        return 'No active route'
    
    # Get the previous and after waypoints in the route, if possible
    # First check if there is only one waypoint in the route
    if len(route.wpname) == 1: 
        # Only one wpt in list, so we can remove it and just add another waypoint without bothering with order
        prev_wpt = None
        after_wpt = None
        
    elif len(route.wpname) == idx_wpt + 1: 
        # If True, wpt is last in list
        prev_wpt = route.wpname[idx_wpt - 1]
        after_wpt = None
        
    elif idx_wpt == 0: 
        # If True, wpt is first in list
        prev_wpt = None
        after_wpt = route.wpname[idx_wpt + 1]
        
    else:
        # Waypoint is somewhere in the list body
        prev_wpt = route.wpname[idx_wpt - 1]
        after_wpt = route.wpname[idx_wpt + 1]
            
    # We need to do the following steps in order to change the altitude at a
    # waypoint:
    # 1. Delete the waypoint from aircraft route
    # 2. Add the waypoint again to the route, with the given altitude constraint
    # 3. If this waypoint was the active waypoint, issue a direct command to the aircraft
    #    towards it.
    
    # Delete the waypoint
    bs.traf.ap.route[idx_ac].delwpt(name_wpt)
    
    print(prev_wpt, after_wpt, name_wpt, alt)
    
    # Add new waypoint to route
    bs.traf.ap.route[idx_ac].addwptStack(idx_ac, name_wpt, alt, spd, prev_wpt, after_wpt)
    
    # Check if the previously active waypoint is the same as the one we modified.
    if name_wpt == route.wpname[idx_active_wpt]:
        # We need to also give a direct command
        bs.traf.ap.route[idx_ac].direct(idx_ac, name_wpt)
        
    return "Success." 

def changeWptAlt(acid, wpt, alt):
    # Run changeWptCons but with spd = None
    changeWptCons(acid, wpt, alt, None)
    
def changeWptSpd(acid, wpt, spd):
    # Run changeWptCons but with alt = None
    changeWptCons(acid, wpt, None, spd)

def changeAlt(acid, alt):
    '''Gives the aircraft an altitude command, and removes the 
    altitude constraint from the active waypoint. VNAV is thus resumed
    normally after active waypoint is passed.'''
    # Check if acid is an index or a string, get index
    if isinstance(acid, str):
        #ACID is a string, determine ACID index
        try:
            idx_ac = bs.traf.id2idx(acid)
        except:
            return f'Cannot parse ACID.'
        id_ac = acid
    elif isinstance(acid, int):
        #ACID is an int, just take it as such
        idx_ac = acid
        try:
            id_ac = bs.traf.id[idx_ac]
        except:
            return f'AC index out of range.'
    else:
        # Just take whatever came through and try making it an int
        try:
            idx_ac = int(acid)
        except:
            print('Cannot parse ACID.')
            

    
    
    