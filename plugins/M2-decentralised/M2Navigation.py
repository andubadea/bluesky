from matplotlib import offsetbox
from bluesky.traffic.turbulence import Turbulence
from bluesky.core.simtime import timed_function
import bluesky as bs
import numpy as np
from bluesky import core
from bluesky import stack
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.ops import cascaded_union, nearest_points
from shapely.affinity import translate
from bluesky.tools.geo import kwikdist, kwikqdrdist, latlondist, qdrdist
from bluesky.tools.aero import nm, ft, kts
from bluesky.tools.misc import degto180
import time

def init_plugin():

    # Addtional initilisation code
    nav = M2Navigation()
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'M2NAVIGATION',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

class M2Navigation(core.Entity):
    def __init__(self):
        super().__init__()  
        
    @timed_function(name='navtimedfunction', dt=0.5)
    def navtimedfunction(self):
        if bs.traf.ntraf == 0:
            return
            
        # Gather some bools
        in_turn = np.logical_or(bs.traf.ap.inturn, bs.traf.ap.dist2turn < 75)  # Are aircraft in a turn?
        cr_active = bs.traf.cd.inconf # Are aircraft doing CR?
        in_vert_man = np.abs(bs.traf.vs) > 0 # Are aircraft performing a vertical maneuver?
        emergency = bs.traf.priority == 4
        speed_zero = np.array(bs.traf.selspd) == 0 # The selected speed is 0, so we're at our destination and landing
        lnav_on = bs.traf.swlnav
        rogue = bs.traf.roguetraffic.rogue_bool
        still_going_to_dest = np.logical_and(abs(degto180(bs.traf.trk - bs.traf.ap.qdr2wp)) < 160.0, 
                                       bs.traf.ap.dist2wp > 5)
        landing = np.logical_and.reduce((np.logical_not(lnav_on), 
                                         bs.traf.actwp.swlastwp,
                                         np.logical_not(still_going_to_dest)))
        
        # CRUISE SPEED STUFF -----------------------------------------
        set_cruise_speed = np.logical_and.reduce((lnav_on, np.logical_not(rogue)))
        
        # Set cruise speed to maximum speed, as aircraft will automatically select
        # a turn speed or CR speed. 
        bs.traf.selspd = np.where(set_cruise_speed, bs.traf.actedge.speed_limit, bs.traf.selspd)
        
        
        # ALTITUDE STUFF-------------------------------------------------------
        # If an aircraft is turning, it should be in a turn layer. Thus, for aircraft that
        # are in a turn, set their selected
        target_turn_layer = np.where(bs.traf.closest_turn_layer_bottom == 0, 
                                      bs.traf.closest_turn_layer_top, 
                                      bs.traf.closest_turn_layer_bottom)*ft
        
        in_turn_layer = bs.traf.flight_layer_type == 'T'
        
        in_constrained = np.where(bs.traf.actedge.edge_airspace_type == 'open', False, True)
        
        give_turn_command = np.logical_and.reduce((in_turn, 
                                                   np.logical_not(in_vert_man), 
                                                   np.logical_not(in_turn_layer),
                                                   in_constrained,
                                                   target_turn_layer != 0,
                                                   lnav_on,
                                                   np.logical_not(rogue)))
        # if 'D53' in bs.traf.id:
        #     idx = bs.traf.id.index('D53')
        #     print('zzz', give_turn_command[idx])
        #     print(in_turn[idx])
        #     print(np.logical_not(in_vert_man)[idx])
        #     print(np.logical_not(in_turn_layer)[idx])
        #     print(in_constrained[idx])
        #     print(target_turn_layer[idx])
        #     print(lnav_on[idx])
        
        bs.traf.selalt = np.where(give_turn_command, target_turn_layer, bs.traf.selalt)

        # GET AIRCRAFT BACK IN A CRUISE LAYER -------------------------------------
        # If an aircraft is not turning, then it should be in a cruise layer
        # Always aim for the bottom cruise layer. However, if both bottom and top
        # are 0, then remain at the same altitude.
        # First, check if we can go up or down
        can_ascend_cruise, can_descend_cruise = self.ascent_descent(64, 70, -70)
        
        # This array is true for aircraft that can simply go down
        can_go_down = np.logical_and(bs.traf.closest_cruise_layer_bottom != 0, can_descend_cruise)
        
        # This array is true for aircraft that can go up
        can_go_up = np.logical_and(bs.traf.closest_cruise_layer_top != 0,can_ascend_cruise)
        
        target_cruise_layer = np.where(can_go_down, bs.traf.closest_cruise_layer_bottom, 
                                       # Now we determine the backup option. Can we go up?
                                       np.where(can_go_up,bs.traf.closest_cruise_layer_top, 
                                        # If we cannot go up either, then we check which layer
                                        # is == 0, and we avoid it
                                        np.where(bs.traf.closest_cruise_layer_bottom != 0, 
                                                 bs.traf.closest_cruise_layer_bottom,
                                                 bs.traf.closest_cruise_layer_top)))*ft

        
        # target_cruise_layer = np.where(bs.traf.closest_cruise_layer_bottom == 0, 
        #                                bs.traf.closest_cruise_layer_top,
        #                                bs.traf.closest_cruise_layer_bottom)*ft

        target_unused_layer = np.where(bs.traf.closest_empty_layer_bottom == 0, 
                                       bs.traf.closest_empty_layer_top,
                                       bs.traf.closest_empty_layer_bottom)*ft
        
        target_cruise_layer = np.where(emergency, target_unused_layer, target_cruise_layer)
        
        in_cruise_layer = np.where(emergency, 
                                   bs.traf.flight_layer_type == 'F',
                                   bs.traf.flight_layer_type == 'C')
        
        give_constrained_cruise_command = np.logical_and.reduce((np.logical_not(in_turn),
                                                     np.logical_not(in_vert_man),
                                                     np.logical_not(cr_active),
                                                     np.logical_not(in_cruise_layer),
                                                     in_constrained,
                                                     np.logical_not(speed_zero),
                                                     lnav_on,
                                                     np.logical_not(rogue)))
        
        bs.traf.selalt = np.where(give_constrained_cruise_command, target_cruise_layer, bs.traf.selalt)
        
        # Open airspace altitude selection -----------------------------------------
        give_open_cruise_command = np.logical_and.reduce((np.logical_not(in_constrained),
                                                         np.logical_not(in_vert_man),
                                                         np.logical_not(cr_active),
                                                         np.logical_not(in_turn),
                                                         np.logical_not(speed_zero),
                                                         lnav_on,
                                                         np.logical_not(rogue)))

        bs.traf.selalt = np.where(give_open_cruise_command, 
                                  bs.traf.open_closest_layer*ft,
                                  bs.traf.selalt)
        
        # Going down a layer if possible -------------------------------------------
        # First of all we have to do some checks. 
        # Don't descend to a new cruise layer when a turn is close
        turn_close = bs.traf.ap.dist2turn < 150 #m
        
        can_ascend, can_descend = self.ascent_descent(150, 200, -100)
        
        # Descent command for aircraft that can
        target_descent_layer = np.where(emergency, bs.traf.closest_empty_layer_bottom,
                                bs.traf.closest_cruise_layer_bottom)*ft

        give_descent_command = np.logical_and.reduce((in_constrained,
                                                     np.logical_not(in_vert_man),
                                                     np.logical_not(cr_active),
                                                     np.logical_not(in_turn),
                                                     np.logical_not(turn_close),
                                                     can_descend,
                                                     target_descent_layer != 0,
                                                     np.logical_not(speed_zero),
                                                     lnav_on,
                                                     np.logical_not(rogue)))
        
        bs.traf.selalt = np.where(give_descent_command,  target_descent_layer, bs.traf.selalt)
        
        # Ascent command for aircraf that are stuck behind another
        target_ascent_layer = np.where(emergency, bs.traf.closest_cruise_layer_top,
                                        bs.traf.closest_empty_layer_top)*ft
        
        give_ascent_command = np.logical_and.reduce((in_constrained,
                                                     np.logical_not(in_vert_man),
                                                     np.logical_or(bs.traf.cr.stuck, np.logical_not(can_descend)),
                                                     np.logical_not(in_turn),
                                                     np.logical_not(turn_close),
                                                     can_ascend,
                                                     target_ascent_layer !=0,
                                                     np.logical_not(speed_zero),
                                                     lnav_on,
                                                     np.logical_not(rogue)))

        bs.traf.selalt = np.where(give_ascent_command, bs.traf.closest_cruise_layer_top*ft, bs.traf.selalt)
        # Set the aircraft we gave the command to as unstuck
        bs.traf.cr.stuck = np.where(give_ascent_command, False, bs.traf.cr.stuck)
        
        # If anyone is below 30 ft altitude and going down, make them hold altitude.
        prevent_negative_altitude = np.logical_and.reduce((bs.traf.vs<0,
                                                           bs.traf.alt<30*ft,
                                                           lnav_on,
                                                           np.logical_not(rogue),
                                                           np.logical_not(speed_zero)))
        # Make em go to 30 ft
        bs.traf.selalt = np.where(prevent_negative_altitude, 30*ft, bs.traf.selalt)
        # Stop their negative VS
        bs.traf.selvs = np.where(prevent_negative_altitude, 0, bs.traf.selvs)
        
        # Same prevention in the positive direction.
        prevent_positive_altitude = np.logical_and.reduce((bs.traf.vs>0,
                                                           bs.traf.alt>480*ft,
                                                           lnav_on,
                                                           np.logical_not(rogue),
                                                           np.logical_not(speed_zero)))
        # Make em go to 30 ft
        bs.traf.selalt = np.where(prevent_positive_altitude, 480*ft, bs.traf.selalt)
        # Stop their negative VS
        bs.traf.selvs = np.where(prevent_positive_altitude, 0, bs.traf.selvs)
        
        # Finally, if anyone has lnav off and is not a rogue, make them go to altitude 0 and give them speed 0
        give_0_command = np.logical_and.reduce((np.logical_not(rogue),
                                                landing))
        
        bs.traf.selalt = np.where(give_0_command, 0, bs.traf.selalt)
        bs.traf.selspd = np.where(give_0_command, 0, bs.traf.selspd)
        
        # Aircraft to delete
        # We delete aircraft if:
        # 1. They are at alt 0 with lnav off
        # 2. They are a loitering aircraft with lnav off
        # 3. Above two only if the last waypoint flag is active
        
        delete_array = np.logical_and.reduce((np.logical_not(rogue),
                                              landing,
                                              np.logical_or(bs.traf.alt <0.1, bs.traf.loiter.loiterbool)
                                              ))
        
        if np.any(delete_array):
            # Get the ACIDs of the aircraft to delete
            acids_to_delete = np.array(bs.traf.id)[delete_array]
            for acid in acids_to_delete:
                stack.stack(f'DEL {acid}')
        
    def ascent_descent(self, distance, above_alt, below_alt):
        # We also don't want descents when there are other aircraft below
        # First, find the pairs of aircraft that are close to each other
        ac_pairs = np.argwhere(bs.traf.cd.dist_mat < distance) #m
        # For these aircraft, check the altitude difference
        alt_diff = bs.traf.alt[ac_pairs[:,0]]-bs.traf.alt[ac_pairs[:,1]]
        # Get the aircraft pairs that have a positive altitude difference
        descend_pairs = ac_pairs[np.argwhere(np.logical_and(1*ft<(alt_diff), (alt_diff)< above_alt*ft))]
        ascend_pairs = ac_pairs[np.argwhere(np.logical_and(below_alt*ft<(alt_diff), (alt_diff)< -1*ft))]
        # The first aircraft in these pairs cannot descend, as they are above and close other
        # aircraft
        ac_cannot_descend = descend_pairs[:,0][:,0]
        can_descend = np.ones(bs.traf.ntraf, dtype = bool)
        can_descend[ac_cannot_descend] = np.zeros(len(ac_cannot_descend), dtype = bool)
        
        ac_cannot_ascend = ascend_pairs[:,0][:,0]
        can_ascend = np.ones(bs.traf.ntraf, dtype = bool)
        can_ascend[ac_cannot_ascend] = np.zeros(len(ac_cannot_ascend), dtype = bool)
        
        return can_ascend, can_descend