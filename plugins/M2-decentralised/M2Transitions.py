from bluesky.core.simtime import timed_function
import bluesky as bs
import numpy as np
from bluesky import core
from bluesky import stack
from bluesky.tools.geo import kwikdist, kwikqdrdist, latlondist, qdrdist
from bluesky.tools.aero import nm, ft, kts
from bluesky.tools.misc import degto180

trans_log = None
def init_plugin():

    # Addtional initilisation code
    trans_log = M2Transitions()
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'M2TRANSITIONS',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        'update':           trans_log.update
    }

    return config

class M2Transitions(core.Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.aircraft_vs_now = np.array([], dtype=np.bool8)
            self.aircraft_vs_prev = np.array([], dtype=np.bool8)
            self.starting_altitude = np.array([], dtype=int)
            self.ending_altitude = np.array([], dtype=int)
            self.starting_layer = np.array([], dtype=str)
            self.ending_layer = np.array([], dtype=str)
            self.transition_and_conf = np.array([], dtype=bool)

        
        self.transition_start_time = dict()

    def update(self):

        # First step is to gather some nice information about the current situation
        self.aircraft_vs_now = np.abs(bs.traf.vs) > 0
        
        # check which aircraft in cr
        reso_pair_arr = np.array(bs.traf.cr.resopairs).flatten()
        aircraft_with_conf = np.logical_or.reduce(
            (
                bs.traf.cd.inconf,
                np.in1d(bs.traf.id, reso_pair_arr)
                
            )
        )

        # Second: check which aircraft have begun a transition
        # This is done by seeing if there are any True values in self.aircraft_with_vs that were not in self.in_transition
        ac_starting_transition = np.logical_and.reduce(
            (
                self.aircraft_vs_now, # this checks for cases when there is a difference between aircraft with a vertical speed and those in transition
                np.logical_not(self.aircraft_vs_prev),
            )
        )

        # For aircraft starting a transition check their current layer
        self.starting_layer = np.where(ac_starting_transition, bs.traf.flight_layer_type, self.starting_layer)
        self.starting_altitude = np.where(ac_starting_transition, bs.traf.flight_levels, self.starting_altitude)

        # now also log if the transition was trigerred while the aircraft was in a conflict
        starting_transition_and_conf = np.logical_and.reduce(
            (
                aircraft_with_conf,
                ac_starting_transition
            )
        )
        # save for ending transition
        self.transition_and_conf = np.where(starting_transition_and_conf, starting_transition_and_conf, self.transition_and_conf)

        # Third: check which aircraft have ended a transition
        ac_ending_transition = np.logical_and.reduce(
            (
                np.logical_not(self.aircraft_vs_now), # this checks for cases when there is a difference between aircraft with a vertical speed and those in transition
                self.aircraft_vs_prev
            )
        )

        # for aircraft ending a transition check their current layer
        self.ending_layer = np.where(ac_ending_transition, bs.traf.flight_layer_type, self.ending_layer)
        self.ending_altitude = np.where(ac_ending_transition, bs.traf.flight_levels, self.ending_altitude)
        
        # begin my log
        self.log(self.starting_layer, self.ending_layer, self.ending_altitude, self.starting_altitude, self.transition_and_conf)

        # set aircraft with vs to in transition
        self.aircraft_vs_prev = self.aircraft_vs_now

    def log(self):
        
        # case 1: transtion due to CR
        cr_trans = np.logical_and.reduce(
            (
                self.ending_altitude != 'T',
                self.transition_and_conf
            )
        )

        # case 2: transition due to hopping up
        hopping_up =  np.logical_and.reduce(
            (
                np.logical_not(self.transition_and_conf),
                self.starting_layer == 'C',
                self.ending_layer == 'C',
                self.ending_altitude - self.starting_altitude > 0
            )
        )

        # case 3: transiton to hopping down
        hopping_down =  np.logical_and.reduce(
            (
                np.logical_not(self.transition_and_conf),
                self.starting_layer == 'C',
                self.ending_layer == 'C',
                self.ending_altitude - self.starting_altitude < 0
            )
        )

        # case 4: transition due to turning
        cruise_to_turn_trans = np.logical_and.reduce(
            (
                self.starting_altitude == 'C',
                self.ending_altitude == 'T',
            )
        )

        # case 5: transition due to turning
        turn_to_cruise_trans = np.logical_and.reduce(
            (
                self.starting_altitude == 'T',
                self.ending_altitude == 'C',
                np.logical_not(self.transition_and_conf)
            )
        )