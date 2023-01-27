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
            self.vs_prev = np.array([], dtype=float)
            self.aircraft_vs_now = np.array([], dtype=np.bool8)
            self.aircraft_vs_prev = np.array([], dtype=np.bool8)
            self.starting_altitude = np.array([], dtype=int)
            self.ending_altitude = np.array([], dtype=int)
            self.starting_layer = np.array([], dtype=str)
            self.ending_layer = np.array([], dtype=str)
            self.starting_transition_and_conf = np.array([], dtype=bool)
            self.during_transition_and_conf = np.array([], dtype=bool)
            self.ac_starting_transition = np.array([], dtype=bool)
            self.ac_ending_transition = np.array([], dtype=bool)
            self.selected_altitude = np.array([], dtype=int)
            self.interrupted_transition = np.array([], dtype=bool)
            self.aircraft_interrupted = np.array([], dtype=bool)
            self.command_from_cr_start = np.array([], dtype=int)
            self.command_from_cr_during = np.array([], dtype=int)
            self.ac_sign_now = np.array([], dtype=str)
            self.ac_sign_prev = np.array([], dtype=str)

        self.transition_start_time = dict()

    def create(self, n=1):
        super().create(n)
        self.command_from_cr_start[-n:] = 10
        self.command_from_cr_during[-n:] = 10

    def update(self):

        # First step is to check which aircraft are currently performing a transition
        self.aircraft_vs_now = np.abs(bs.traf.vs) > 0

        # calculate the difference in vs from prev and now
        self.ac_sign_now = np.where(bs.traf.vs < 0, 'negative', self.ac_sign_now)        
        self.ac_sign_now = np.where(bs.traf.vs > 0, 'postive', self.ac_sign_now)
        self.ac_sign_now = np.where(bs.traf.vs == 0, 'zero', self.ac_sign_now)

        
        # check which aircraft in cr
        reso_pair_arr = np.array(bs.traf.cr.resopairs).flatten()
        aircraft_with_conf = np.logical_or.reduce(
            (
                bs.traf.cd.inconf,
                np.in1d(bs.traf.id, reso_pair_arr),
                
            )
        )

        # Second: check which aircraft have begun a transition
        # This is done by seeing if there are any True values in self.aircraft_with_vs that were not in self.in_transition
        self.ac_starting_transition = np.logical_and.reduce(
            (
                self.aircraft_vs_now, # this checks for cases when there is a difference between aircraft with a vertical speed and those in transition
                np.logical_not(self.aircraft_vs_prev),
            )
        )

        # For aircraft starting a transition check their current layer
        self.starting_layer = np.where(self.ac_starting_transition, bs.traf.flight_layer_type, self.starting_layer)
        self.starting_altitude = np.where(self.ac_starting_transition, bs.traf.flight_levels, self.starting_altitude)
        # also important to get the selected altitude to check at the end whether it was a succesful or failed transition
        self.selected_altitude = np.where(self.ac_starting_transition, np.rint(bs.traf.selalt/ft), self.selected_altitude)
        
        # now also log if the transition was trigerred while the aircraft was in a conflict
        starting_transition_and_conf = np.logical_and.reduce(
            (
                aircraft_with_conf,
                self.ac_starting_transition,
            )
        )

        # for any aircraft in conflict check the latest command received from CR
        command_from_cr_start = np.logical_and.reduce(
            (
                starting_transition_and_conf,
                bs.traf.cr.altitudeCR < 10,
            )
        )
        self.command_from_cr_start = np.where(command_from_cr_start, bs.traf.cr.altitudeCR, self.command_from_cr_start)

        # for any aircraft in conflict check the latest command received from CR
        command_from_cr_during = np.logical_and.reduce(
            (
                aircraft_with_conf,
                self.aircraft_vs_prev,
                bs.traf.cr.altitudeCR < 10,
            )
        )
        self.command_from_cr_during = np.where(command_from_cr_during, bs.traf.cr.altitudeCR, self.command_from_cr_during)

        # also check if the conflict happened during a transition
        during_transition_and_conf = np.logical_and.reduce(
            (
                aircraft_with_conf,
                self.aircraft_vs_prev,
            )
        )
        # now check
        # save for ending transition
        self.starting_transition_and_conf = np.where(starting_transition_and_conf, starting_transition_and_conf, self.starting_transition_and_conf)
        self.during_transition_and_conf = np.where(during_transition_and_conf, during_transition_and_conf, self.during_transition_and_conf)
       
        # self.transition_and_conf = np.where(during_transition_and_conf, )
        # Third: check which aircraft have ended a transition
        self.ac_ending_transition = np.logical_and.reduce(
            (
                
                np.logical_not(self.aircraft_vs_now), # this checks for cases when there is a difference between aircraft with a vertical speed and those in transition
                self.aircraft_vs_prev,
            )
        )

        # There is another condition for ending a transition and this is when the sign of the bs.traf.vs changes in one iteration


        # for aircraft ending a transition check their current layer
        self.ending_layer = np.where(self.ac_ending_transition, bs.traf.flight_layer_type, self.ending_layer)
        self.ending_altitude = np.where(self.ac_ending_transition, bs.traf.flight_levels, self.ending_altitude)
        
        # now do a check for interrupted transitions due to a conflict
        self.interrupted_transition = np.logical_and.reduce(
            (
                self.ac_ending_transition,
                np.rint(bs.traf.alt/ft) != self.selected_altitude
            )
            
        )

        # begin my log
        self.log()

        # keep count of which aircraft where interrupted to log the recover transition
        self.aircraft_interrupted = np.where(self.interrupted_transition, True, self.aircraft_interrupted)

        # save stuff for next iteration
        self.aircraft_vs_prev = self.aircraft_vs_now
        self.vs_prev = bs.traf.vs
        self.ac_sign_prev = self.ac_sign_now

    def log(self):

        id_arr = np.asarray(bs.traf.id, dtype=object)


        # Only select aircraft in constrained airspace
        in_constrained = np.where(bs.traf.actedge.edge_airspace_type == 'open', False, True)

        # don't log aircraft that are priority 4 since they travel in unused layers
        emergency = bs.traf.priority == 4
        
        # case 1 interrupted transition
        # TODO: log type of interruption from Cruise from Turn? etc...
        interrupted_transition = np.logical_and.reduce(
            (
                self.interrupted_transition,
                in_constrained,
                np.logical_not(emergency)
            )
        )

        # case 2: recovering transition
        recover_transition =  np.logical_and.reduce(
            (
                self.ac_ending_transition,
                self.aircraft_interrupted,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
            )
        )

        # case 3: transtion due to CR would mainly happen if there was a conflict
        # at the start of the transition
        cr_trans = np.logical_and.reduce(
            (
                self.ending_layer != 'T',
                self.starting_transition_and_conf,
                self.ac_ending_transition,
                in_constrained,
                self.command_from_cr_start != 2, # ensure that CR did not tell aircraft to hold
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 4
        # smart hop up: There are some cases where CR tells the aircraft to hold
        # however, if M2 NAV notices that there is space above then it will 
        # perform hop regardless of CR
        smart_hop = np.logical_and.reduce(
            (
                self.ending_layer != 'T',
                self.starting_transition_and_conf,
                self.ac_ending_transition,
                in_constrained,
                self.command_from_cr_start == 2, # ensure that CR did not tell aircraft to hold
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 5: transition due to hopping up
        hopping_up =  np.logical_and.reduce(
            (
                np.logical_not(self.starting_transition_and_conf),
                self.starting_layer == 'C',
                self.ending_layer == 'C',
                self.ending_altitude - self.starting_altitude > 0,
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 6: transiton to hopping down
        hopping_down =  np.logical_and.reduce(
            (
                np.logical_not(self.starting_transition_and_conf),
                self.starting_layer == 'C',
                self.ending_layer == 'C',
                self.ending_altitude - self.starting_altitude < 0,
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 7: transition due to turning
        cruise_to_turn_trans = np.logical_and.reduce(
            (
                self.starting_layer== 'C',
                self.ending_layer == 'T',
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 8: transition due to returning to cruise
        turn_to_cruise_trans = np.logical_and.reduce(
            (
                self.starting_layer == 'T',
                self.ending_layer == 'C',
                np.logical_not(self.starting_transition_and_conf),
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 9: Takeoff
        takeoff_trans = np.logical_and.reduce(
            (
                self.starting_altitude == 0,
                np.logical_not(self.starting_transition_and_conf),
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)
            )
        )

        # case 10 is from a free to a cruise or a turn
        # this usually happens when turn is to close to take off
        # or when entering constrained airspace from open airspace
        free_to_other_transition = np.logical_and.reduce(
            (
                self.ac_ending_transition,
                self.starting_layer == 'F',
                np.logical_not(emergency),
                np.logical_not(takeoff_trans),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition)

            )
        )

        # case 11 missed transition
        missed_transitions = np.logical_and.reduce(
            (
                np.logical_not(cr_trans),
                np.logical_not(smart_hop),
                np.logical_not(hopping_up),
                np.logical_not(hopping_down),
                np.logical_not(cruise_to_turn_trans),
                np.logical_not(turn_to_cruise_trans),
                np.logical_not(takeoff_trans),
                np.logical_not(interrupted_transition),
                np.logical_not(recover_transition),
                np.logical_not(free_to_other_transition),
                self.ac_ending_transition,
                in_constrained,
                np.logical_not(emergency)

            )
        )

        if np.any(cr_trans):
            print('CR transition')
            print(bs.sim.simt)
            print(id_arr[cr_trans])
            print('Command from CR')
            print(self.command_from_cr_start[cr_trans])
            print('----------------')

        if np.any(smart_hop):
            print('Smart transition')
            print(bs.sim.simt)
            print(id_arr[smart_hop])
            print('Command from CR')
            print(self.command_from_cr_start[smart_hop])
            print('----------------')

        if np.any(hopping_up):
            print('Hopping up')
            print(bs.sim.simt)
            print(id_arr[hopping_up])
            print('----------------')

        if np.any(hopping_down):
            print('Hopping down')
            print(bs.sim.simt)
            print(id_arr[hopping_down])
            print('----------------')

        if np.any(cruise_to_turn_trans):
            print('Cruise to Turn transition')
            print(bs.sim.simt)
            print(id_arr[cruise_to_turn_trans])
            print('----------------')

        if np.any(turn_to_cruise_trans):
            print('Turn to Cruise transition')
            print(bs.sim.simt)
            print(id_arr[turn_to_cruise_trans])
            print('----------------')

        if np.any(takeoff_trans):
            print('Takeoff transition')
            print(bs.sim.simt)
            print(id_arr[takeoff_trans])
            print('----------------')

        if np.any(interrupted_transition):
            print('Interrupted transition')
            print(bs.sim.simt)
            print(id_arr[interrupted_transition])
            print('Command from CR')
            print(self.command_from_cr_during[interrupted_transition])
            print('----------------')

        if np.any(recover_transition):
            print('Recover transition')
            print(bs.sim.simt)
            print(id_arr[recover_transition])
            print('----------------')

        if np.any(free_to_other_transition):
            print('Free layer transition')
            print(bs.sim.simt)
            print(id_arr[free_to_other_transition])
            print('----------------')

        if np.any(missed_transitions):
            print('Missed transition')
            print(bs.sim.simt)
            print(id_arr[missed_transitions])
            print('----------------')


        # Some HOUSEKEEPING

        # after logging a recover transition ensure that it gets removed from recovering transitions
        recovered_ac = np.logical_and.reduce(
            (
                recover_transition,
                self.aircraft_interrupted,
                in_constrained
            )
        )
        self.aircraft_interrupted = np.where(recovered_ac, False, self.aircraft_interrupted)


        # aircraft may have turned even if they were in self.transition_and_conf
        # aircraft may have done the cr transition already so clean it up
        # Therefore after a cruise_to_turn transition we should again set self.transition_and_costartinf to false
        starting_cleanup_conf_transition = np.logical_and.reduce(
            (
                self.ac_ending_transition,
                self.starting_transition_and_conf                
            )
        )
        self.starting_transition_and_conf = np.where(starting_cleanup_conf_transition, False, self.starting_transition_and_conf)
        self.starting_transition_and_conf = np.where(in_constrained, self.starting_transition_and_conf, False)
        self.command_from_cr_start = np.where(starting_cleanup_conf_transition, 10, self.command_from_cr_start)

        # aircraft may have finished the cr transition
        cleanup_during_transition_conf = np.logical_and.reduce(
            (
                self.ac_ending_transition,
                self.during_transition_and_conf                
            )
        )

        self.during_transition_and_conf = np.where(cleanup_during_transition_conf, False, self.during_transition_and_conf)
        self.during_transition_and_conf = np.where(in_constrained, self.during_transition_and_conf, False)
        self.command_from_cr_during = np.where(cleanup_during_transition_conf, 10, self.command_from_cr_during)
