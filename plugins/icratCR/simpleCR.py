from bluesky.traffic.asas import ConflictResolution
import bluesky as bs
import numpy as np

# This function is present in all plugins, and is used to name the plugin and
# to set the plugin specific settings.
def init_plugin():
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SIMPLECR',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

# For conflict resolution plugins, we subclass the ConflictResolution class.
class SimpleCR(ConflictResolution):
    def __init__(self):
        super().__init__()
        
    # This function is called when conflicts are detected. It is called from within
    # the simulation loop. The parameters that are passed down are conf, ownship and intruder. 
    # The conf parameter is an instance of the ConflictDetection class, and contains information
    # about the conflicts. The ownship and intruder parameters are instances of the Traffic class. 
    # They are named this way to facilitate clear naming when using the Traffic class.  
    def resolve(self, conf, ownship, intruder):
        # We make a copy of the data of the aircraft data, so that we can modify it.
        newgscapped = np.copy(ownship.gs)
        newvs       = np.copy(ownship.vs)
        newalt      = np.copy(ownship.alt)
        newtrack    = np.copy(ownship.trk)
        
        # The conf variable gives us all the aircraft in conflict in a bool array
        # We can iterate over all of these aircraft and make then turn right
        for acidx in np.argwhere(conf.inconf).flatten():  
            # Just add 5 degrees to the track of the aircraft
            newtrack[acidx] = newtrack[acidx] + 5
            
        # We send the new data to the simulation.
        return newtrack, newgscapped, newvs, newalt