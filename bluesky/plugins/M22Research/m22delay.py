import bluesky as bs
import numpy as np

from bluesky.core import Entity
from bluesky import stack

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22DELAY',
        'plugin_type': 'sim',
        #'reset': M22Delay.reset
    }
    return config

class M22Delay(Entity):
    def __init__(self):
        super().__init__()
        self.mean = 0 # Mean delay
        self.delay_probability = 0 # probability of delay
        
    def reset(self):
        pass
        
        
    @stack.command
    def M22cre(self, acid, actype, aclat, aclon, achdg, acalt, acspd):
        """The function to attempt the creation of an aircraft. Delay can be introduced here.
        """
    
    def get_delay(self):
        """Return a random delay for this aircraft. Distribution is exponential
        """
        return np.random.default_rng().exponential(self.mean)
        
    @stack.command
    def setdelay(self, avg=0, std=0):
        """Set the average and standard deviation of the delay.
        """
        self.avg_delay = avg
        self.std_delay = std
        