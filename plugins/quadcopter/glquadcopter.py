""" Plugin to replace aircraft symbol with a quadcopter """
import numpy as np
from os import path

from bluesky import settings
from bluesky.ui.qtgl.gltraffic import Traffic

# Register settings defaults
settings.set_variable_defaults(
    text_size=13, ac_size=16,
    asas_vmin=200.0, asas_vmax=500.0)

### Initialization function of plugin.
def init_plugin():
    config = {
        'plugin_name':     'QUADGUI',
        'plugin_type':     'gui',
        }

    return config

# Bird traffic class
class QuadTraffic(Traffic):
    # TODO: FIX labels
    def __init__(self):
        super().__init__()
    
    def create(self):
        super().create()

        # get aircraft size
        ac_size = settings.ac_size

        # set square in aircraft vertices
        acvertices = np.array([(-1.0*ac_size, -1.0 * ac_size), (-1.0 * ac_size, 1.0 * ac_size),
                               (1.0*ac_size, 1.0 * ac_size), (1.0 * ac_size, -1.0 * ac_size)],
                              dtype=np.float32)

        # texture coordinates
        texcoords = np.array([1, 1, 1, 0, 0, 0, 0, 1], dtype=np.float32)

        # filepath of the texture
        fname = path.join(settings.plugin_path, 'quadcopter', 'quadcopter.png')

        # create the vertex array object. NOTE: will get a warning. But ignore it.
        self.ac_symbol.create(vertex=acvertices, texcoords=texcoords, texture=fname)

