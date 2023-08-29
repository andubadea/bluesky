''' State-based conflict detection. '''
import numpy as np
from bluesky import stack
import bluesky as bs
from bluesky.tools import geo, datalog
from bluesky.tools.aero import nm
from bluesky.traffic.asas import ConflictDetection

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'M22CD',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

confheader = \
    '#######################################################\n' + \
    'CONF LOG\n' + \
    'Conflict Statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'Unique CONF ID [-]' + \
    'ACID1 [-],' + \
    'ACID2 [-],' + \
    'LAT1 [deg],' + \
    'LON1 [deg],' + \
    'ALT1 [ft],' + \
    'LAT2 [deg],' + \
    'LON2 [deg],' + \
    'ALT2 [ft]\n'

uniqueconflosheader = \
    '#######################################################\n' + \
    'Unique CONF LOS LOG\n' + \
    'Shows whether unique conflicts results in a LOS\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Unique CONF ID, ' + \
    'Resulted in LOS\n'


class M22CD(ConflictDetection):
    def __init__(self):
        super().__init__()
        self.dist_mat = np.array([])
        self.qdr_mat = np.array([])
        self.rpz_actual = 32 #m
        self.rpz_buffered = 40 #m
        
        # Logging
        self.conflictlog = datalog.crelog('CDR_CONFLICTLOG', None, confheader)
        self.uniqueconfloslog = datalog.crelog('CDR_WASLOSLOG', None, uniqueconflosheader)
        
        # Conflict related
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.unique_conf_dict = dict()
        self.counter2id = dict() # Keep track of the other way around
        self.unique_conf_id_counter = 0 # Start from 0, go up
        self.confhold =  [] # array to keep track of the conflicts we are
        self.already_logged = [] #unique conflict IDs that have already been logged
        return
        
    def clearconfdb(self):
        ''' Clear conflict database. '''
        self.confpairs_unique.clear()
        self.lospairs_unique.clear()
        self.confpairs.clear()
        self.lospairs.clear()
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.inconf = np.zeros(bs.traf.ntraf)
        self.tcpamax = np.zeros(bs.traf.ntraf)
        self.dist_mat = np.array([])
        self.qdr_mat = np.array([])
        
        # Conflict related
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.unique_conf_dict = dict()
        self.counter2id = dict() # Keep track of the other way around
        self.unique_conf_id_counter = 0 # Start from 0, go up
        self.confhold =  [] # array to keep track of the conflicts we are
        self.already_logged = [] #unique conflict IDs that have already been logged
        return
        
    def update(self, ownship, intruder):
        ''' Perform an update step of the Conflict Detection implementation. '''
        self.confpairs, self.lospairs, self.inconf, self.tcpamax, self.qdr, \
            self.dist, self.dcpa, self.tcpa, self.tLOS, self.qdr_mat, self.dist_mat = \
                self.detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique    
        
        # Update the logging
        self.update_log()
        
    def detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' Conflict detection between ownship (traf) and intruder (traf/adsb).'''
        # Calculate everything using the buffered RPZ
        rpz = np.zeros(len(rpz)) + self.rpz_buffered
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------

        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.trk)
        intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        dvs = ownship.vs.reshape(1, ownship.ntraf) - \
            intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) * \
            (tinconf < dtlookahead) * (1.0 - I), dtype=bool)

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        # It's a LOS if the actual RPZ of 32m is violated.
        swlos = (dist < (np.zeros(len(rpz)) + self.rpz_actual)) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return confpairs, lospairs, inconf, tcpamax, \
            qdr[swconfl], dist[swconfl], np.sqrt(dcpa2[swconfl]), \
                tcpa[swconfl], tinconf[swconfl], qdr, dist
                
    def update_log(self):
        '''Here, we are logging the information for current conflicts as well as
        whether these conflicts resulted in a LOS or not.'''
        confpairs_new = list(set(self.confpairs) - self.prevconfpairs) # New confpairs
        confpairs_out = list(self.prevconfpairs - set(self.confpairs)) # Pairs that are no longer in conflict
        lospairs_new = list(set(self.lospairs) - self.prevlospairs) # New lospairs
        
        # First of all, add the new conflicts to the unique dict tracker
        for confpair in confpairs_new:
            # The dict is of the following format:
            # lower_number_acidx_newer_number_acidx : [unique_id, was it a LOS or not]
            # First, get the aircraft IDX
            idx1 = bs.traf.id.index(confpair[0])
            idx2 = bs.traf.id.index(confpair[1])
            # Create dictionary entry
            if idx1 < idx2:
                dictkey = confpair[0] + confpair[1]
            else:
                dictkey = confpair[1] + confpair[0]
                
            if dictkey in self.unique_conf_dict:
                # Pair already in there
                continue
            else:
                self.unique_conf_dict[dictkey] = [self.unique_conf_id_counter, False]
                self.counter2id[self.unique_conf_id_counter] = dictkey 
                self.unique_conf_id_counter += 1
                
        # Log data in the conflog for all existing conflicts
        done_pairs = []
        for confpair in self.confpairs:
            idx1 = bs.traf.id.index(confpair[0])
            idx2 = bs.traf.id.index(confpair[1])
            if idx1 < idx2:
                dictkey = confpair[0] + confpair[1]
            else:
                dictkey = confpair[1] + confpair[0]
                
            if dictkey in done_pairs:
                # Already done, continue
                continue
            
            self.conflictlog.log(
                self.unique_conf_dict[dictkey][0],
                confpair[0],
                confpair[1],
                bs.traf.lat[idx1],
                bs.traf.lon[idx1],
                bs.traf.alt[idx1],
                bs.traf.lat[idx2],
                bs.traf.lon[idx2],
                bs.traf.alt[idx2]
            )
            
        # Now check the new LOS
        done_pairs = []
        for lospair in lospairs_new:
            # Set the los flag of these in the unique dict tracker
            idx1 = bs.traf.id.index(lospair[0])
            idx2 = bs.traf.id.index(lospair[1])
            if idx1 < idx2:
                dictkey = lospair[0] + lospair[1]
            else:
                dictkey = lospair[1] + lospair[0]
                
            if dictkey in done_pairs:
                # Already done, continue
                continue
            
            done_pairs.append(dictkey)
                
            # Set the bool as true
            if dictkey in self.unique_conf_dict:
                self.unique_conf_dict[dictkey][1] = True
            else:
                # This LOS was not detected, but it is usually because of weird geometry
                #print(dictkey)
                continue
                
            
        # Now handle aircraft that are no longer in confpairs
        done_pairs = []
        for confpair in confpairs_out:
            # There is a possibility that one aircraft thinks it is still in a conflict while the other
            # doesn't. If this confpair is still in confpairs but inverted, skip it
            if (confpair[1], confpair[0]) in self.confpairs:
                continue
            # Log these in the uniqueconfloslog
            if confpair[0] not in bs.traf.id or confpair[1] not in bs.traf.id:
                # One of these aircraft was deleted, so just log them and done.
                if confpair[0] + confpair[1] in self.unique_conf_dict:
                    dictkey = confpair[0] + confpair[1]
                elif confpair[1] + confpair[0] in self.unique_conf_dict:
                    dictkey = confpair[1] + confpair[0]
                else:
                    # Absolutely no clue, continue I guess
                    print('huh')
                    continue
                
                self.uniqueconfloslog.log(
                self.unique_conf_dict[dictkey][0],
                str(self.unique_conf_dict[dictkey][1])
                )
                self.unique_conf_dict.pop(dictkey)
                continue
                    
                
            idx1 = bs.traf.id.index(confpair[0])
            idx2 = bs.traf.id.index(confpair[1])
            
            if idx1 < idx2:
                dictkey = confpair[0] + confpair[1]
            else:
                dictkey = confpair[1] + confpair[0]
                
            if dictkey in done_pairs:
                # Already done, continue
                continue
            
            done_pairs.append(dictkey)
            
            # We want to keep this entry for a few extra seconds and see what happens
            # Get the conflict info and log it, then delete the entry
            self.uniqueconfloslog.log(
                self.unique_conf_dict[dictkey][0],
                str(self.unique_conf_dict[dictkey][1])
            )
            self.unique_conf_dict.pop(dictkey)
        
        self.prevconfpairs = set(self.confpairs)
        self.prevlospairs = set(self.lospairs)