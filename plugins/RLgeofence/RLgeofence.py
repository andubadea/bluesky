import numpy as np
import geopandas as gpd
import shapely.geometry as geom

import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.tools.geo import kwikqdrdist_matrix, kwikqdrdist, qdrpos
from bluesky.stack import stack
from bluesky.tools.aero import nm

from geofence import Geofence
from sac_agent import SAC

# import os
# os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

def init_plugin():

    # Addtional initilisation code
    global medrl
    medrl = MedRL()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'MEDRL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
        }
    
    return config


ML_DT = 1.0 #seconds
ML_STEPS = 1 #Steps

MAX_HEADING_CHANGE = 6 * ML_DT # degrees

episode_counter = 0

avg_rewards = []

    
class MedRL(Entity):
    def __init__(self):
        super().__init__()
        # Initialise stuff
        self.Agent = SAC(1, 5)
        
        self.step_counter = 0
        
        self.reward_history = []
        self.state = [0,0,0,0,0] # dr, dd, aL, aR, bd
        self.state_ = [0,0,0,0,0]
        self.action = 0
            
        # Create the scenario
        create_scenario()
    
        return
    
    @timed_function(dt=ML_DT)
    def step(self):
        # Get current state
        self.state = self.state_
        
        self.state_ = self.get_state(0)
        
        reward, done = self.get_reward(0, self.state, self.state_)
        
        if self.step_counter == 0:
            self.step_counter += 1
            return
        
        self.reward_history.append(reward)
        
        if done:
            stack('HOLD')
            global episode_counter
            episode_counter += 1
            avg_rewards.append(sum(self.reward_history)/len(self.reward_history))
            while len(avg_rewards) > 100:
                avg_rewards.pop(0)
            # Print episode number, average reward, and average loss
            print(f'----------------- EPISODE {episode_counter} -----------------')
            print(f'Rolling average reward: {np.mean(avg_rewards):.3f}')
            print(f'Average reward for this episode: {sum(self.reward_history)/len(self.reward_history):.3f}')
            print('\n')
            self.ML_reset()
            return
        
        if self.step_counter != 0:
            self.Agent.memory.store(self.state, self.action, reward, self.state_, done)
            
        self.action = self.Agent.do_step(self.state_)
        
        # Compute heading change
        heading_change = self.action * MAX_HEADING_CHANGE
        
        # Execute action
        stack(f'HDG {bs.traf.id[0]} {heading_change}')
        
        if self.step_counter % ML_STEPS == 0:
            self.Agent.trainChooChoo()
            
        self.step_counter += 1
        
        return
    
    def get_state(self, acidx):
        # Get the geofence data
        geofence = Geofence.geo_by_name['AIGEO']
        
        # Get the aircraft data
        ac_lat = bs.traf.lat[acidx]
        ac_lon = bs.traf.lon[acidx]
        ac_hdg = bs.traf.trk[acidx]
        
        # Get the points of the geofence
        geolats = geofence.coordinates[::2]
        geolons = geofence.coordinates[1::2]
        
        # Get the relative bearing of all points of the rectangle
        abs_brg, _ = kwikqdrdist_matrix(ac_lat, ac_lon, geolats, geolons)
        
        bearings = ((abs_brg - ac_hdg) + 180) % 360 - 180  
        
        a1 = max(bearings)
        a2 = min(bearings)
        
        # Distance to geofence, just take the lat
        dr = ac_lat
        
        # Distance to destination
        bd_abs, dd = kwikqdrdist(ac_lat, ac_lon, AC_DESTINATION_LATLON[0], AC_DESTINATION_LATLON[1])
        dd = dd * 1852
        bd = ((bd_abs - ac_hdg) + 180) % 360 - 180 
        
        return [dr, dd, a1, a2, bd]
        
    def get_reward(self, acidx, state, state_):
        ac_lat = bs.traf.lat[acidx]
        ac_lon = bs.traf.lon[acidx]
        ac_hdg = bs.traf.hdg[acidx]
        done = False
        reward = 0
        # If distance to destination is less than 100m we are done
        if state[1] < 100 and state[1] != 0:
            print('Reached destination.')
            done = True
            reward += 1
        
        # Check if we hit the geofence
        bbox = Geofence.geo_by_name['AIGEO'].bbox
        if bbox[0] < ac_lat < bbox[2] and bbox[1] < ac_lon < bbox[3]:
            print('Hit geofence.')
            done = True
            reward -= 2
            
        # Reward the aircraft as it gets closer to the destination
        _, dist2dest = kwikqdrdist(ac_lat, ac_lon, AC_DESTINATION_LATLON[0], AC_DESTINATION_LATLON[1])
        reward -= dist2dest
        
        return reward, done
    
    def ML_reset(self):
        # This is called when we are done. First, call a simulation-wide reset
        bs.sim.reset()
        
        # Reset the rest
        self.step_counter = 0
        
        self.reward_history = []
        self.state = [0,0,0,0,0] # dr, dd, aL, aR, bd
        self.state_ = [0,0,0,0,0]
        self.action = 0
            
        # Create the scenario again
        create_scenario()
        return
    
def create_scenario():
    ##### TUNING PARAMETERS #####
    # create a point where the center is at
    origin_lat = 0
    origin_lon = 0

    # set the width and depth of a rectangle meters
    # To randomize?
    depth = 500 # meters
    width = 4000 # meters

    # now set the origin of the aircraft to be a certain distance from the border of rectangle
    dist_origin_x = 0 # meters
    dist_origin_y = 4000 # meters

    # distance from the border of top of rectangle
    dist_destination_x = 0 # meters
    dist_destination_y = 4000 # meters

    ##### END TUNING PARAMETERS #####

    # create geopandas dataframe with point origin
    point_df = gpd.GeoDataFrame(geometry=[geom.Point(origin_lon, origin_lat)], crs="EPSG:4326")

    # convert to crs 3857 to work with meters
    point_df = point_df.to_crs(epsg=3857)

    # get origin point
    origin_x = point_df.geometry.x.values[0] 
    origin_y  = point_df.geometry.y.values[0] - depth/2 - dist_origin_y
    origin_df = gpd.GeoDataFrame(geometry=[geom.Point(origin_x, origin_y)], crs="EPSG:3857")

    # get destination point
    destination_x = point_df.geometry.x.values[0]
    destination_y = point_df.geometry.y.values[0] + depth/2 + dist_destination_y
    destination_df = gpd.GeoDataFrame(geometry=[geom.Point(destination_x, destination_y)], crs="EPSG:3857")

    # create a rectangle centered at point_df with depth and width
    rectangle = geom.box(point_df.geometry.x.values[0] - width/2, point_df.geometry.y.values[0] - depth/2, point_df.geometry.x.values[0] + width/2, point_df.geometry.y.values[0] + depth/2)
    rectangle_df = gpd.GeoDataFrame(geometry=[rectangle], crs="EPSG:3857")

    # convert everything to lat lon
    origin_df = origin_df.to_crs(epsg=4326)
    destination_df = destination_df.to_crs(epsg=4326)
    rectangle_df = rectangle_df.to_crs(epsg=4326)

    # create geodence command which is a sequence of lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4
    xy_values = rectangle_df.geometry.values[0].exterior.coords.xy
    lat_lon = [f'{lat}, {lon}' for lon, lat in zip(xy_values[0], xy_values[1])]
    stack('GEOFENCE,AIGEO,25000,0, ' + ','.join(lat_lon))

    # create an aircraft
    stack(f'CRE AI01 B744 {origin_df.geometry.y.values[0]} {origin_df.geometry.x.values[0]} 0 FL250 200')

    global AC_DESTINATION_LATLON
    AC_DESTINATION_LATLON = [destination_df.geometry.y.values[0], destination_df.geometry.x.values[0]]
    # add a waypoint
    stack(f'ADDWPT AI01 {destination_df.geometry.y.values[0]} {destination_df.geometry.x.values[0]}')
    stack('OP')
    stack('SCHEDULE 00:00:01 PAN 0,0')
    stack('SCHEDULE 00:00:01 ZOOM 10')
    stack('FF')
    
    return