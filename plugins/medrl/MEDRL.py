import torch
import numpy as np
import bluesky as bs
from bluesky.core import Entity, timed_function
import matplotlib.pyplot as plt
import torch.nn.functional as F
import torch.optim as optim
from torch.nn.utils.clip_grad import clip_grad_norm_
from typing import Dict, List, Deque, Tuple
from collections import deque
import torch.nn as nn
from torch.distributions import Normal
from geofence import Geofence
from bluesky.tools.geo import kwikqdrdist_matrix, kwikqdrdist
from bluesky.tools import areafilter
from bluesky.stack import stack
import geopandas as gpd
import shapely.geometry as geom
from shapely.ops import nearest_points
from bluesky.tools.aero import nm
import os
from plugins.medrl.sac_agent import SAC as Agent
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

ML_DT = 1.0 #seconds
ML_STEPS = 1 #Steps
ML_ACTION_DT = 1 # seconds

MAX_HEADING_CHANGE = 20 # degrees
MAX_SIMT = 200 # seconds

SHOW_LINES = False
FAST = True

episode_counter = 0

avg_rewards = []

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

class MedRL(Entity):
    def __init__(self):
        super().__init__()
        # Initialise stuff
        self.Agent = Agent(1, 8)
        
        self.step_counter = 0
        self.time_passed_counter = 0
        
        self.reward_history = []
        self.state = [0,0,0,0,0,0,0,0] # dr, dd, sina1, cosa1, sina2, cosa2, sinbd, cosbd
        self.state_ = [0,0,0,0,0,0,0,0]
        self.action = 0
            
        # Create the scenario
        create_scenario()
    
        return
    
    @timed_function(dt=ML_DT)
    def step(self):
        # Get current state
        self.state = self.state_
        
        self.state_ = self.get_state(0)
        
        if self.step_counter == 0:
            self.step_counter += 1
            self.time_passed_counter += 1
            return
        
        reward, done, reason = self.get_reward(0, self.state, self.state_)
        
        self.reward_history.append(reward)
        
        if done:
            stack('HOLD')
            global episode_counter
            episode_counter += 1
            avg_rewards.append(sum(self.reward_history))
            while len(avg_rewards) > 100:
                avg_rewards.pop(0)
            # Print episode number, average reward, and average loss
            print(f'----------------- EPISODE {episode_counter} -----------------')
            print(f'Rolling average reward: {np.mean(avg_rewards):.3f}')
            print(f'Average reward for this episode: {sum(self.reward_history)/len(self.reward_history):.3f}')
            print(reason)
            print('--------------------------------------------------------------')
            self.ML_reset()
            return
        
        if self.step_counter != 0:
            self.Agent.memory.store(self.state, self.action, reward, self.state_, done)
        
        # Only get a new action if enough seconds passed
        time_passed = ML_DT * self.time_passed_counter
        if time_passed > ML_ACTION_DT or self.step_counter == 1:
            self.time_passed_counter = 0
            self.action = self.Agent.step(self.state_)
        
        # Compute heading change
        heading_change = self.action * MAX_HEADING_CHANGE
        
        # Execute action
        stack(f'HDG {bs.traf.id[0]} {heading_change}')
        
        if self.step_counter % ML_STEPS == 0:
            self.Agent.train()
            
        self.step_counter += 1
        self.time_passed_counter += 1
        
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
        
        # Compute the absolute qdrs to all the points of the geofence
        geoqdrs, _ = kwikqdrdist_matrix(ac_lat, ac_lon, geolats, geolons)
        
        # Compute the relative bearings for all the geoqdrs
        geoqdr_rel = ((geoqdrs - ac_hdg) + 180) % 360 - 180
        
        # a1 is the smallest absolute qdr, a2 is the biggest
        a1 = min(geoqdr_rel)
        a2 = max(geoqdr_rel)
        
        sina1 = np.sin(np.deg2rad(a1))
        cosa1 = np.cos(np.deg2rad(a1))
        sina2 = np.sin(np.deg2rad(a2))
        cosa2 = np.cos(np.deg2rad(a2))
        
        
        # Get the point index
        a1idx = np.where(geoqdr_rel == a1)[0][0]
        a2idx = np.where(geoqdr_rel == a2)[0][0]
        
        # Smallest distance to geofence, just take the lat
        # First, get the nearest point to the aircraft in the geofence
        geopoly = geom.Polygon(zip(geolats, geolons))
        # Then create a point out of the aircraft position
        ac_point = geom.Point(ac_lat, ac_lon)
        # Find the nearest point to the aircraft in the geofence
        p1, _ = nearest_points(geopoly, ac_point)
        
        # Now use kwikdist to find the distance
        _, dr = kwikqdrdist(ac_lat, ac_lon, p1.x, p1.y)
        
        # Convert dr to metres
        dr = dr * nm
        
        if SHOW_LINES:
            if self.step_counter > 0:
                areafilter.deleteArea('a1LINE')
                areafilter.deleteArea('a2LINE')
                areafilter.deleteArea('drLINE')
                
            
            # We can actually draw these lines so we see how the aircraft functions
            areafilter.defineArea('a1LINE', "LINE", [ac_lat, ac_lon, geolats[a1idx], geolons[a1idx]])
            areafilter.defineArea('a2LINE', "LINE", [ac_lat, ac_lon, geolats[a2idx], geolons[a2idx]])
            areafilter.defineArea('drLINE', "LINE", [ac_lat, ac_lon, p1.x, p1.y])
        
        # Distance to destination
        bd_abs, dd = kwikqdrdist(ac_lat, ac_lon, AC_DESTINATION_LATLON[0], AC_DESTINATION_LATLON[1])
        dd = dd * nm
        bd = ((bd_abs - ac_hdg) + 180) % 360 - 180 
        
        sinbd = np.sin(np.deg2rad(bd))
        cosbd = np.cos(np.deg2rad(bd))
        
        return [dr/2500, (dd-2500)/5000, sina1, cosa1, sina2, cosa2, sinbd, cosbd]
        
    def get_reward(self, acidx, state, state_):
        ac_lat = bs.traf.lat[acidx]
        ac_lon = bs.traf.lon[acidx]
        done = False
        reason = None
        reward = 0
        # If distance to destination is less than 100m we are done
        dist2dest = state_[1] * 5000 + 2500
        if dist2dest < 300  and dist2dest != 0:
            reason = 'Reached destination.'
            done = True
            reward += 2
        
        # Check if we hit the geofence
        bbox = Geofence.geo_by_name['AIGEO'].bbox
        if bbox[0] < ac_lat < bbox[2] and bbox[1] < ac_lon < bbox[3]:
            reason = 'Hit geofence.'
            done = True
            reward -= 3
            
        # Stop if simulation time is more than 1 minute
        if bs.sim.simt > MAX_SIMT:
            reason = 'Simulation time is more than MAX_SIMT.'
            done = True
        
        # Look at previous state and new state, and give a reward based on the change in state
        diff_in_state = state[1] - state_[1]
        #dist2dest = dist2dest / nm # Get it in nautical miles as it's a good order of magnitude
        reward += diff_in_state
        
        return reward, done, reason
    
    def ML_reset(self):
        # This is called when we are done. First, call a simulation-wide reset
        bs.sim.reset()
        
        # Reset the rest
        self.step_counter = 0
        
        self.reward_history = []
        self.state = [0,0,0,0,0,0,0,0] # dr, dd, aL, aR, bd
        self.state_ = [0,0,0,0,0,0,0,0]
        self.action = 0
            
        # Create the scenario again
        create_scenario()
        return
    
def create_scenario():
    ##### TUNING PARAMETERS #####
    # create a point where the center is at
    # Get the point as a random number between -1 and 1
    origin_lat = 0
    origin_lon = 0

    # set the width and depth of a rectangle meters
    # To randomize?
    depth = 500 # meters
    width = 2000 # meters

    # now set the origin of the aircraft to be a certain distance from the border of rectangle
    dist_origin_x = 0 # meters
    dist_origin_y = 14000 # meters

    # distance from the border of top of rectangle
    dist_destination_x = 0 # meters
    dist_destination_y = 14000 # meters

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

    # create random width offset 
    width_left = np.random.uniform(low=0, high=width)
    width_right = np.random.uniform(low=0, high=width)
    
    # create a rectangle centered at point_df with depth and width
    rectangle = geom.box(point_df.geometry.x.values[0] - width_left, point_df.geometry.y.values[0] - depth/2, point_df.geometry.x.values[0] + width_right, point_df.geometry.y.values[0] + depth/2)
    rectangle_df = gpd.GeoDataFrame(geometry=[rectangle], crs="EPSG:3857")

    # convert everything to lat lon
    origin_df = origin_df.to_crs(epsg=4326)
    destination_df = destination_df.to_crs(epsg=4326)
    rectangle_df = rectangle_df.to_crs(epsg=4326)

    # create geodence command which is a sequence of lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4
    xy_values = rectangle_df.geometry.values[0].exterior.coords.xy
    lat_lon = [f'{lat}, {lon}' for lon, lat in zip(xy_values[0], xy_values[1])]
    stack('GEOFENCE,AIGEO,25000,0, ' + ','.join(lat_lon))
    
    # Get a random heading between -90 and 90 mapped to 0-360
    hdg = 0 #np.random.randint(-90, 90)

    # create an aircraft
    stack(f'CRE AI01 B744 {origin_df.geometry.y.values[0]} {origin_df.geometry.x.values[0]} {hdg} FL250 200')

    global AC_DESTINATION_LATLON
    AC_DESTINATION_LATLON = [destination_df.geometry.y.values[0], destination_df.geometry.x.values[0]]
    # add a waypoint
    stack(f'ADDWPT AI01 {destination_df.geometry.y.values[0]} {destination_df.geometry.x.values[0]}')
    stack('OP')
    stack('SCHEDULE 00:00:01 PAN 0,0')
    stack('SCHEDULE 00:00:01 ZOOM 2')
    stack('SCHEDULE 00:00:01 AI01')
    if FAST:
        stack('FF')
    
    return