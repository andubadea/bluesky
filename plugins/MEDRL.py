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
from bluesky.stack import stack
import geopandas as gpd
import shapely.geometry as geom

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

GAMMMA = 0.97 # The rate at which the future is uncertain and not counted towards the reward
TAU =5e-3 # Low pass filter factor for the neural network that stops it from going too wild from one time step from another
INITIAL_RANDOM_STEPS = 100 # Number of steps to take random actions before learning starts
POLICY_UPDATE_FREQUENCE = 2 # How often to update the policy

BUFFER_SIZE = 10000000 # Number of stuff kept in memory
BATCH_SIZE = 256 # Number of random smaples taken from the buffer 

LR_A = 3e-4 # Learn rate of the actor
LR_Q = 3e-4 # Learn rate of the critic

MEANS = [57000,57000,0,0,0,0,0,0]
STDS = [31500,31500,100000,100000,1,1,1,1]

STATE_SIZE = 5
ACTION_SIZE = 1

ML_DT = 1.0 #seconds
ML_STEPS = 1 #Steps

MAX_HEADING_CHANGE = 6 * ML_DT # degrees

AC_DESTINATION_LATLON = [0,0]

class MaSacAgent:
    def __init__(self, num_agents, action_dim, state_dim):                
        self.statedim = state_dim
        self.actiondim = action_dim
        self.number_intruders_state = 1
        self.use_altitude = False

        self.memory = ReplayBuffer(self.statedim,self.actiondim, BUFFER_SIZE, BATCH_SIZE)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.target_alpha = -np.prod((self.actiondim,)).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=3e-4)

        self.actor = Actor(self.statedim, self.actiondim).to(self.device)

        self.vf = CriticV(self.statedim).to(self.device)
        self.vf_target = CriticV(self.statedim).to(self.device)
        self.vf_target.load_state_dict(self.vf.state_dict())

        self.qf1 = CriticQ(self.statedim + self.actiondim).to(self.device)
        self.qf2 = CriticQ(self.statedim + self.actiondim).to(self.device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=LR_A)
        self.vf_optimizer = optim.Adam(self.vf.parameters(), lr=LR_Q)
        self.qf1_optimizer = optim.Adam(self.qf1.parameters(), lr=LR_Q)
        self.qf2_optimizer = optim.Adam(self.qf2.parameters(), lr=LR_Q)

        self.transition = [[] for i in range(num_agents)]

        self.total_step = 0

        self.is_test = False
        
        if self.device.type == 'cpu':
            print('DEVICE USED', self.device.type)
        else:
            print('DEVICE USED', torch.cuda.device(torch.cuda.current_device()), torch.cuda.get_device_name(0))

    def do_step(self, state, test = False):

        if not test and self.total_step < INITIAL_RANDOM_STEPS and not self.is_test:
            selected_action = np.random.uniform(-1, 1, (len(state), self.actiondim))
        else:
            action = self.actor(torch.FloatTensor(state[i]).to(self.device))[0].detach().cpu().numpy()
            selected_action = np.clip(action, -1, 1)

        self.total_step += 1
        return selected_action.tolist()
    
    def setResult(self,episode_name, state, new_state, reward, action, done):       
        if not self.is_test:
            for i in range(len(state)):               
                self.transition[i] = [state[i], action[i], reward, new_state[i], done]
                self.memory.store(*self.transition[i])

        if (len(self.memory) >  BATCH_SIZE and self.total_step > INITIAL_RANDOM_STEPS):
            self.update_model()
    
    def update_model(self):
        device = self.device

        samples = self.memory.sample_batch()
        state = torch.FloatTensor(samples["obs"]).to(device)
        next_state = torch.FloatTensor(samples["next_obs"]).to(device)
        action = torch.FloatTensor(samples["acts"].reshape(-1, self.actiondim)).to(device)
        reward = torch.FloatTensor(samples["rews"].reshape(-1,1)).to(device)
        done = torch.FloatTensor(samples["done"].reshape(-1, 1)).to(device)
        new_action, log_prob = self.actor(state)

        alpha_loss = ( -self.log_alpha.exp() * (log_prob + self.target_alpha).detach()).mean()

        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()

        alpha = self.log_alpha.exp()

        mask = 1 - done
        q1_pred = self.qf1(state, action)
        q2_pred = self.qf2(state, action)
        vf_target = self.vf_target(next_state)
        q_target = reward + GAMMMA * vf_target * mask
        qf1_loss = F.mse_loss(q_target.detach(), q1_pred)
        qf2_loss = F.mse_loss(q_target.detach(), q2_pred)

        v_pred = self.vf(state)
        q_pred = torch.min(
            self.qf1(state, new_action), self.qf2(state, new_action)
        )
        v_target = q_pred - alpha * log_prob
        v_loss = F.mse_loss(v_pred, v_target.detach())

        if self.total_step % POLICY_UPDATE_FREQUENCE== 0:
            advantage = q_pred - v_pred.detach()
            actor_loss = (alpha * log_prob - advantage).mean()

            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            self._target_soft_update()
        else:
            actor_loss = torch.zeros(1)
        
        self.qf1_optimizer.zero_grad()
        qf1_loss.backward()
        self.qf1_optimizer.step()
        self.qf2_optimizer.zero_grad()
        qf2_loss.backward()
        self.qf2_optimizer.step()

        qf_loss = qf1_loss + qf2_loss

        self.vf_optimizer.zero_grad()
        v_loss.backward()
        self.vf_optimizer.step()

        return actor_loss.data, qf_loss.data, v_loss.data, alpha_loss.data
    
    def save_models(self):
        torch.save(self.actor.state_dict(), "results/mactor.pt")
        torch.save(self.qf1.state_dict(), "results/mqf1.pt")
        torch.save(self.qf2.state_dict(), "results/mqf2.pt")
        torch.save(self.vf.state_dict(), "results/mvf.pt")       

    def load_models(self):
        # The models were trained on a CUDA device
        # If you are running on a CPU-only machine, use torch.load with map_location=torch.device('cpu') to map your storages to the CPU.
        self.actor.load_state_dict(torch.load("results/mactor.pt", map_location=torch.device('cpu')))
        self.qf1.load_state_dict(torch.load("results/mqf1.pt", map_location=torch.device('cpu')))
        self.qf2.load_state_dict(torch.load("results/mqf2.pt", map_location=torch.device('cpu')))
        self.vf.load_state_dict(torch.load("results/mvf.pt", map_location=torch.device('cpu')))
    
    def _target_soft_update(self):
        for t_param, l_param in zip(
            self.vf_target.parameters(), self.vf.parameters()
        ):
            t_param.data.copy_(TAU * l_param.data + (1.0 - TAU) * t_param.data)

    def normalizeState(self, s_t, max_speed, min_speed):
        # distance to closest #NUMBER_INTRUDERS_STATE intruders
        for i in range(0, self.number_intruders_state):
            s_t[i] = (s_t[i]-MEANS[0])/(STDS[0]*2)

        # relative bearing to closest #NUMBER_INTRUDERS_STATE intruders
        for i in range(self.number_intruders_state, self.number_intruders_state*2):
            s_t[i] = (s_t[i]-MEANS[1])/(STDS[1]*2)

        # current dy intruder (from ownship frame of reference)
        for i in range(self.number_intruders_state*2, self.number_intruders_state*3):
            s_t[i] = (s_t[i]-MEANS[2])/(STDS[2]*2)
        
        # current dx intruder (from ownship frame of reference)
        for i in range(self.number_intruders_state*3, self.number_intruders_state*4):
            s_t[i] = (s_t[i]-MEANS[3])/(STDS[3]*2)
        
        # relative track with intruder
        for i in range(self.number_intruders_state*4, self.number_intruders_state*5):
            s_t[i] = (s_t[i])/(3.1415)

        if self.use_altitude:     
            # current speed
            s_t[self.number_intruders_state*6] = ((s_t[self.number_intruders_state*6]-min_speed)/(max_speed-min_speed))*2 - 1
            # optimal speed
            s_t[self.number_intruders_state*6 + 1] = ((s_t[self.number_intruders_state*6 + 1]-min_speed)/(max_speed-min_speed))*2 - 1

            # bearing to target
            s_t[self.number_intruders_state*6+3] = s_t[self.number_intruders_state*6+3]
            s_t[self.number_intruders_state*6+4] = s_t[self.number_intruders_state*6+4]
        else:
             # current speed
            s_t[self.number_intruders_state*5] = ((s_t[self.number_intruders_state*5]-min_speed)/(max_speed-min_speed))*2 - 1
            # optimal speed
            s_t[self.number_intruders_state*5 + 1] = ((s_t[self.number_intruders_state*5 + 1]-min_speed)/(max_speed-min_speed))*2 - 1
            
            # # bearing to target
            s_t[self.number_intruders_state*5+2] = s_t[self.number_intruders_state*5+2]
            s_t[self.number_intruders_state*5+3] = s_t[self.number_intruders_state*5+3]


        return s_t

class ReplayBuffer:
    def __init__(self, obs_dim: int, action_dim: int, size: int, batch_size: int = 32):
        self.obs_buf = np.zeros([size, obs_dim], dtype=np.float32)
        self.next_obs_buf = np.zeros([size, obs_dim], dtype=np.float32)
        self.rews_buf = np.zeros([size], dtype=np.float32)
        self.acts_buf = np.zeros([size, action_dim], dtype=np.float32)
        self.done_buf = np.zeros(size, dtype=np.float32)
        self.max_size, self.batch_size = size, batch_size
        self.ptr, self.size = 0, 0

    def store(
        self,
        obs: np.ndarray,
        act: np.ndarray,
        rew: float,
        next_obs: np.ndarray,
        done: bool,
    ) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, bool]:
        """ Store transition """
        self.obs_buf[self.ptr] = obs
        self.acts_buf[self.ptr] = act
        self.rews_buf[self.ptr] = rew
        self.next_obs_buf[self.ptr] = next_obs
        self.done_buf[self.ptr] = done
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample_batch(self) -> Dict[str, np.ndarray]:
        """ Sample from storage"""
        idx = np.random.choice(self.size, size=self.batch_size, replace=False)
        return dict(obs = self.obs_buf[idx],
            next_obs = self.next_obs_buf[idx],
            acts = self.acts_buf[idx],
            rews = self.rews_buf[idx],
            done = self.done_buf[idx])
    
    def __len__(self) -> int:
        return self.size
    
class Actor(nn.Module):
    def __init__(
        self,
        in_dim: int,
        out_dim: int,
        log_std_min: float= -20,
        log_std_max: float=2,
        hidden_dim1: int=256,
        hidden_dim2: int=256):
        super(Actor, self).__init__()

        self.log_std_min = log_std_min
        self.log_std_max = log_std_max

        self.hidden1 = nn.Linear(in_dim, hidden_dim1)
        self.hidden2 = nn.Linear(hidden_dim1, hidden_dim2)

        log_std_layer = nn.Linear(hidden_dim2, out_dim)
        self.log_std_layer = init_layer_uniform(log_std_layer)

        mu_layer = nn.Linear(hidden_dim2, out_dim)
        self.mu_layer = init_layer_uniform(mu_layer)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        x = F.relu(self.hidden1(state))
        x = F.relu(self.hidden2(x))

        mu =  self.mu_layer(x).tanh()

        log_std = self.log_std_layer(x).tanh()
        log_std = self.log_std_min  + 0.5 * (
            self.log_std_max - self.log_std_min
            ) * (log_std + 1)
        std = torch.exp(log_std)

        dist = Normal(mu, std)
        z = dist.rsample()

        action = z.tanh()

        log_prob = dist.log_prob(z) - torch.log(1 - action.pow(2) + 1e-7)
        log_prob = log_prob.sum(-1, keepdim=True)

        return action, log_prob


class CriticQ(nn.Module):
    def __init__(
        self,
        in_dim: int,
        hidden_dim1: int=256,
        hidden_dim2: int=256):
        super().__init__()

        self.hidden1 = nn.Linear(in_dim, hidden_dim1)
        self.hidden2 = nn.Linear(hidden_dim1, hidden_dim2)
        self.out = nn.Linear(hidden_dim1, 1)
        self.out = init_layer_uniform(self.out)

    def forward(
        self, 
        state:torch.Tensor, 
        action: torch.Tensor) -> torch.Tensor:
        x = torch.cat((state, action), dim=-1)
        x = F.relu(self.hidden1(x))
        x = F.relu(self.hidden2(x))
        value = self.out(x)

        return value

class CriticV(nn.Module):
    def __init__(
        self,
        in_dim: int,
        hidden_dim1: int=256,
        hidden_dim2: int=256):
        super().__init__()

        self.hidden1 = nn.Linear(in_dim, hidden_dim1)
        self.hidden2 = nn.Linear(hidden_dim1, hidden_dim2)
        self.out = nn.Linear(hidden_dim2, 1)
        self.out = init_layer_uniform(self.out)

    def forward(
        self, 
        state: torch.Tensor) -> torch.Tensor:
        x = F.relu(self.hidden1(state))
        x = F.relu(self.hidden2(x))
        value = self.out(x)

        return value
    
def init_layer_uniform(layer: nn.Linear, init_w: float = 3e-3) -> nn.Linear:
    layer.weight.data.uniform_(-init_w, init_w)
    layer.bias.data.uniform_(-init_w, init_w)

    return layer

    
class MedRL(Entity):
    def __init__(self):
        super().__init__()
        # Initialise stuff
        self.Agent = MaSacAgent(1, 1, STATE_SIZE)
        
        self.step_counter = 0
        
        with self.settrafarrays():
            self.state = [] # dr, dd, aL, aR, bd
            self.state_ = []
            self.action = []
            
        
        return
    
    def create(self, n=1):
        super().create(n)
        
        self.state[-n:] = [0]*STATE_SIZE
        self.state_[-n:] = [0]*STATE_SIZE
        self.action[-n:] = [0]*ACTION_SIZE
    
    @timed_function(dt=ML_DT)
    def step(self):
        
        # Get current state
        self.state = self.state_[0]
        
        self.state_ = self.get_state(0)
        
        reward, done = self.get_reward(0, self.state, self.state_)
        
        if done:
            self.reset()
            return
        
        if self.step_counter != 0:
            self.Agent.memory.store(self.state, self.action, reward, self.state_, done)
            
        self.action = self.Agent.do_step(self.state_)
        
        # Compute heading change
        heading_change = self.action * MAX_HEADING_CHANGE
        
        # Execute action
        bs.traf.ap.trk[0] = bs.traf.trk[0] + heading_change
        
        if self.step_counter % ML_STEPS == 0:
            self.Agent.update_model()
        
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
        dd, bd_abs = kwikqdrdist(ac_lat, ac_lon, AC_DESTINATION_LATLON[0], AC_DESTINATION_LATLON[1])
        
        bd = ((bd_abs - ac_hdg) + 180) % 360 - 180 
        
        return [dr, dd, a1, a2, bd]
        
    def get_reward(self, acidx, state, state_):
        ac_lat = bs.traf.lat[acidx]
        ac_lon = bs.traf.lon[acidx]
        done = False
        reward = 0
        # If distance to destination is less than 100m we are done
        if state[4] < 100:
            done = True
            reward += 1
        
        # Check if we hit the geofence
        bbox = Geofence.geo_by_name['AIGEO'].bbox
        if bbox[0] < ac_lat < bbox[2] and bbox[1] < ac_lon < bbox[3]
            done = True
            reward -= 2
            
        # 
        
        
        
        return reward, done
    
def create_scenario():
    ##### TUNING PARAMETERS #####
    # create a point where the center is at
    origin_lat = 0
    origin_lon = 0

    # set the width and depth of a rectangle meters
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
    stack(addwpt_command = f'ADDWPT AI01 {destination_df.geometry.y.values[0]} {destination_df.geometry.x.values[0]}')
