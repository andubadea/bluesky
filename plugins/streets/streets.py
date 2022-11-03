"""
Airspace edge information. Copies traffic, autopilot, route, activewaypoint
"""
# TODO: check if deleting routes/aircraft/waypoints works

from os import execlp, stat
import pickle
from bluesky.tools.misc import lat2txt
import json
import numpy as np
from numpy import *
from collections import Counter
import pandas as pd
from scipy.sparse import csr_matrix

from shapely.geometry import LineString, Point, MultiLineString, MultiPoint, GeometryCollection
from shapely.ops import nearest_points, split, transform, linemerge
import geopandas as gpd
from shapely.affinity  import affine_transform, scale, translate
from pyproj import Transformer



import bluesky as bs
from bluesky import core, stack, traf, scr, sim  #settings, navdb, tools
from bluesky.tools.aero import ft, kts, nm
from bluesky.tools import geo
from bluesky.tools.geo import kwikdist_matrix
from bluesky.core import Entity, Replaceable
from bluesky.traffic import Route
from bluesky.tools.misc import degto180, txt2tim, txt2alt, txt2spd
from bluesky.core.simtime import timed_function
from bluesky.core.varexplorer import access_plugin_object

import dill
import networkx as nx

from plugins.streets.flow_control import street_graph,bbox
from plugins.streets.agent_path_planning import *
from plugins.streets.open_airspace_grid import *
import time

path_plans = None
flight_layers = None
edge_traffic = None

def init_plugin():
    
    global path_plans, flight_layers, edge_traffic
    
    edge_traffic = EdgeTraffic()
    flight_layers = FlightLayers()
    path_plans = PathPlans()

    config = {
        # The name of your plugin
        'plugin_name'      : 'streets',
        'plugin_type'      : 'sim',
        'update':          update,
        'reset':           reset
        }
    
    return config

# defaults
use_path_plan = True
use_flow_control = True

# initialise queue
queue_dict = dict()

# initialise dill loading
dill_to_load = -1
angle_range = ''
heading_based_constrained = False

# TODO: 
#   - update CREM2 command for pre processed path planning
#   - density tracking per height? 

######################## UPDATE FUNCTION  ##########################

def update():
    global streets_bool

    if streets_bool:
        # Main update function for streets plugin. updates edge and flight layer tracking
        # Update edege autopilot
        edge_traffic.edgeap.update()

        # update layer tracking
        flight_layers.layer_tracking()

        #print(f'Edges took {round((time2 - time1)*1000, 4)} ms.')
        #print(f'Layers took {round((time3 - time2)*1000, 4)} ms.')
        # print(edge_traffic.actedge.wpedgeid[1])
        # print(edge_traffic.edgeap.edge_rou[0].iactwp)
        # print('-----------------------------------')
        # try:
        #     print(edge_traffic.edgeap.edge_rou[3].iactwp)
        #     print(bs.traf.ap.route[3].iactwp)
        #     print(edge_traffic.actedge.wpedgeid[3])
        #     iactwp = edge_traffic.edgeap.edge_rou[3].iactwp
        #     print(edge_traffic.edgeap.edge_rou[3].wpedgeid[iactwp+1])

        #     print('------------------------------------------------------')
        # except:
        #     pass

# =============================================================================
#     try:
#         i=bs.traf.id.index("D2")
#         print(len(bs.traf.ap.route[i].wpname))
#         print("############################################################")
#     except:
#         print("deleted")
# =============================================================================

####################### RESET FUNCTION ###############################
def reset():
    # when reseting bluesky turn off streets and turn on flow controls
    global streets_bool, use_flow_control
    streets_bool = False
    use_flow_control = True

    # default setting for streets is not constrained
    global heading_based_constrained

    # set hopping to true on the reset
    access_plugin_object('M2NAVIGATION').hopping = True
    access_plugin_object('SPEEDBASEDM2').hopping = True
    bs.traf.cr.heading_based = False
    
    heading_based_constrained = False

    # reset queue
    global queue_dict
    queue_dict = dict()

######################## FLOW CONTROL FUNCTIONS #########################
@core.timed_function(dt=10)
def do_flowcontrol():
    global use_path_plan
    # TODO: update the speed limit value of edges in (edge_traffic.edge_dict[wpedgeid]['speed_limit'])
    # probably need streeys_bool here as well
    if use_path_plan and use_flow_control:
        log_replan_changed_routes = np.zeros(bs.traf.ntraf)
        log_replan_same_route = np.zeros(bs.traf.ntraf)
        log_no_replan_but_changes = np.zeros(bs.traf.ntraf)
        log_no_replan_high_traffic = np.zeros(bs.traf.ntraf)
        log_no_replan_last_point = np.zeros(bs.traf.ntraf)
        log_no_replan_open_airspace = np.zeros(bs.traf.ntraf)
        #print("flow control")

        # go through any deleted loitering geofences
        for loiter_acid in bs.traf.loiter.deleted_loiters:
            # get list of edges and send to delete_loitering_flowcontrol
            delete_loitering_flowcontrol(path_plans.loitering_edges_dict[loiter_acid])

        # clear deleted loitering geofences
        bs.traf.loiter.deleted_loiters = []

        edge_count_dict = dict(Counter(edge_traffic.actedge.wpedgeid)) #only contains edges with aircrafts
        group_count_dict = dict(Counter(edge_traffic.actedge.group_number)) 
        flow_count_dict = dict(Counter(edge_traffic.actedge.flow_number))

        if 0 in flow_count_dict.keys():
            del flow_count_dict[0]

# =============================================================================
#         if -1 in group_count_dict.keys():
#             del group_count_dict[-1]
#         if 1578 in group_count_dict.keys():
#             del group_count_dict[1578]
#         if 1577 in group_count_dict.keys():
#             del group_count_dict[1577]
#         if 1576 in group_count_dict.keys():
#             del group_count_dict[1576]
#         if 2000 in group_count_dict.keys():
#             del group_count_dict[2000]
# =============================================================================

        #print(flow_count_dict)
        ###########
        #TODO: vectorize with numpy
        #########changes##########
        edges_changes=[]
        path_plans.graph.edges_previous_speed=copy.deepcopy(path_plans.graph.edges_current_speed)
        
        for key_str in path_plans.graph.flows_lengths.keys():
            #key_str=str(key)
            key=int(key_str)

            if key in flow_count_dict.keys():
                dens=flow_count_dict[key]/path_plans.graph.flows_lengths[key_str]
                #print(path_plans.graph.flows_lengths[key_str])
            else: 
                dens=0
            
            #key=str(key)
            ## assume that allowed density is 1 drone every 20 meters, maybe 40 meters?
            if dens >0.025:# high traffic

                if path_plans.graph.modified_group[key]==2:
                    continue
                if path_plans.graph.modified_group[key]==1:
                    path_plans.graph.medium_traffic_groups.remove(key)
                path_plans.graph.modified_group[key]=2
                path_plans.graph.high_traffic_groups.append(key)
                for edge in path_plans.graph.flows_dict[key_str]:
                    
                    k = int(edge.split("-")[0])
                    kk = int(edge.split("-")[1])
                    if (k,kk) in path_plans.graph.loiter_nfz_edges:
                        continue
                    path_plans.graph.edges_current_speed[k][kk]=0.1 # not zero because it could trap an aircraft
                    tmp=[k,kk,path_plans.graph.edges_current_speed[k][kk]]#the keys of the vertices of the edges, followed by the new speed
                    edges_changes.append(tmp)
                    #print('Applying a high traffic speed limit!!!!!!!!!!!!!')
                    edge_traffic.edge_dict[edge]['speed_limit'] = 15
                    
            elif dens >0.005:#medium traffic

                if path_plans.graph.modified_group[key]==1:
                    continue
                if path_plans.graph.modified_group[key]==2:
                    path_plans.graph.high_traffic_groups.remove(key)
                path_plans.graph.modified_group[key]=1
                path_plans.graph.medium_traffic_groups.append(key)
                for edge in path_plans.graph.flows_dict[key_str]:
                    k = int(edge.split("-")[0])
                    kk = int(edge.split("-")[1])
                    if (k,kk) in path_plans.graph.loiter_nfz_edges:
                        continue
                    path_plans.graph.edges_current_speed[k][kk]=path_plans.graph.edges_graph[k][kk].max_speed/2
                    tmp=[k,kk,path_plans.graph.edges_current_speed[k][kk]]#the keys of the vertices of the edges, followed by the new speed
                    edges_changes.append(tmp)
                    edge_traffic.edge_dict[edge]['speed_limit'] = 30
 
                    
            else:# low traffic
                if path_plans.graph.modified_group[key]==0:
                    continue
                if path_plans.graph.modified_group[key]==2:
                    path_plans.graph.high_traffic_groups.remove(key)
                if path_plans.graph.modified_group[key]==1:
                    path_plans.graph.medium_traffic_groups.remove(key)
                path_plans.graph.modified_group[key]=0
                for edge in path_plans.graph.flows_dict[key_str]:

                    k = int(edge.split("-")[0])
                    kk = int(edge.split("-")[1])
                    if (k,kk) in path_plans.graph.loiter_nfz_edges:
                        continue
                    path_plans.graph.edges_current_speed[k][kk]=path_plans.graph.edges_graph[k][kk].max_speed
                    tmp=[k,kk,path_plans.graph.edges_current_speed[k][kk]]#the keys of the vertices of the edges, followed by the new speed
                    edges_changes.append(tmp)
                    edge_traffic.edge_dict[edge]['speed_limit'] = 30

        if edges_changes!=[]:
            log_replan_changed_routes, \
            log_replan_same_route, \
            log_no_replan_but_changes, \
            log_no_replan_high_traffic, \
            log_no_replan_last_point, \
            log_no_replan_open_airspace = handle_replan(edges_changes)

        bs.traf.flowlog.log(*bs.traf.id)
        bs.traf.flowlog.log(*bs.traf.alt/ft)
        bs.traf.flowlog.log(*bs.traf.lat)
        bs.traf.flowlog.log(*bs.traf.lon)
        bs.traf.flowlog.log(*bs.traf.actedge.wpedgeid)
        bs.traf.flowlog.log(*bs.traf.actedge.edge_airspace_type)

        bs.traf.flowlog.log(*log_replan_changed_routes)
        bs.traf.flowlog.log(*log_replan_same_route)
        bs.traf.flowlog.log(*log_no_replan_but_changes)
        bs.traf.flowlog.log(*log_no_replan_high_traffic)
        bs.traf.flowlog.log(*log_no_replan_last_point)
        bs.traf.flowlog.log(*log_no_replan_open_airspace)


## Call function when new loitering geofence is applied
def apply_loitering_flowcontrol(loitering_edges):

    if bs.traf.ntraf == 0:
        return
    
    if use_path_plan and use_flow_control:
        #print("loitering_flow_control")
        edges_changes=[]
        path_plans.graph.edges_previous_speed=copy.deepcopy(path_plans.graph.edges_current_speed)
        ##Loitering missions changes
        for edge in loitering_edges:
            k = edge[0]
            kk = edge[1]
            key_str = str(k)+"-"+str(kk)
            if edge in path_plans.graph.loiter_nfz_edges:
                continue
            path_plans.graph.loiter_nfz_edges.append(edge)
            path_plans.graph.edges_current_speed[k][kk]=0
            tmp=[k,kk,path_plans.graph.edges_current_speed[k][kk]]#the keys of the vertices of the edges, followed by the new speed
            edges_changes.append(tmp)
            edge_traffic.edge_dict[key_str]['speed_limit'] = 30
            
        if edges_changes!=[]:
            handle_replan(edges_changes)
            
## Call function when loitering geofence is lifted
def delete_loitering_flowcontrol(lifted_loitering_edges):
    if use_path_plan and use_flow_control:
        #print("delete loitering_flow_control")
        edges_changes=[]
        path_plans.graph.edges_previous_speed=copy.deepcopy(path_plans.graph.edges_current_speed)
        ##Loitering missions changes
        for edge in lifted_loitering_edges:
            k = edge[0]
            kk = edge[1]
            key_str = str(k)+"-"+str(kk)
            if edge not in path_plans.graph.loiter_nfz_edges:
                continue
            path_plans.graph.loiter_nfz_edges.remove(edge)
            path_plans.graph.edges_current_speed[k][kk]=path_plans.graph.edges_graph[k][kk].max_speed
            tmp=[k,kk,path_plans.graph.edges_current_speed[k][kk]]#the keys of the vertices of the edges, followed by the new speed
            edges_changes.append(tmp)
            edge_traffic.edge_dict[key_str]['speed_limit'] = 30

            
        if edges_changes!=[]:
            handle_replan(edges_changes)         
            
def handle_replan(edges_changes):

    log_replan_changed_routes = np.zeros(bs.traf.ntraf)
    log_replan_same_route = np.zeros(bs.traf.ntraf)
    log_no_replan_but_changes = np.zeros(bs.traf.ntraf)
    log_no_replan_high_traffic = np.zeros(bs.traf.ntraf)
    log_no_replan_last_point = np.zeros(bs.traf.ntraf)
    log_no_replan_open_airspace = np.zeros(bs.traf.ntraf)

    for idx,path in enumerate(path_plans.pathplanning):
        #print("index")
        #print(idx)
        acid = bs.traf.id[idx]

        if acid[0] == 'R':
            continue

        #print(acid)
        ##Get the current position
        lat=traf.lat[idx]
        lon=traf.lon[idx]
        
        ##Get the index of the next node
        current_edge=edge_traffic.actedge.wpedgeid[idx]
        
        next_node_osmnx_id=int(current_edge.split("-")[1])
        prev_node_osmnx_id=int(current_edge.split("-")[0])
        
        if next_node_osmnx_id==path.goal_index or next_node_osmnx_id==path.goal_index_next: 
            continue
        
        else:
            #print(next_node_osmnx_id,prev_node_osmnx_id,lat,lon)
            route,turns,edges,next_turn,groups,in_constrained,turn_speeds,replantype=path.replan(edges_changes,prev_node_osmnx_id,next_node_osmnx_id,lat,lon)
            if len(route)>0:                    

                acrte = Route._routes.get(acid)

                # get old lats, lons of route
                oldlats = {acrte.wplat}
                oldlons = {acrte.wplon}

                # If replantype is 0 this emeans that overall graph was updated but this aircraft
                # was not affected
                if replantype == 0:
                    log_no_replan_but_changes[idx] = 1
                
                # if replantype is 1 this means that the aircraft replanned
                # still unsure if it is the same route or previous route.
                elif replantype == 1:
                    newlats = {coord[1] for coord in route}
                    newlons = {coord[0] for coord in route}

                    # now check if it is same route
                    if newlats.issubset(oldlats) and newlons.issubset(oldlons):
                        log_replan_same_route[idx] = 1
                    else:
                        log_replan_changed_routes[idx] = 1

                # if replantype is 2 it means it did not replan because of high traffic
                # and aircraft has low or medium priority
                elif replantype == 2:
                    log_no_replan_high_traffic[idx] = 1
                
                # if replantype is 3 it means it did not replan because it is near the last point of route
                elif replantype == 3:
                    log_no_replan_last_point[idx] = 1

                # if replantype is 4 it means that aircraft has route completely in open airspace
                elif replantype == 4:
                    log_no_replan_open_airspace[idx] = 1

                # If the next waypoint is a turn waypoint, then remember the turnrad
                nextqdr_to_remember = bs.traf.actwp.next_qdr[idx]
                    
                edge_traffic.edgeap.update_route(idx)

                utm_x = []
                utm_y = []
                for j, rte in enumerate(route):
                    lat = rte[1] # deg
                    lon = rte[0] # deg

                    # convert to utm #TODO: check x y in correct order
                    x,y = path_plans.transformer_to_utm.transform(lat, lon)
                    utm_x.append(x)
                    utm_y.append(y)

                    alt = -999
                    spd = -999
                    
                    # Do flyby or flyturn processing
                    if turns[j] and j < len(route)-1:
                        acrte.turnspd = turn_speeds[j]*kts
                        acrte.swflyby   = False
                        acrte.swflyturn = True
                    else:
                        # Either it's a flyby, or a typo.
                        acrte.swflyby   = True
                        acrte.swflyturn = False
                    
                    name    = acid
                    wptype  = Route.wplatlon
                    
                    wpidx = acrte.addwpt_simple(idx, name, wptype, lat, lon, alt, spd)
                
                    # Add the streets stuff
                    # get group number
                    group_number = groups[j]
                    
                    wpedgeid = f'{edges[j][0]}-{edges[j][1]}'
                    
                    edge_layer_type = edge_traffic.edge_dict[wpedgeid]['height_allocation']
                    edge_layer_dict = flight_layers.layer_dict["config"][edge_layer_type]['levels']

                    if edge_layer_type != 'open' and heading_based_constrained:
                        # Get the layer dictionary for the heading range
                        edge_layer_dict = edge_layer_dict[flight_layers.constrained_airspace_alloc[idx]]

                    flow_number = edge_traffic.edge_dict[wpedgeid]['flow_group']

                    # get the edge_airspace_type
                    if in_constrained[j]:
                        edge_airspace_type = 'constrained'
                    else:
                        edge_airspace_type = 'open'
                    turn_lat = next_turn[j][1]
                    turn_lon = next_turn[j][0]
                    
                    wpidxstreets = edge_traffic.edgeap.edge_rou[idx].addwpt(idx, name, wpedgeid, group_number, flow_number, edge_layer_dict, 
                                                turn_lat, turn_lon, edge_airspace_type)
                # Calculate flight plan
                acrte.calcfp()
                edge_traffic.edgeap.edge_rou[idx].direct(idx,edge_traffic.edgeap.edge_rou[idx].wpname[1])
                stack.stack(f'LNAV {acid} ON')
                stack.stack(f'VNAV {acid} ON')
                
                # # Add the turndist back
                if not bs.traf.actwp.flyby[idx] and bs.traf.actwp.flyturn[idx]:
                    bs.traf.actwp.next_qdr[idx] = nextqdr_to_remember

                # add utm route points to path_plans
                path_plans.route_coords[idx] = list(zip(utm_x, utm_y))

                # add route as linestring for projected based on UTM coordinates
                original_route = LineString(list(zip(utm_x, utm_y))).simplify(0.0001)

                #assert that the route_merged is a linestring
                path_plans.lineintersects[idx] = not original_route.is_simple
                
                # update the route
                path_plans.lineroutes[idx] = original_route

    return  log_replan_changed_routes, log_replan_same_route, log_no_replan_but_changes, \
            log_no_replan_high_traffic, log_no_replan_last_point, log_no_replan_open_airspace



######################## STACK COMMANDS ##########################  
@stack.command
def queuem2(acid, actype: str="B744", path_file: str="", aclat: float=52., aclon: float=4., destlat: float = 51., destlon: float = 3., achdg: float=None, acalt: float=0,  
        acspd: float = 0, prio: int = 1,  geodur:float = 0, *geocoords:float):
    # Queues up an aircraft, and checks if it can be spawned without provoking a conflict.
    # First, correct some values.
    acspd *= kts
    acalt *= ft

    # always set geodur = 0
    geodur = 0.0
    
    # Attempt to create
    queue_attempt_create(bs.sim.simt, acid, actype, path_file, aclat, aclon, destlat, destlon, achdg, acalt, acspd, prio, geodur, geocoords)

    # add path plan for specific aircraft
    #bs.traf.path_plans[-1] = path_plan_dict[ACID]

    return

@stack.command
def addwptm2(acid: 'acid', lat: float, lon: float, alt: float = -999, spd: float = -999, wpedgeid: 'txt'="",  
            group_number: 'txt' = "", action_lat: float=48.1351, action_lon: float = 11.58, airspace_type: 'txt' ="0"):
    """ADDWPTM2 acid, (lat,lon),[alt],[spd],[edgeid],[turn_node]"""
    # DEPRECATED
    # edgeid comes from graph
    # M2 wpt command
    # corrected arguments
    
    latlon = f'{lat},{lon}'
    if spd != -999:
        spd *= kts
    if alt != -999:
        alt *= ft

    # get group number
    group_number = edge_traffic.edge_dict[wpedgeid]['stroke_group']

    # get flow number
    flow_number = edge_traffic.edge_dict[wpedgeid]['flow_group']
    
    # get layer type
    edge_layer_type = edge_traffic.edge_dict[wpedgeid]['height_allocation']

    # dictionary of layers
    edge_layer_dict = flight_layers.layer_dict["config"][edge_layer_type]['levels']

    # get the edge_airspace_type
    if airspace_type=="1":
        edge_airspace_type = 'constrained'
    else:
        edge_airspace_type = 'open'

    # add edge info to stack
    edge_traffic.edgeap.edge_rou[acid].addwptedgeStack(acid, latlon, alt, spd, wpedgeid, group_number, flow_number, 
                                                       edge_layer_dict, action_lat, action_lon, edge_airspace_type)

@stack.command
def CREM2(acid, actype: str="B744", aclat: float=52., aclon: float=4., achdg: float=None, acalt: float=0,  
        acspd: float = 0, prio: int = 1, geodur:float = 0, *geocoords:float):
    """CREM2 acid, type, [latlon], [hdg], [alt], [spd], prio"""
    ## DEPRECATED!!!
    # Creates an aircrft, but also assigns priority
    # Convert stuff for bs.traf.cre

    # correct some argument units
    acspd *= kts
    acalt *= ft
        
    # First create the aircraft
    bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)

    acidx = bs.traf.id.index(acid)
    if geodur!=0:
        # It's a loitering mission, so add the loitering stuff
        bs.traf.loiter.futuregeofences[acidx] = geocoords
        bs.traf.loiter.geodurations[acidx] = geodur
        bs.traf.loiter.loiterbool[acidx] = True
        
        # send to flow contorl
        apply_loitering_flowcontrol(path_plans.loitering_edges_dict[acid])
        
    # print(prio)
    # Then assign its priority
    idx = bs.traf.id.index(acid)
    bs.traf.priority[idx] = prio

    # add path plan for specific aircraft
    #bs.traf.path_plans[-1] = path_plan_dict[ACID]

    return

@stack.command
def edgeid(acid: 'txt'):
    """EDGEID, acid"""
    # check edge id of aircraft
    idx = traf.id2idx(acid)
    bs.scr.echo(f'{acid} flying over {edge_traffic.actedge.wpedgeid[idx]}')

@stack.command
def dis2int(acid: 'txt'):
    """dis2int acid"""
    # distance to next intersection
    idx = traf.id2idx(acid)

    current_edge = edge_traffic.actedge.wpedgeid[idx]

    node_id = int(current_edge.split("-",1)[1])

    node_lat, node_lon = osmid_to_latlon(current_edge, 1)

    _, d = geo.qdrdist(traf.lat[idx], traf.lon[idx], node_lat, node_lon)
    
    bs.scr.echo(f'{acid} is {d*nm} meters from node {node_id}')

@stack.command
def loadpath(filename: 'txt'):
    """LOADPATH, filename"""
    global use_path_plan
    # Initialize Path Plans
    file_path = f'plugins/streets/path_plans/{filename}.dill'

    if use_path_plan:
        path_plans.load_flow_dill(file_path)
    
@stack.command
def DELRTEM2(acidx: 'acid' = None):

    # add edge info to stack
    edge_traffic.edgeap.update_route(acidx)
    return True

@stack.command
def streetsenable():
    """streetsenable"""
    # # Turns on streets for scenario
    global streets_bool

    streets_bool = True

@stack.command
def headingconstrained():
    """headingconstrained"""
    # # Turns on heading constrained airspace for scenario
    global heading_based_constrained, nav

    heading_based_constrained = True

    # set M2 Navigation hopping to False
    access_plugin_object('M2NAVIGATION').hopping = False
    access_plugin_object('SPEEDBASEDM2').hopping = False

@stack.command
def loadloiteringdill(fpath: str):
    """
    Load loitering edges dictionary from a dill file.
    """
    loitering_fpath = f'plugins/streets/scenario_loitering_dills/{fpath}'  
    
    path_plans.load(loitering_fpath)

@stack.command
def airspaceinfo(fpath: str):
    """
    Load the layer edges edges, nodes and structure into edge_traffic and flight layers.
    Must be inside plugins/streets/ folders
    """
    # graph_data/03-graph/layers.json
    fpath = f'plugins/streets/{fpath}'  
    
    edge_traffic.load(fpath)
    flight_layers.load(fpath)
    path_plans.load_flow_dill(fpath)

@stack.command
def DISABLEFLOWCONTROL():
    """DISABLEFLOWCONTROL"""
    # # Turns off flow control for scenario
    global use_flow_control

    use_flow_control = False
    
######################## QUEUE ###############################
def queue_attempt_create(first_time, acid, actype, path_file, aclat, aclon, destlat, destlon, achdg, acalt, acspd, prio, geodur, geocoords = None):
    # Easiest check, if any aircraft is below 30 ft
    alt_not_ok = np.any(bs.traf.alt<40*ft)
    
    global dill_to_load, angle_range

    if not alt_not_ok:
        # First, set the global DILL loading variable
        dill_to_load = path_file

        if heading_based_constrained:
            # assign the flight layer allocation in constrained airspace
            # step 1: calculate the heading from origin to destination
            qdr_full, _ = geo.qdrdist(aclat, aclon, destlat, destlon)
            qdr_full = qdr_full % 360

            # step 2: check between which heading range the aircraft is
            # TODO: make this dynamic
            heading_ranges_constrained = np.array([0,72,144,216,288,360])

            # check which two values qdr is in between
            idx_qdr = np.where(qdr_full<heading_ranges_constrained)[0]

            # select the idx and the one before
            angle_range = f'{heading_ranges_constrained[idx_qdr-1][0]}-{heading_ranges_constrained[idx_qdr][0]}'

        # Then create the aircraft
        bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)

        acidx = bs.traf.id.index(acid)
        if geodur!=0:
            # It's a loitering mission, so add the loitering stuff
            bs.traf.loiter.futuregeofences[acidx] = geocoords
            bs.traf.loiter.geodurations[acidx] = geodur
            bs.traf.loiter.loiterbool[acidx] = True

            # send to flow contorl
            apply_loitering_flowcontrol(path_plans.loitering_edges_dict[acid])
            
        # print(prio)
        # Then assign its priority
        idx = bs.traf.id.index(acid)
        bs.traf.priority[idx] = prio
        
        # Add the necessary stack commands for this aircraft
        stack.stack(f'LNAV {acid} ON')
        stack.stack(f'VNAV {acid} ON')
        return True
        
    #We reach this point if there are aircraft below 30 ft. There can't be THAT many, so we
    # can extract their coords and see what the distance is
    lats = bs.traf.lat[alt_not_ok]
    lons = bs.traf.lon[alt_not_ok]
    
    dist = kwikdist_matrix(np.array([aclat]), np.array([aclon]), lats, lons)
    
    # Second check, if distance is smaller than rpz * 4?
    dist_not_ok = np.any(dist<(bs.settings.asas_pzr*2))
    
    if not dist_not_ok:
        # First create the aircraft
        dill_to_load = path_file

        if heading_based_constrained:
            # assign the flight layer allocation in constrained airspace
            # step 1: calculate the heading from origin to destination
            qdr_full, _ = geo.qdrdist(aclat, aclon, destlat, destlon)
            qdr_full = qdr_full % 360

            # step 2: check between which heading range the aircraft is
            # TODO: make this dynamic
            heading_ranges_constrained = np.array([0,72,144,216,288,360])

            # check which two values qdr is in between
            idx_qdr = np.where(qdr_full<heading_ranges_constrained)[0]

            # select the idx and the one before
            angle_range = f'{heading_ranges_constrained[idx_qdr-1][0]}-{heading_ranges_constrained[idx_qdr][0]}'
            
        bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)

        acidx = bs.traf.id.index(acid)
        if geodur!=0:
            # It's a loitering mission, so add the loitering stuff
            bs.traf.loiter.futuregeofences[acidx] = geocoords
            bs.traf.loiter.geodurations[acidx] = geodur
            bs.traf.loiter.loiterbool[acidx] = True

            # send to flow contorl
            apply_loitering_flowcontrol(path_plans.loitering_edges_dict[acid])
            
        # print(prio)
        # Then assign its priority
        idx = bs.traf.id.index(acid)
        bs.traf.priority[idx] = prio
        
        # Add the necessary stack commands for this aircraft
        stack.stack(f'LNAV {acid} ON')
        stack.stack(f'VNAV {acid} ON')
        return True
    
    # Spawn aircraft if first time is greater than 5 minutes 300 seconds
    if bs.sim.simt - first_time > 300:
        # First create the aircraft
        dill_to_load = path_file

        if heading_based_constrained:
            # assign the flight layer allocation in constrained airspace
            # step 1: calculate the heading from origin to destination
            qdr_full, _ = geo.qdrdist(aclat, aclon, destlat, destlon)
            qdr_full = qdr_full % 360

            # step 2: check between which heading range the aircraft is
            # TODO: make this dynamic
            heading_ranges_constrained = np.array([0,72,144,216,288,360])

            # check which two values qdr is in between
            idx_qdr = np.where(qdr_full<heading_ranges_constrained)[0]

            # select the idx and the one before
            angle_range = f'{heading_ranges_constrained[idx_qdr-1][0]}-{heading_ranges_constrained[idx_qdr][0]}'
        
        bs.traf.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)

        acidx = bs.traf.id.index(acid)
        if geodur!=0:
            # It's a loitering mission, so add the loitering stuff
            bs.traf.loiter.futuregeofences[acidx] = geocoords
            bs.traf.loiter.geodurations[acidx] = geodur
            bs.traf.loiter.loiterbool[acidx] = True

            # send to flow contorl
            apply_loitering_flowcontrol(path_plans.loitering_edges_dict[acid])
            
        # print(prio)
        # Then assign its priority
        idx = bs.traf.id.index(acid)
        bs.traf.priority[idx] = prio

        # Add the necessary stack commands for this aircraft
        stack.stack(f'LNAV {acid} ON')
        stack.stack(f'VNAV {acid} ON')
        return True
        
        
    # All attempts to create failed, so we add to queue

    # Add all the arguements
    queue_dict[acid] = [first_time, acid,actype,path_file,aclat, aclon, destlat, destlon, achdg, acalt, acspd, prio, geodur, geocoords]
    return False

# Attempt to spawn queued aircraft once every 5 seconds
@timed_function(dt = 5)
def check_queue():
    # For loop inevitable
    # First, create a copy of the dictionary
    temp = queue_dict.copy()
    for acid in temp:
        #Attempt to create
        if queue_attempt_create(*temp[acid]):
            # We're succesful, delete this entry
            queue_dict.pop(acid)
            # If unsuccessful, then it will attempt again at next iteration.

######################## WAYPOINT TRAFFIC TRACKING  ##########################

# "traffic" class. Contains edge "autopilot" and "activedge"
class EdgeTraffic(Entity):

    def __init__(self):
        super().__init__()

        with self.settrafarrays():

            self.edgeap   = EdgesAp()
            self.actedge  = ActiveEdge()
            self.edge_routes = np.array([], dtype=LineString)

        # make variables available in bs.traf
        bs.traf.edgeap = self.edgeap
        bs.traf.actedge = self.actedge

        # initialize edge and nodes dictionaries and arrays
        self.edge_dict = {}
        self.node_dict = {}

        self.edge_to_uv_array = np.array([], dtype='uint16,uint16')
        self.const_edge_stroke_array = np.array([], dtype='uint16')
        self.const_edge_flow_array = np.array([], dtype='uint16')
        self.const_edge_height_allocation_array = np.array([], dtype='uint8')       
        self.const_edge_speed_limit_array = np.array([], dtype='uint16')

        self.const_edge_id_array = np.array([], dtype='uint16')
        self.uv_to_edge_matrix = csr_matrix(np.array([]), dtype=np.uint16)

    def load(self, dict_file_path):
        # load the edge and node info from the stack command

        # Opening edges.JSON as a dictionary
        with open(f'{dict_file_path}/edges.json', 'r') as filename:
            self.edge_dict = json.load(filename)

        # Opening nodes.JSON as a dictionary
        with open(f'{dict_file_path}/nodes.json', 'r') as filename:
            node_dict = json.load(filename)

        # reverse dictionary
        self.node_dict = {v: k for k, v in node_dict.items()}

        # TODO: could comment everything below this line if we don't use it
        # read constrained node dict
        with open(f'{dict_file_path}/constrained_node_dict.json', 'r') as filename:
            constrained_node_dict = json.load(filename)

        # open constrained edges JSON as a dictionary
        with open(f'{dict_file_path}/const_edges.json', 'r') as filename:
            constrained_edges_dict = json.load(filename)

        # Build mapping from node to edges
        edge_array_df = pd.DataFrame(constrained_edges_dict).T
        self.edge_to_uv_array = np.array([(x.split('-')[0], x.split('-')[1]) for x in list(edge_array_df.index)], dtype='uint16,uint16')
        self.const_edge_stroke_array = np.array(edge_array_df['stroke_group']).astype(np.int16)
        self.const_edge_flow_array = np.array(edge_array_df['flow_group']).astype(np.int16)
        self.const_edge_height_allocation_array = np.array(edge_array_df['height_allocation']).astype(np.int8)        
        self.const_edge_speed_limit_array = np.array(edge_array_df['speed_limit']).astype(np.int16)

        # build the edge_id_array
        self.const_edge_id_array = np.arange(len(self.edge_to_uv_array), dtype='uint16')

        # intialize an empty matrix that is the len of nodes
        adj_matrix = np.zeros((len(constrained_node_dict), len(constrained_node_dict)), dtype='uint16')

        # populate this matrix (row=node_id, col=node_id) and value at row, col is edge_id
        for j, ids in enumerate(self.edge_to_uv_array):
            adj_matrix[ids[0], ids[1]] = j
        
        # convert to sparse matrix to get the (u,v) to edge_id mapping
        self.uv_to_edge_matrix = csr_matrix(adj_matrix, dtype=np.uint16)

# "autopilot"
class EdgesAp(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.edge_rou = []
        
        # make a vectorized function to check speed limits
        self.update_speed_limits = np.vectorize(self.check_speed_limits)
    
    def create(self, n=1):
        super().create(n)

        for ridx, acid in enumerate(bs.traf.id[-n:]):
            self.edge_rou[ridx - n] = Route_edge(acid)
    
    def update_route(self, idx):

        acid = traf.id[idx]

        self.edge_rou[idx] = Route_edge(acid)

        traf.ap.route[idx].delrte(idx)
        
    def update(self):
        
        # Main autopilot update loop

        # See if waypoints have reached their destinations
        for i in np.where(bs.traf.ap.reached)[0]:

            # Skip this if aircraft is rogue aircraft
            if bs.traf.roguetraffic.rogue_bool[i]:
                continue

            edge_traffic.actedge.wpedgeid[i], \
            edge_traffic.actedge.intersection_lat[i] , edge_traffic.actedge.intersection_lon[i], \
            edge_traffic.actedge.group_number[i], edge_traffic.actedge.flow_number[i], \
            edge_traffic.actedge.edge_layer_dict[i], \
            edge_traffic.actedge.turn_lat[i], edge_traffic.actedge.turn_lon[i], \
            edge_traffic.actedge.hdg_lat[i], edge_traffic.actedge.hdg_lon[i], \
            edge_traffic.actedge.const_lat[i], edge_traffic.actedge.const_lon[i], \
            edge_traffic.actedge.edge_airspace_type[i] = self.edge_rou[i].getnextwp()      

        # TODO: only calculate for drones that are in constrained airspace
        # get distance of drones to next intersection/turn intersection
        dis_to_int = np.where(bs.traf.roguetraffic.rogue_bool, 9999.9, 
                                geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.intersection_lat, 
                                                    edge_traffic.actedge.intersection_lon))
        dis_to_turn = np.where(bs.traf.roguetraffic.rogue_bool, 9999.9, 
                                geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.turn_lat, 
                                                    edge_traffic.actedge.turn_lon))
        # TODO: only calculate for drones that are in open airspace
        # get distances of drones to constrained airspace and to next heading change that requires an altitude change
        dis_to_const = np.where(bs.traf.roguetraffic.rogue_bool, 9999.9, 
                                geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.const_lat, 
                                                    edge_traffic.actedge.const_lon))
        dis_to_hdg = np.where(bs.traf.roguetraffic.rogue_bool, 9999.9, 
                                geo.kwikdist_matrix(traf.lat, traf.lon, 
                                                    edge_traffic.actedge.hdg_lat, 
                                                    edge_traffic.actedge.hdg_lon))
        
        # # check for speed limit changes

        if bs.traf.ntraf > 0:
            edge_traffic.actedge.speed_limit = self.update_speed_limits(edge_traffic.actedge.wpedgeid, 
                                                                        bs.traf.type, 
                                                                        bs.traf.roguetraffic.rogue_bool)

        # flatten numpy arrays
        edge_traffic.actedge.dis_to_int = np.asarray(dis_to_int).flatten()
        edge_traffic.actedge.dis_to_turn = np.asarray(dis_to_turn).flatten()
        edge_traffic.actedge.dis_to_const = np.asarray(dis_to_const).flatten()
        edge_traffic.actedge.dis_to_hdg = np.asarray(dis_to_hdg).flatten()

        # update variables available in bs.traf
        bs.traf.edgeap = edge_traffic.edgeap
        bs.traf.actedge = edge_traffic.actedge

        return

    @staticmethod
    def check_speed_limits(wpedgeid, ac_type, rogue_bool):

        # check if aircraft is rogue, if yes just give it a 30 knot speed limit
        if rogue_bool:
            return 30*kts

        # check for speed limit changes
        speed_limit = int(edge_traffic.edge_dict[wpedgeid]['speed_limit'])

        # get aircraft cruise speed from last two values of ac_type
        cruise_speed = int(ac_type[-2:])

        # choose the minimum of the two for the new speed limit
        speed_limit = min(speed_limit, cruise_speed)*kts

        return speed_limit


# active edge class "the active waypoint"
class ActiveEdge(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            # TODO: choose correct dtypes for optimization
            self.wpedgeid = np.array([], dtype="S22")
            self.nextwpedgeid = np.array([], dtype=str)
            self.nextturnnode = np.array([], dtype=str)

            # cosntraint airspace information
            self.intersection_lat = np.array([])
            self.intersection_lon = np.array([])

            self.turn_lat = np.array([])
            self.turn_lon = np.array([])

            # open airspace information
            self.const_lat = np.array([])
            self.const_lon = np.array([])

            self.hdg_lat = np.array([])
            self.hdg_lon = np.array([])

            # Distances to next intersection/turn intersection

            self.dis_to_int = np.array([])
            self.dis_to_turn = np.array([])

            self.dis_to_hdg = np.array([])
            self.dis_to_const = np.array([])

            self.group_number = np.array([], dtype=int)
            self.flow_number = np.array([], dtype=int)

            # Open to optimization!
            self.edge_layer_dict = np.array([], dtype=object)

            self.edge_airspace_type = np.array([], dtype=str)

            # speed limit
            self.speed_limit = np.array([], dtype=np.int32)

    
    def create(self, n=1):
        super().create(n)

        self.wpedgeid[-n:]                  = ""
        self.nextwpedgeid[-n:]              = ""
        self.nextturnnode[-n:]              = ""

        self.intersection_lat[-n:]          = 89.99
        self.intersection_lon[-n:]          = 89.99

        self.turn_lat[-n:]                  = 89.99
        self.turn_lon[-n:]                  = 89.99

        self.const_lat[-n:]                 = 89.99
        self.const_lon[-n:]                 = 89.99

        self.hdg_lat[-n:]                   = 89.99
        self.hdg_lon[-n:]                   = 89.99

        self.dis_to_int[-n:]                = 9999.9
        self.dis_to_turn[-n:]               = 9999.9

        self.dis_to_hdg[-n:]                = 9999.9
        self.dis_to_const[-n:]              = 9999.9

        self.group_number[-n:]              = 999
        self.flow_number[-n:]               = 999
        self.edge_layer_dict[-n:]           = {}

        self.edge_airspace_type[-n:]        = ""

        self.speed_limit[-n:]               = 999

# route_edge class. keeps track of when aircraft move to new edges and adds edges to stack
class Route_edge(Replaceable):

    def __init__(self, acid):
        # Aircraft id (callsign) of the aircraft to which this route belongs
        self.acid = acid
        self.nwp = 0

        # Waypoint data
        self.wpname = []

        # Current actual waypoint
        self.iactwp = -1
   
        # initialize edge id list. osmids of edge
        self.wpedgeid = []

        # initialize location of turn in constrained airspae
        self.turn_lat = []
        self.turn_lon = []

        # initialize location of heading change in open airspae
        self.hdg_lat = []
        self.hdg_lon = []

        # initialize group_number and flow_number
        self.group_number = []
        self.flow_number = []

        # initialize edge_layer_dict
        self.edge_layer_dict = []

        # initialize the airspace type
        self.edge_airspace_type = []

    def addwptedgeStack(self, idx, latlon, alt, spd, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type): 

        # if airspace type is 'constrained' then the action_lat/lon is the location of next turn
        # if airspace type is 'open' thne the action_lat/lon is the locaiton where an altitude change must occur due to heading rules
        # send command to bluesky waypoint stack
        traf.ap.route[idx].addwptStack(idx, latlon, alt, spd)

        # Get name
        name    = bs.traf.id[idx]
        
        # Add waypoint
        wpidx = self.addwpt(idx, name, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type)

        # Check for success by checking inserted location in flight plan >= 0
        if wpidx < 0:
            return False, "Waypoint " + name + " not added."

        # check for presence of orig/dest
        norig = int(bs.traf.ap.orig[idx] != "") # 1 if orig is present in route
        ndest = int(bs.traf.ap.dest[idx] != "") # 1 if dest is present in route
  
        # Check whether this is first 'real' waypoint (not orig & dest),
        # And if so, make active
        if self.nwp - norig - ndest == 1:  # first waypoint: make active
            self.direct(idx, self.wpname[norig])  # 0 if no orig

        return True
    
    def overwrite_wpt_data(self, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type):
        """
        Overwrites information for a waypoint, via addwpt_data/9
        """
        # TODO: check if it works

        self.addwpt_data(True, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type)
    
    def insert_wpt_data(self, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type):
        """
        Inserts information for a waypoint, via addwpt_data/9
        """
        # TODO: check if it works
        
        self.addwpt_data(True, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type)

    def addwpt_data(self, overwrt, wpidx, wpname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type):
        """
        Overwrites or inserts information for a waypoint
        """
        # Process the type of action lat and lon
        if edge_airspace_type == "constrained" or edge_airspace_type ==1:
            hdg_lat = 48.1351
            hdg_lon = 11.582

            turn_lat = action_lat
            turn_lon = action_lon

        elif edge_airspace_type == "open" or edge_airspace_type == 0:
            hdg_lat = action_lat
            hdg_lon = action_lon

            turn_lat = 48.1351
            turn_lon = 11.582
            

        if overwrt:
            self.wpname[wpidx]  = wpname
            self.wpedgeid[wpidx] = wpedgeid
            self.group_number[wpidx] = group_number
            self.flow_number[wpidx] = flow_number
            self.edge_layer_dict[wpidx] = edge_layer_dict   
            self.hdg_lat[wpidx] = hdg_lat
            self.hdg_lon[wpidx] = hdg_lon
            self.turn_lat[wpidx] = action_lat
            self.turn_lon[wpidx] = action_lon
            self.edge_airspace_type[wpidx] = edge_airspace_type

        else:
            self.wpname.insert(wpidx, wpname)
            self.wpedgeid.insert(wpidx, wpedgeid)
            self.group_number.insert(wpidx, group_number)
            self.flow_number.insert(wpidx, flow_number)
            self.edge_layer_dict.insert(wpidx, edge_layer_dict)
            self.turn_lat.insert(wpidx, turn_lat)
            self.turn_lon.insert(wpidx, turn_lon)
            self.hdg_lat.insert(wpidx, hdg_lat)
            self.hdg_lon.insert(wpidx, hdg_lon)
            self.edge_airspace_type.insert(wpidx, edge_airspace_type)

    def addwpt(self, iac, name, wpedgeid ="", group_number="", flow_number="", edge_layer_dict ="", action_lat=48.1351, action_lon=11.582, edge_airspace_type="open"):
        """Adds waypoint an returns index of waypoint, lat/lon [deg], alt[m]"""

        # For safety
        self.nwp = len(self.wpedgeid)

        name = name.upper().strip()

        newname = Route_edge.get_available_name(
            self.wpname, name, 3)

        wpidx = self.nwp

        self.addwpt_data(False, wpidx, newname, wpedgeid, group_number, flow_number, edge_layer_dict, action_lat, action_lon, edge_airspace_type)

        idx = wpidx
        self.nwp += 1

        return idx

    def direct(self, idx, wpnam):
        # print(idx)
        # print("Hello from direct")
        """Set active point to a waypoint by name"""
        name = wpnam.upper().strip()
        if name != "" and self.wpname.count(name) > 0:
            wpidx = self.wpname.index(name)
            self.iactwp = wpidx

            # set edge id and intersection/turn lon lat for actedge
            edge_traffic.actedge.wpedgeid[idx] = self.wpedgeid[wpidx]
            
            # distance to next node
            edge_traffic.actedge.intersection_lat[idx], edge_traffic.actedge.intersection_lon[idx] \
                = osmid_to_latlon(self.wpedgeid[wpidx], 1)

            # distance to next turn
            edge_traffic.actedge.turn_lat[idx] = self.turn_lat[wpidx]
            edge_traffic.actedge.turn_lon[idx] = self.turn_lon[wpidx]

            # distance to heading change
            edge_traffic.actedge.hdg_lat[idx] = self.hdg_lat[wpidx]
            edge_traffic.actedge.hdg_lon[idx] = self.hdg_lon[wpidx]

            # set group_number/flow number and edge layer_dict
            edge_traffic.actedge.group_number[idx] = self.group_number[wpidx]
            edge_traffic.actedge.flow_number[idx] = self.flow_number[wpidx]
            edge_traffic.actedge.edge_layer_dict[idx] = self.edge_layer_dict[wpidx]

            # set edge airspace type
            edge_traffic.actedge.edge_airspace_type[idx] = self.edge_airspace_type[wpidx]

            return True
        else:
            return False, "Waypoint " + wpnam + " not found"

    def getnextwp(self):
        # if self.iactwp < len(self.wpedgeid) - 1:
        #    self.iactwp += 1

        idx = bs.traf.id2idx(self.acid)
        self.iactwp = bs.traf.ap.route[idx].iactwp

        # get airspace type
        edge_airspace_type = self.edge_airspace_type[self.iactwp]

        # get next edge id
        wpedgeid = self.wpedgeid[self.iactwp]

        # get lat/lon of next intersection or distnace to constrained airspace
        if edge_airspace_type == "constrained" or edge_airspace_type == 1:
            intersection_lat ,intersection_lon = osmid_to_latlon(wpedgeid, 1)
            const_lat, const_lon = 48.1351, 11.582

        elif edge_airspace_type == "open" or edge_airspace_type == 0:
            intersection_lat ,intersection_lon = 48.1351, 11.582
            const_lat, const_lon = osmid_to_latlon(wpedgeid, 1)

        # update turn lat and lon
        turn_lat = self.turn_lat[self.iactwp]
        turn_lon = self.turn_lon[self.iactwp]

        # update heading lat and lon
        hdg_lat = self.hdg_lat[self.iactwp]
        hdg_lon = self.hdg_lon[self.iactwp]

        # Update group number/flow number and edge_layer_dict
        group_number = self.group_number[self.iactwp]
        flow_number = self.flow_number[self.iactwp]
        edge_layer_dict = self.edge_layer_dict[self.iactwp]

        return wpedgeid, intersection_lat, intersection_lon, group_number, flow_number, edge_layer_dict, turn_lat, turn_lon, hdg_lat, hdg_lon, \
            const_lat, const_lon, edge_airspace_type

    @staticmethod
    def get_available_name(data, name_, len_=2):
        """
        Check if name already exists, if so add integer 01, 02, 03 etc.
        """
        appi = 0  # appended integer to name starts at zero (=nothing)
        # Use Python 3 formatting syntax: "{:03d}".format(7) => "007"
        fmt_ = "{:0" + str(len_) + "d}"

        # Avoid using call sign without number
        if bs.traf.id.count(name_) > 0:
            appi = 1
            name_ = name_+fmt_.format(appi)

        while data.count(name_) > 0 :
            appi += 1
            name_ = name_[:-len_]+fmt_.format(appi)
        return name_

def osmid_to_latlon(osmid , i=2):

    # input an edge and get the lat lon of one of the nodes
    # i = 0 gets nodeid of first node of edges
    # i = 1 gets nodeid of second node of edge

    if not i == 2:
        # if given an edge
        node_id = int(osmid.split("-",1)[i])
    else:
        # if given a node
        node_id = int(osmid)

    node_latlon = edge_traffic.node_dict[node_id]
    node_lat = float(node_latlon.split("-",1)[0])
    node_lon = float(node_latlon.split("-",1)[1])

    return node_lat, node_lon

######################## FLIGHT LAYER TRACKING ############################
class FlightLayers(Entity):
    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.flight_levels                  = np.array([], dtype=int)
            self.flight_layer_type              = np.array([], dtype=str)

            self.closest_cruise_layer_bottom    = np.array([], dtype=int)
            self.closest_cruise_layer_top       = np.array([], dtype=int)

            self.closest_turn_layer_bottom      = np.array([], dtype=int)
            self.closest_turn_layer_top         = np.array([], dtype=int)
            
            self.closest_empty_layer_bottom     = np.array([], dtype=int)
            self.closest_empty_layer_top        = np.array([], dtype=int)

            self.lowest_cruise_layer            = np.array([], dtype=int)
            self.highest_cruise_layer           = np.array([], dtype=int)
            
            self.ac_angle_levels                = np.array([], dtype=float)
            self.leg_angle_levels               = np.array([], dtype=float)
            self.next_leg_angle_levels          = np.array([], dtype=float)

            self.open_closest_layer             = np.array([], dtype=int)

            self.constrained_airspace_alloc     = np.array([], dtype=str)

        # Patch bs.traf with new information
        bs.traf.flight_levels = self.flight_levels
        bs.traf.flight_layer_type = self.flight_layer_type
        bs.traf.closest_cruise_layer_bottom = self.closest_cruise_layer_bottom
        bs.traf.closest_cruise_layer_top = self.closest_cruise_layer_top
        bs.traf.closest_turn_layer_bottom = self.closest_turn_layer_bottom
        bs.traf.closest_turn_layer_top = self.closest_turn_layer_top
        bs.traf.closest_empty_layer_bottom = self.closest_empty_layer_bottom
        bs.traf.closest_empty_layer_top = self.closest_empty_layer_top
        bs.traf.lowest_cruise_layer = self.lowest_cruise_layer
        bs.traf.highest_cruise_layer = self.highest_cruise_layer
        bs.traf.ac_angle_levels = self.ac_angle_levels
        bs.traf.leg_angle_levels = self.leg_angle_levels
        bs.traf.next_leg_angle_levels = self.next_leg_angle_levels
        bs.traf.open_closest_layer = self.open_closest_layer
        bs.traf.constrained_airspace_alloc = self.constrained_airspace_alloc
        
        # Initliaze flight layer tracking variables
        self.layer_dict = {}
        self.heading_levels = {}
        self.layer_heading_level_choices = {}

        self.layer_spacing = None
        self.angle_spacing = None

        self.layer_levels = np.array([], dtype=int)
        self.layer_dist_center = np.array([], dtype=float)

        self.dist_between_cruise_layers = 0.0

        # vectorize function to extract layer_info
        self.layer_info = np.vectorize(self.get_layer_type)
    
    def create(self, n=1):
        super().create(n)

        self.flight_levels[-n:]                 = 0
        self.flight_layer_type[-n:]             = ""

        self.closest_cruise_layer_top[-n:]      = 0
        self.closest_cruise_layer_bottom[-n:]   = 0

        self.closest_turn_layer_top[-n:]        = 0
        self.closest_turn_layer_bottom[-n:]     = 0
        
        self.closest_empty_layer_top[-n:]       = 0
        self.closest_empty_layer_bottom[-n:]    = 0

        self.lowest_cruise_layer[-n:]           = 0
        self.highest_cruise_layer[-n:]          = 0
        
        self.ac_angle_levels[-n:]               = 0.0
        self.leg_angle_levels[-n:]              = 0.0
        self.next_leg_angle_levels[-n:]         = 0.0

        self.open_closest_layer[-n:]            = 0

        self.constrained_airspace_alloc[-n:]    = angle_range
    
    def load(self, dict_file_path):
        # Load layer structure from stack command

        # Opening edges.JSON as a dictionary
        with open(f'{dict_file_path}/layers.json', 'r') as filename:
            self.layer_dict = json.load(filename)
        
        self.layer_spacing = self.layer_dict['info']['spacing']
        self.angle_spacing = self.layer_dict['info']['angle_spacing']
        self.layer_levels = np.array(self.layer_dict['info']['levels'])
        self.layer_dist_center = self.layer_levels - self.layer_dict['info']['spacing']/2
        self.heading_levels = self.layer_dict['config']['open']['heading']['heights']['center']
        self.layer_heading_level_choices = self.layer_dict['config']['open']['heading']['angle']['center']

        # get distance between cruise layers
        const_pattern = np.array(self.layer_dict['config']['0']['pattern'])
        cruise_layers = self.layer_levels[np.where(const_pattern == 'C')]
        self.dist_between_cruise_layers = cruise_layers[1] - cruise_layers[0]
        
        bs.traf.dist_between_cruise_layers = self.dist_between_cruise_layers

    def layer_tracking(self):
        
        # update flight levels
        self.flight_levels = np.array((np.round((bs.traf.alt/ft) / self.layer_spacing))*self.layer_spacing, dtype=int)
        
        self.flight_levels = np.where(self.flight_levels < 0, 30, self.flight_levels)
        self.flight_levels = np.where(self.flight_levels > 480, 480, self.flight_levels)

        # get ccw headin of aircraft and get middle angle level
        ccw_ac = bs.traf.hdg % 360
        self.ac_angle_levels = np.array((np.floor((ccw_ac) / self.angle_spacing)*self.angle_spacing+ self.angle_spacing/2), dtype=float)

        # get ccw headin of current leg and actual angle level of whre you should be
        ccw_leg = bs.traf.actwp.curlegdir % 360
        self.leg_angle_levels = np.array((np.floor((ccw_leg) / self.angle_spacing)*self.angle_spacing+ self.angle_spacing/2), dtype=float)

        # get ccw headin of current leg and actual angle level of whre you should be
        ccw_next_leg = bs.traf.actwp.next_qdr % 360
        self.next_leg_angle_levels = np.array((np.floor((ccw_next_leg) / self.angle_spacing)*self.angle_spacing+ self.angle_spacing/2), dtype=float)

        # update flight layer type
        edge_layer_dicts = bs.traf.actedge.edge_layer_dict

        # only go into vectorized function if there is traffic
        if bs.traf.ntraf > 0:
            self.flight_layer_type, self.closest_cruise_layer_bottom, self.closest_cruise_layer_top,\
            self.closest_turn_layer_bottom, self.closest_turn_layer_top,\
            self.closest_empty_layer_bottom, self.closest_empty_layer_top,\
            self.lowest_cruise_layer, self.highest_cruise_layer,\
            self.open_closest_layer = self.layer_info(self.flight_levels, edge_layer_dicts, 
                                                        edge_traffic.actedge.edge_airspace_type, 
                                                        self.leg_angle_levels, 
                                                        self.layer_heading_level_choices,
                                                        bs.traf.roguetraffic.rogue_bool)

        # patch bs.traf with new information
        bs.traf.flight_levels = self.flight_levels
        bs.traf.flight_layer_type = self.flight_layer_type
        bs.traf.closest_cruise_layer_bottom = self.closest_cruise_layer_bottom
        bs.traf.closest_cruise_layer_top = self.closest_cruise_layer_top
        bs.traf.closest_turn_layer_bottom = self.closest_turn_layer_bottom
        bs.traf.closest_turn_layer_top = self.closest_turn_layer_top
        bs.traf.closest_empty_layer_bottom = self.closest_empty_layer_bottom
        bs.traf.closest_empty_layer_top = self.closest_empty_layer_top
        bs.traf.lowest_cruise_layer = self.lowest_cruise_layer
        bs.traf.highest_cruise_layer = self.highest_cruise_layer
        bs.traf.ac_angle_levels = self.ac_angle_levels
        bs.traf.leg_angle_levels = self.leg_angle_levels
        bs.traf.next_leg_angle_levels = self.next_leg_angle_levels
        bs.traf.open_closest_layer = self.open_closest_layer
        bs.traf.constrained_airspace_alloc = self.constrained_airspace_alloc

        return

    @staticmethod
    def get_layer_type(flight_level, edge_layer_dict, airspace_type, leg_angle_level, layer_heading_level_choices, rogue_bool):

        # print("print",flight_level,edge_layer_dict, airspace_type )
        # vectorized function to process edge layer dictionary
        # TODO: maybe access dictionaries here instead of having it
        # as active waypoint data

        # first check if it is a rogue aircraft and return empty things
        if rogue_bool:
            return "", 0, 0, 0, 0, 0, 0, 0, 0, 0


        # get correct layer_info_list based on your flight level
        layer_list = edge_layer_dict[f'{flight_level}']
        
        # get layer_type
        layer_type = layer_list[0]

        # get closest cruise layers
        closest_cruise_layer_bottom = layer_list[1]
        closest_cruise_layer_top = layer_list[2]

        if closest_cruise_layer_bottom == '':
            closest_cruise_layer_bottom = 0

        if closest_cruise_layer_top =='':
            closest_cruise_layer_top = 0
        # do constrained
        if airspace_type == 'constrained' or airspace_type == '1' or airspace_type == 1:
            # get closest turnlayers
            closest_turn_layer_bottom = layer_list[3]
            closest_turn_layer_top = layer_list[4]

            if closest_turn_layer_bottom == '':
                closest_turn_layer_bottom = 0

            if closest_turn_layer_top =='':
                closest_turn_layer_top = 0
                
            # get closest empty layers
            closest_empty_layer_bottom = layer_list[5]
            closest_empty_layer_top = layer_list[6]

            if closest_empty_layer_bottom == '':
                closest_empty_layer_bottom = 0

            if closest_empty_layer_top =='':
                closest_empty_layer_top = 0

            # get idx lowest cruise layer and highest
            idx_lowest_cruise_layer, idx_highest_cruise_layer = 7, 8

            # in closed airspace open closest layer is zero
            open_closest_layer = 0
        
        elif airspace_type == 'open' or airspace_type == '0' or airspace_type == 0:
            
            # In open airspace there are no turns or empty layers
            closest_turn_layer_bottom = 0
            closest_turn_layer_top = 0
            closest_empty_layer_bottom = 0
            closest_empty_layer_top = 0

            # get lowest cruise layer and highest
            idx_lowest_cruise_layer, idx_highest_cruise_layer = 3, 4

            # get open closest layer depending on angle
            # get choices of where you should be
            height_choices = layer_heading_level_choices[str(leg_angle_level)]

            # choose closest layer to current flight level
            open_closest_layer = min(height_choices, key=lambda x:abs(x-flight_level))

        lowest_cruise_layer = layer_list[idx_lowest_cruise_layer]

        if lowest_cruise_layer == '':
            lowest_cruise_layer = 0
        
        # get highest cruise layer
        highest_cruise_layer = layer_list[idx_highest_cruise_layer]

        if highest_cruise_layer == '':
            highest_cruise_layer = 0

        return layer_type, closest_cruise_layer_bottom, closest_cruise_layer_top, \
            closest_turn_layer_bottom, closest_turn_layer_top, \
            closest_empty_layer_bottom, closest_empty_layer_top, \
            lowest_cruise_layer, highest_cruise_layer, open_closest_layer

############################ Path Plans ###################################

class PathPlans(Entity):
    def __init__(self):
        super().__init__()
        
        with self.settrafarrays():
            self.pathplanning = []
            self.lineroutes = np.array([], dtype=LineString)
            self.route_coords = np.array([], dtype=object)
            self.lineintersects = np.array([], dtype=bool)

        bs.traf.lineroutes = self.lineroutes
        bs.traf.lineintersects = self.lineintersects

        self.transformer_to_utm    = Transformer.from_crs("EPSG:4326", "EPSG:32633")
        self.transformer_to_latlon = Transformer.from_crs("EPSG:32633", "EPSG:4326")

    def load(self, loitering_fpath):
        # load loitering aircraft 
        # self.loitering_edges_dict = dill.load(open(loitering_fpath, 'rb'))

        # make loitering dill empty
        self.loitering_edges_dict = {}

            
    def create(self, n = 1):
        super().create(n)
        # Only do path planning for aircraft that are not Rogue aircraft.
        # Rogue aircraft will have an id that starts with 'R'
        acid = bs.traf.id[-1]
        if acid[0] == 'R':
            self.pathplanning[-1] = None
            return
                
        path_file = f'plugins/streets/path_plan_dills/{dill_to_load}.dill'
        self.pathplanning[-1] = dill.load(open(path_file, 'rb'),ignore=True)
        self.pathplanning[-1].flow_graph=self.graph
        ridx = -1
        
        edges_changes=[]
        for key in path_plans.graph.flows_dict.keys():
            key_str=key
            key=int(key)
            
            if path_plans.graph.modified_group[key]==2 or path_plans.graph.modified_group[key]==1:
                for edge in path_plans.graph.flows_dict[key_str]:
                    k = int(edge.split("-")[0])
                    kk = int(edge.split("-")[1])
                    if (k,kk) in path_plans.graph.loiter_nfz_edges:
                        continue
                    tmp=[k,kk,path_plans.graph.edges_graph[k][kk].speed]#the keys of the vertices of the edges, followed by the new speed
                    edges_changes.append(tmp)
                    
        for (k,kk) in path_plans.graph.loiter_nfz_edges:

            tmp=[k,kk,path_plans.graph.edges_graph[k][kk].speed]#the keys of the vertices of the edges, followed by the new speed
            edges_changes.append(tmp)

        route,turns,edges,next_turn,groups,in_constrained,turn_speeds=self.pathplanning[-1].replan_spawned(edges_changes,
                                                        self.pathplanning[-1].start_index_previous,self.pathplanning[-1].start_index,
                                                        self.pathplanning[-1].start_point.y,self.pathplanning[-1].start_point.x)
        #[lat, lon, alt, spd, TURNSPD/FLYBY, turn speed, wpedgeid, group number, action_lat, action_lon, airspace_type]
        # Get needed values
        acrte = Route._routes.get(acid)
        #print(turns)
        
        # get coordinates for route in utm
        utm_x = []
        utm_y = []

        for j, rte in enumerate(route):
            lat = rte[1] # deg
            lon = rte[0] # deg

            # convert to utm #TODO: check x y in correct order
            x,y = self.transformer_to_utm.transform(lat, lon)
            utm_x.append(x)
            utm_y.append(y)

            alt = -999
            spd = -999
            
            # Do flyby or flyturn processing
            if turns[j] and j < len(route)-1:
                acrte.turnspd = turn_speeds[j]*kts
                acrte.swflyby   = False
                acrte.swflyturn = True
            else:
                # Either it's a flyby, or a typo.
                acrte.swflyby   = True
                acrte.swflyturn = False
            
            name    = acid
            wptype  = Route.wplatlon
            
            wpidx = acrte.addwpt_simple(ridx, name, wptype, lat, lon, alt, spd)
        
            # Add the streets stuff
            # get group number
            group_number = groups[j]
            
            wpedgeid = f'{edges[j][0]}-{edges[j][1]}'
            
            edge_layer_type = edge_traffic.edge_dict[wpedgeid]['height_allocation']
            edge_layer_dict = flight_layers.layer_dict["config"][edge_layer_type]['levels']

            # when layer type is not in open airspace check if there is a heading based
            # constrained airspace
            if edge_layer_type != 'open' and heading_based_constrained:
                # Get the layer number
                edge_layer_dict = edge_layer_dict[angle_range]

            flow_number = edge_traffic.edge_dict[wpedgeid]['flow_group']

            # get the edge_airspace_type
            if in_constrained[j]:
                edge_airspace_type = 'constrained'
            else:
                edge_airspace_type = 'open'

            turn_lat = next_turn[j][1]
            turn_lon = next_turn[j][0]
            edge_traffic.edgeap.edge_rou[ridx].addwpt(ridx, name, wpedgeid, group_number, flow_number, edge_layer_dict, 
                                        turn_lat, turn_lon, edge_airspace_type)

        # For this aircraft, manually set the first "next_qdr" in actwp
        # We basically need to find the qdr between the second and the third waypoint, as
        # the first one is the origin
        if len(acrte.wplat)>2:
            bs.traf.actwp.next_qdr[ridx], dummy = geo.qdrdist(acrte.wplat[1], acrte.wplon[1],
                                                        acrte.wplat[2], acrte.wplon[2])
            
        # Calculate flight plan
        acrte.calcfp()
        edge_traffic.edgeap.edge_rou[ridx].direct(ridx,edge_traffic.edgeap.edge_rou[ridx].wpname[1])

        # get route in a traff array
        self.route_coords[-1] = list(zip(utm_x, utm_y))

        # add route as linestring for projected based on UTM coordinates
        original_route = LineString(list(zip(utm_x, utm_y))).simplify(0.0001)

        #assert that the route_merged is a linestring
        self.lineintersects[-1] = not original_route.is_simple
        
        # finally assign to trafficarray
        self.lineroutes[-1] = original_route

# =============================================================================
#         bs.traf.swlnav[ridx]    = True
#         bs.traf.swvnav[ridx]    = True
#         bs.traf.swvnavspd[ridx] = True
# =============================================================================
    
    def load_flow_dill(self, fpath):
        self.graph=dill.load(open(f"{fpath}/Flow_control.dill", "rb"))
