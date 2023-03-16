import dill
import plugins.streets.agent_path_planning
import plugins.streets.flow_control
import os
import numpy as np
import json

graph=dill.load(open("graph_data/M2-graph/Flow_control.dill", "rb"))

# get list of scenarios
scenarios = os.listdir('scenarios')

dict_turns_and_ac = {
    'very_low'  : {
            repetition: {
                'acids_in_constrained': [],
                'num_ac_constrained' : 0
            } for repetition in range(0,9)    
        },
    'low'       : {
            repetition: {
                'acids_in_constrained': [],
                'num_ac_constrained' : 0
            } for repetition in range(0,9)    
        },
    'medium'    : {
            repetition: {
                'acids_in_constrained': [],
                'num_ac_constrained' : 0
            } for repetition in range(0,9)    
        },
    'high'      : {
            repetition: {
                'acids_in_constrained': [],
                'num_ac_constrained' : 0
            } for repetition in range(0,9)    
        },
    'ultra'     : {
            repetition: {
                'acids_in_constrained': [],
                'num_ac_constrained' : 0
            } for repetition in range(0,9)    
        },
}

for scn in scenarios:
    # read the scenario
    density = 'very_low' if 'very' in scn else scn.split('_')[2]
    repetition = int(scn.split('_')[-2])

    with open(f'scenarios/{scn}', 'r') as f:
        lines = f.readlines()[12:]

    # get acids and dills
    acids_dills = [
        (cmd.split(',')[0].split(' ')[-1] , cmd.split(',')[2]) for cmd in lines
    ]


    # Initialize list to save information
    acids_in_constrained = []
    for acid, dill_to_load in acids_dills:
        
        # open the dill
        path_file = f'path_plan_dills/{dill_to_load}.dill'

        loaded_dill = dill.load(open(path_file, 'rb'),ignore=True)
        loaded_dill.flow_graph=graph
                
        route,turns,edges,next_turn,groups,in_constrained,turn_speeds=loaded_dill.replan_spawned([],
                                                    loaded_dill.start_index_previous,loaded_dill.start_index,
                                                    loaded_dill.start_point.y,loaded_dill.start_point.x)

        in_constrained = np.array(in_constrained)

        if np.any(in_constrained):
            acids_in_constrained.append(acid)
            

    # update dictionary
    dict_turns_and_ac[density][repetition]['acids_in_constrained'] = acids_in_constrained
    dict_turns_and_ac[density][repetition]['num_ac_constrained'] = len(acids_in_constrained)

    
    print(scn)
    print(acids_in_constrained)
    print(len(acids_in_constrained))
    print(dict_turns_and_ac)
    print('-----------')


# Writing to sample.json
with open("ac_in_constrained.json", "w") as outfile:
    outfile.write(dict_turns_and_ac, indent=4)