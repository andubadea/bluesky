# Simple script to create scenarios and a batch file
experiment_name = 'CDR'
folder_name = f'scenario/{experiment_name}/'
batchfilename = f'{folder_name}batchSB.scn'
# How many repetitions
num_repetitions = 5
scen_duration = '02:00:00'

scenario_names = []

# Independent variables
densities = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]
winds = [2,4,6,8]
directions = [0, 90, 180, 270]
cd_cr_methods = [['M22CD', 'M22CR'],['INTENTCD', 'DEFENSIVECR'], ['DEFENSIVECD', 'DEFENSIVECR']]
cd_cr_methods = [['M22CD', 'M22CR']]

for density in densities:
    for method in cd_cr_methods:
        for i in range(num_repetitions):
            to_write = f'00:00:00>TRAFFICNUMBER {density}\n' + \
                    f'00:00:00>SEED {i}\n' + \
                    f'00:00:00>CDMETHOD {method[0]}\n' + \
                    f'00:00:00>RESO {method[1]}\n' + \
                    '00:00:00>STARTLOGS\n' + \
                    '00:00:00>STARTCDRLOGS\n' + \
                    '00:00:00>IMPL WINDSIM CDRWIND\n' + \
                    '00:00:00>SETWIND 0 0\n' + \
                    '00:00:01>FF'
            scen_name = f'{experiment_name}_{method[0]}_{density}_{i}_{0}_{0}_CR'
            scenario_names.append(scen_name)
            with open(f'{folder_name}{scen_name}.scn', 'w') as f:
                f.write(to_write)
# #Noreso
# for density in densities:
#     for method in cd_cr_methods:
#         for i in range(num_repetitions):
#             to_write = f'00:00:00>TRAFFICNUMBER {density}\n' + \
#                     f'00:00:00>SEED {i}\n' + \
#                     f'00:00:00>CDMETHOD {method[0]}\n' + \
#                     '00:00:00>STARTLOGS\n' + \
#                     '00:00:00>STARTCDRLOGS\n' + \
#                     '00:00:00>IMPL WINDSIM CDRWIND\n' + \
#                     '00:00:00>SETWIND 0 0\n' + \
#                     '00:00:01>FF'
#             scen_name = f'{experiment_name}_{method[0]}_{density}_{i}_{0}_{0}_NR'
#             scenario_names.append(scen_name)
#             with open(f'{folder_name}{scen_name}.scn', 'w') as f:
#                 f.write(to_write)
#         break
                
# Wind
for windmag in winds:
    for winddir in directions:
        for method in cd_cr_methods:
            for i in range(num_repetitions):
                density = 300
                to_write = f'00:00:00>TRAFFICNUMBER {density}\n' + \
                        f'00:00:00>SEED {i}\n' + \
                        f'00:00:00>CDMETHOD {method[0]}\n' + \
                        f'00:00:00>RESO {method[1]}\n' + \
                        '00:00:00>STARTLOGS\n' + \
                        '00:00:00>STARTCDRLOGS\n' + \
                        '00:00:00>IMPL WINDSIM CDRWIND\n' + \
                        f'00:00:00>SETWIND {windmag} {winddir}\n' + \
                        '00:00:01>FF'
                scen_name = f'{experiment_name}_{method[0]}_{density}_{i}_{windmag}_{winddir}_CR'
                scenario_names.append(scen_name)
                with open(f'{folder_name}{scen_name}.scn', 'w') as f:
                    f.write(to_write)

with open(batchfilename, 'w') as f:
    for name in scenario_names:
        to_write = f'00:00:00.00>SCEN {name}\n' + \
                    f'00:00:00.00>PCALL {experiment_name}/{name}.scn\n' + \
                    f'00:00:00>SCHEDULE {scen_duration} HOLD\n' + \
                    '00:00:00.00>FF\n\n'
        f.write(to_write)
