# Simple script to create scenarios and a batch file

batchfilename = 'scenario/CDR/batch.scn'
# How many repetitions
num_repetitions = 5

scenario_names = []

# Independent variables
#densities = [50, 100, 150, 200, 250]
densities = [600, 800, 1000]
cd_cr_methods = [['M22CD', 'M22CR'],['INTENTCD', 'INTENTCR'], ['DEFENSIVECD', 'DEFENSIVECR']]

for density in densities:
    for method in cd_cr_methods:
        for i in range(num_repetitions):
            to_write = f'00:00:00>TRAFFICNUMBER {density}\n' + \
                    f'00:00:00>SEED {i}\n' + \
                    f'00:00:00>CDMETHOD {method[0]}\n' + \
                    f'00:00:00>RESO {method[1]}\n' + \
                    '00:00:00>STARTLOGS\n' + \
                    '00:00:00>STARTCDRLOGS\n' + \
                    '00:00:01>FF'
            scen_name = f'CDR_{method[0]}_{density}_{i}'
            scenario_names.append(scen_name)
            with open(f'scenario/CDR/{scen_name}.scn', 'w') as f:
                f.write(to_write)

with open(batchfilename, 'w') as f:
    for name in scenario_names:
        to_write = f'00:00:00.00>SCEN {name}\n' + \
                    f'00:00:00.00>PCALL CDR/{name}.scn\n' + \
                    '00:00:00>SCHEDULE 02:00:00 HOLD\n' + \
                    '00:00:00.00>FF\n\n'
                    
        f.write(to_write)
