# Simple script to create a batch file

filename = 'scenario/CDSTUDYBATCH.scn'
# How many repetitions
num_repetitions = 16

with open(filename, 'w') as f:
    for i in range(num_repetitions):
        to_write = f'00:00:00.00>SCEN INTENTCD_{i+1}\n' + \
                    '00:00:00.00>PCALL CDSTUDY.scn\n' + \
                    f'00:00:00.00>SEED {i}\n' + \
                    '00:00:00.00>FF\n\n'
                    
        f.write(to_write)
