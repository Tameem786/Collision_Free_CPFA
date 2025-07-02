import os
import numpy as np


def generate_results():
  num_of_bots = [32, 48, 64, 80, 96]
  durations = [15, 18, 20]
  result = []

  for i in num_of_bots:
    mean_values = []
    for time in durations:
      file = f'results_{time}_mins_{i}_robots.txt'

      if os.path.exists(file):
        resource_collected = []
        with open(file) as f:
          for line in f.readlines():
            if('Resource' in line):
              resource_collected.append(float(line.split(',')[0].split(' ')[2].strip()))
        mean_values.append(np.mean(resource_collected))
        f.close()

      else:
        print(f'No File {file}')
    
    result.append(mean_values)
  
  return result