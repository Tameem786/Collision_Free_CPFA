import os
import numpy as np
import matplotlib.pyplot as plt

num_of_bots = [32, 48, 64, 80, 96]
mean_values = []

for i in num_of_bots:
  file = f'simulation_results_{i}_robots.txt'

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

# print(mean_values)

plt.figure(figsize=(8, 6))
plt.title("Comparison between different Robot Size (CPFA Version 1)")
plt.plot(mean_values, label='Avg on 10 Runs (15 mins each)', marker='o')
plt.xticks(ticks=np.arange(5), labels=['32', '48', '64', '80', '96'])
plt.xlabel("Number of Robots")
plt.ylabel("Resource Collected")
plt.grid(True)
plt.legend()
plt.show()