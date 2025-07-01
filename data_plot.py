import matplotlib.pyplot as plt
import numpy as np
from run import generate_results

# X-axis: Groups of robots
robot_groups = [32, 48, 64, 80, 96]

# Each group has results for [15, 18, 20] minutes
time_durations = [15, 18, 20]

# Dummy resource collection data (replace with your real data)
# Each inner list is for one robot group: [at 15 min, 18 min, 20 min]
resource_data = generate_results()

# Transpose for easy plotting
resource_data = np.array(resource_data).T

# Bar width
bar_width = 0.2
x = np.arange(len(robot_groups))  # the label locations

# Plot
plt.figure(figsize=(10, 6))
for i, duration in enumerate(time_durations):
    plt.bar(x + i * bar_width, resource_data[i], width=bar_width, label=f'{duration} mins')

# Labels and formatting
plt.xlabel('Number of Robots')
plt.ylabel('Resources Collected')
plt.title('Resource Collection vs Number of Robots (CPFA Version 2)')
plt.xticks(x + bar_width, robot_groups)
plt.legend()
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()
