from matplotlib.lines import lineStyles
import matplotlib.pyplot as plt
import numpy as np
from run import generate_results_18_minuets, generate_results_18_minutes_cpfa


scores, collision_times, time_inside_red_circle = generate_results_18_minuets()
scores_cpfa, collision_times_cpfa, time_inside_red_circle_cpfa = generate_results_18_minutes_cpfa()

# plt.figure(figsize=(8, 6))
# plt.title("Resouce Collection Comparison in 18 minutes")
# plt.plot(scores, marker="o", label='cpfa_version_2', color='blue')
# plt.plot(scores_cpfa, marker="o", label='cpfa_original', color='red')
# plt.xticks(ticks=np.arange(5), labels=['48', '64', '80', '96', '112'])
# plt.xlabel("Number of Robots")
# plt.ylabel("Mean Values")
# plt.legend()
# plt.grid(True, linestyle='--')
# plt.show()

# plt.figure(figsize=(8, 6))
# plt.title("Collision Time Comparison in 18 minutes")
# plt.plot(collision_times, marker="o", label='cpfa_version_2', color='blue')
# plt.plot(collision_times_cpfa, marker="o", label='cpfa_original', color='red')
# plt.xticks(ticks=np.arange(5), labels=['48', '64', '80', '96', '112'])
# plt.xlabel("Number of Robots")
# plt.ylabel("Mean Values")
# plt.legend()
# plt.grid(True, linestyle='--')
# plt.show()

plt.figure(figsize=(8, 6))
plt.title("Drop Resource Time Comparison in 18 minutes")
plt.plot(time_inside_red_circle, marker="o", label='cpfa_version_2', color='blue')
plt.plot(time_inside_red_circle_cpfa, marker="o", label='cpfa_original', color='red')
plt.xticks(ticks=np.arange(5), labels=['48', '64', '80', '96', '112'])
plt.xlabel("Number of Robots")
plt.ylabel("Mean Values [mins]")
plt.legend()
plt.grid(True, linestyle='--')
plt.show()

