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


def generate_results_18_minuets():

  # Resource Collected: %f, Collision Time: %f, Time Took: %f minutes, Random Seed: %lu

  num_of_bots = [48, 64, 80, 96, 112]
  score = []
  collision_time = []
  time_inside_red_circle = []

  for i in num_of_bots:
    file = f'results_18_mins_{i}_robots.txt'

    if os.path.exists(file):
      scores = []
      collision_times = []
      times = []
      with open(file) as f:
        for line in f.readlines():
          if('Resource' in line):
            scores.append(float(line.split(',')[0].split(' ')[2].strip()))
            collision_times.append(float(line.split(',')[1].split(' ')[3].strip()))
            times.append(float(line.split(',')[2].split(' ')[4].strip()))
      score.append(np.mean(scores))
      collision_time.append(np.mean(collision_times))
      time_inside_red_circle.append(np.mean(times))
      f.close()

    else:
      print(f'No File {file}')
  
  return score, collision_time, time_inside_red_circle


def generate_results_18_minutes_cpfa():

  # Resource Collected: %f, Collision Time: %f, Time Took: %f minutes, Random Seed: %lu

  num_of_bots = [48, 64, 80, 96, 112]
  score = []
  collision_time = []
  time_inside_red_circle = []

  for i in num_of_bots:
    file = f'results/results_18_mins_{i}_robots_cpfa.txt'

    if os.path.exists(file):
      scores = []
      collision_times = []
      times = []
      with open(file) as f:
        for line in f.readlines():
          if('Resource' in line):
            scores.append(float(line.split(',')[0].split(' ')[2].strip()))
            collision_times.append(float(line.split(',')[1].split(' ')[3].strip()))
            times.append(float(line.split(',')[2].split(' ')[4].strip()))
      score.append(np.mean(scores))
      collision_time.append(np.mean(collision_times))
      time_inside_red_circle.append(np.mean(times))
      f.close()

    else:
      print(f'No File {file}')
  
  return score, collision_time, time_inside_red_circle
