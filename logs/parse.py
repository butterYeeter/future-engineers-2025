import csv
import matplotlib.pyplot as plt
import numpy as np

with open("log_2020_04_10_20_53_30_434585.csv", "r") as f:
  lines = f.read().split("\n")

lines = lines[1:-2]

angles = np.sqrt(np.array([float(l.split(", ")[0]) for l in lines]))
# angles = 2 * ((angles - np.max(angles)) / (np.max(angles) - np.min(angles))) - 1
# angles = 0.5 * np.log((1 + angles) / (1 - angles))
# print(angles)
# target_angles = [int(l.split(", ")[1]) for l in lines]
# left_distance = [int(l.split(", ")[2]) for l in lines]

# delta_distances = [-left_distance[i] + left_distance[i - 1] for i in range(1, len(left_distance))]

fig, axes = plt.subplots(1,1) # Create two subplots

# axes.plot(left_distance, label="left distance")
# axes.hist(angles)
axes.plot(angles, label="angle")
# axes.plot(target_angles, label="target angle")
# axes.plot(delta_distances, label="delta")
plt.legend()
plt.show()