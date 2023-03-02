# 3D Heatmap in Python using matplotlib
  
# to make plot interactive 
#%matplotlib
  
# importing required libraries
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from pylab import *

import sys
  
print('arg:', sys.argv)
if len(sys.argv) <= 1:
    print('Give file path as arg')
    exit()

filepath = sys.argv[1]
f = open(filepath, "r")
data = f.read()

# creating a dataset
x_succ = []
y_succ = []
z_succ = []

x_fail = []
y_fail = []
z_fail = []

for l in data.split("\n"):
    d = l.split(",")
    if len(d) == 0:
        continue
    s = d[0]
    for p in d[1:]:
        p = p.split(" ")
        if len(p) < 3:
            continue
        if s == 'success':
           x_succ.append(float(p[0])) 
           y_succ.append(float(p[1]))
           z_succ.append(float(p[2]))
        else:
           x_fail.append(float(p[0]))
           y_fail.append(float(p[1]))
           z_fail.append(float(p[2]))

x_succ = np.array(x_succ)
y_succ = np.array(y_succ)
z_succ = np.array(z_succ)

x_fail = np.array(x_fail)
y_fail = np.array(y_fail)
z_fail = np.array(z_fail)
  
# creating figures
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# setting color bar
color_map1 = cm.ScalarMappable(cmap=cm.Greens_r)
color_map1.set_array([x_succ + y_succ + z_succ])
  
color_map2 = cm.ScalarMappable(cmap=cm.Reds_r)
color_map2.set_array([x_fail + y_fail + z_fail])

# creating the heatmap
img1 = ax.scatter(x_succ, y_succ, z_succ, marker='s',
                 s=2, color='green')
  
img2 = ax.scatter(x_fail, y_fail, z_fail, marker='s',
                 s=0.1, color='red')
# adding title and labels
ax.set_title("3D Heatmap")
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
  
# displaying plot
plt.show()
