import numpy as np
import os, sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/etc")

arr = np.loadtxt("1_5__Waypoint.txt")
# print(arr)
a = arr[::,1:3]
b = np.transpose(a)
print(b)
np.save("jeju_island_gp.npy",a)