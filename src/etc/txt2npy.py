import numpy as np
import os, sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/etc")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/src/path/npy_file/path")

arr1 = np.load("jeju_lane.npy")
arr2 = np.load("jeju_island_gp.npy")
# print(arr)
a = np.concatenate((arr1,arr2))

np.save("jeju_island_gp_lane.npy",a)