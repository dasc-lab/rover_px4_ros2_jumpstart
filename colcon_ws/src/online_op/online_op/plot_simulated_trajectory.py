import numpy as np
import sys, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('/Users/albusfang/Coding Projects/gp_ws/rover_px4_ros2_jumpstart/colcon_ws/src/online_op/online_op/')
folder_path = '/Users/albusfang/Coding Projects/gp_ws/rover_px4_ros2_jumpstart/colcon_ws/src/online_op/online_op/'
arr = np.load(folder_path+"dataset/pos_vec.npy")
print(arr.shape)
x = arr[:,0]
y = arr[:,1]
z = arr[:,2]
print(type(x))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_zlim(0,1)
ax.scatter(x, y, z, c= 'r')
# ax.scatter(x_ideal, y_ideal, z_ideal, c = 'b')
#plt.savefig("trajectory.png")
plt.show()