import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取数据
data = np.loadtxt("FrameTrajectory_TUM_Format.txt", delimiter=" ")
timestamps = data[:,0]
timestamps_diff = data[1:, 0] - data[:-1, 0]
positions = data[:,1:4]

# 绘制 3D 轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:,0], positions[:,1], positions[:,2], 'b-')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.savefig('test.png')
plt.clf()
plt.plot(timestamps_diff)
plt.savefig('time.png')