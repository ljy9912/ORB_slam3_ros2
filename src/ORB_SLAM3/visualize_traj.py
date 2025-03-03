import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取数据
try:
    data_path = sys.argv[1]
except IndexError:
    print("Usage: python3 visualize_traj.py path_to_traj.csv")
    sys.exit(1)
data = np.genfromtxt(data_path, delimiter=",")

# 绘制 3D 轨迹
plt.plot(data[1:, 0], data[1:, 1], label='x')
plt.legend()
plt.show()