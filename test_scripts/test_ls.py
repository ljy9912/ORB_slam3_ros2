import numpy as np
from scipy.spatial.transform import Rotation as R

data_file = '../../data/250303-3.csv'
data = np.genfromtxt(data_file, delimiter=',')
data = data[1:, :]
data_length = data.shape[0]
data = data[:data_length, :]

A_rows = []
b_rows = []

for i in range(data.shape[0]):
    q = data[i, 4:]
    pc = data[i, 1:4]
    try:
        rotation = R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()  # 转换为 (x, y, z, w) 输入
    except:
        print(f"Error at line {i}")
        break
    Rc = rotation
    
    # 构建系数块 [I | Rc]
    block = np.hstack([np.eye(3), -Rc])
    A_rows.append(block)
    
    # 构建观测向量
    b_rows.append(pc.reshape(-1, 1))

A = np.vstack(A_rows)  # 最终维度: 3N x 6
b = np.vstack(b_rows)  # 最终维度: 3N x 1

# 使用 numpy 的最小二乘法
x, residuals, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)

# 提取结果
ps = x[:3].flatten()
off = x[3:].flatten()

print(f"Optimized ps: {ps}")
print(f"Optimized off: {off}")
print(f'Residuals: {residuals / data_length}')
print(f'Length of offsets: {np.sqrt(np.sum(off**2))}')
print(f'Data length: {data.shape[0]}')