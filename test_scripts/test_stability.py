import numpy as np

data = np.genfromtxt('../../data/250303.csv', delimiter=',', skip_header=1)

t_start = data[0, 0] + 7e10
t_indices = np.where(data[:, 0] > t_start)[0]

cov_list = []
for i in range(len(t_indices) - 10):
    for j in range(1, 4):
        data_c = data[t_indices[i]:t_indices[i]+10, j]
        cov_list.append(np.cov(data_c)) 
    
print(np.max(cov_list))