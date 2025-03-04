import numpy as np

class RecursiveLeastSquares:
    """
    递归最小二乘法(RLS)实现，用于解决线性最小二乘问题 min ||Ax - b||²
    特别适用于求解高维系统
    """
    
    def __init__(self, n_params=None, forgetting_factor=1.0, delta=1000.0):
        """
        初始化RLS求解器
        
        参数:
        n_params: 参数向量x的维度
        forgetting_factor: 遗忘因子(0<λ≤1)，控制对旧数据的"记忆"
        delta: 初始协方差矩阵的缩放因子
        """
        self.n_params = n_params
        self.lambda_ = forgetting_factor
        self.delta = delta
        self.theta = None  # 参数向量 (对应于x)
        self.P = None      # 协方差矩阵
        self.initialized = False
    
    def _initialize(self, n_params):
        """初始化参数向量和协方差矩阵"""
        self.n_params = n_params
        # 初始参数估计为零向量
        self.theta = np.zeros(n_params)
        # 初始化协方差矩阵为大值对角阵
        self.P = self.delta * np.eye(n_params)
        self.initialized = True
    
    def update(self, a, b_scalar):
        """
        使用一行观测数据更新参数估计
        
        参数:
        a: 系数矩阵A的一行，即一个观测的特征向量，形状为(n_params,)
        b_scalar: 目标向量b中对应的元素
        
        返回:
        error: 当前观测的预测误差
        """
        a = np.asarray(a).flatten()
        
        # 如果尚未初始化，根据输入特征维度初始化
        if not self.initialized:
            self._initialize(len(a))
        
        # 计算当前预测误差
        y_pred = np.dot(a, self.theta)
        error = b_scalar - y_pred
        
        # 计算增益
        Pa = np.dot(self.P, a)
        denominator = self.lambda_ + np.dot(a, Pa)
        gain = Pa / denominator
        
        # 更新参数
        self.theta = self.theta + gain * error
        
        # 更新协方差矩阵
        self.P = (self.P - gain.reshape(-1, 1) @ a.reshape(-1, 1).T @ self.P) / self.lambda_
        
        return error
    
    def solve(self, A, b):
        """
        求解线性最小二乘问题 min ||Ax - b||²
        
        参数:
        A: 系数矩阵，形状为(m, n)，其中m是观测数量，n是参数数量
        b: 目标向量，形状为(m,) 或 (m, 1)
        
        返回:
        x: 最小二乘解，形状为(n,)
        residuals: 残差平方和
        rank: 矩阵A的秩(估计值)
        s: 奇异值(在RLS中不直接计算，返回None)
        """
        A = np.asarray(A)
        b = np.asarray(b).flatten()
        
        m, n = A.shape
        
        # 重置状态
        self.initialized = False
        
        # 逐行处理观测数据
        for i in range(m):
            self.update(A[i], b[i])
        
        # 计算残差
        residuals = np.sum((np.dot(A, self.theta) - b) ** 2)
        
        # 为了与np.linalg.lstsq接口兼容
        residuals_array = np.array([residuals])
        
        return self.theta, residuals_array, min(m, n), None
    
    def get_solution(self):
        """获取当前参数估计"""
        if not self.initialized:
            raise ValueError("模型尚未初始化，请先调用solve或update")
        return self.theta.copy()