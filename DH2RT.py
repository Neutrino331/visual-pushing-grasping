import numpy as np

def DH_params(theta, d, a, alpha):
    """
    計算DH參數矩陣
    """
    DH = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return DH

def forward_kinematics(thetas, ds, as_, alphas):
    """
    計算機械手臂的正向運動學
    """
    n_joints = len(thetas)
    T = np.eye(4)
    for i in range(n_joints):
        T_i_iplus1 = DH_params(thetas[i], ds[i], as_[i], alphas[i])
        T = np.dot(T, T_i_iplus1)
    return T

# 機械手臂的六個關節
thetas = [np.pi/2, np.pi/2, 0, 0, 0, 0]
ds = [0, 0, 0.15, 0.45, 0.45, 0.15]
as_ = [0, 0, 0, 0, 0, 0]
alphas = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0]

# 計算機械手臂的正向運動學
T = forward_kinematics(thetas, ds, as_, alphas)

# 提取旋轉矩陣和平移矩陣
R = T[:3, :3]
P = T[:3, 3]

print("旋轉矩陣：")
print(R)
print("平移矩陣：")
print(P)