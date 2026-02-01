import numpy as np
import control

# URDF参数

M=0.6 #底盘质量
m=1.8 #轮子质量
l=0.035 #力臂
g=9.81
I =0.011895 #底盘转动惯量
b=0.02 #阻尼系数
r=0.06

Mt=M+m
Jt=I+m*(l**2)
delta=Mt*Jt - (m*l)**2

#状态空间矩阵
A = np.array([
    [0, 1, 0, 0],
    [0, -(Jt*b)/delta, -(m**2 * g * l**2)/delta, 0],
    [0, 0, 0, 1],
    [0, -(m*l*b)/delta, (Mt*m*g*l)/delta, 0]
])

B = np.array([
    [0],
    [Jt/(delta*r)],   
    [0],
    [-(m*l)/(delta*r)]   
])

Q = np.diag([
    30,    # 位置权重 
    6.0,      # 速度权重
    400,    # 角度权重 /800
    8.0      # 角速度权重
])

R = 8 # 控制输入权重

K, S, E = control.lqr(A, B, Q, R)

print("--- LQR 计算结果 ---")
print(f"反馈增益 K: \n{K}")
print(f"\n闭环极点 (确保实部都为负): \n{E}")


print("\n--- 实现参考 ---")
print(f"double base_effort = -({K[0,0]:.4f} * pos_error + {K[0,1]:.4f} * v_lqr_ + {K[0,2]:.4f} * pitch_lqr_ + {K[0,3]:.4f} * omega_lqr_);")

print(f"  k1: {K[0,0]:.4f}\n  k2: {K[0,1]:.4f} \n  k3: {K[0,2]:.4f} \n  k4: {K[0,3]:.4f}")