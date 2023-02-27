from ctypes.wintypes import SIZE
from platform import java_ver
import numpy as np
import math
import matplotlib.pyplot as plt

# 定义变量
m = .041
s = HERO_BULLET_SIZE: 0.0013854423366
# s = 0.005
# m = 0.0032
# s = 0.00022686
M = 16. / 340.

x = [0]
y = [5000]
theta = [30/360*2*math.pi]
v = [20]
time_f = [0]

x_result = []
y_result = []
theta_result = []
v_result = []

alpha = 0
h = 0.01  # 步长在这里
g = 9.8
angle = []
v1 = []  # 加速度
time = []
cnt = 0

# 读入目标点
x_target = 10.9238
y_target = 1.2

# 初始化


def init(h_new, x_new, y_new, theta_new, v_new):
    '''
    description:用于初始化初始值
    '''
    global x, y, theta, v, h, cnt
    h = h_new
    x = [x_new]
    y = [y_new]
    theta = [theta_new]
    v = [v_new]
    cnt = 0

# 定义x导的微分方程


def x_deriv(v_f, theta_f):
    x1 = v_f * math.cos(theta_f)
    return x1
# 定义v导的微分方程


def y_deriv(v_f, theta_f):
    y1 = v_f * math.sin(theta_f)
    return y1
# 定义V导的微分方程


def v_deriv(v_f, theta_f, y_f):
    global alpha, g, m
    a = np.array([[(v_f / 340)**2, v_f / 340, 1]])
    b = np.array([[0.0002 , 0.0038 , 0.1582 ],
                  [-0.0022, -0.0132, -0.8520],
                  [0.0115 , -0.0044, 1.9712]])
    c = np.array([[alpha**2],
                  [alpha],
                  [1]])
    Cx = np.matmul(np.matmul(a, b), c)
    ro = 1.225*math.e**(-0.00015*y_f)
    D = 0.5*ro*v_f**2*s*Cx[0][0]
    v1 = -D/m-g*math.sin(theta_f)
    return v1
# 定义θ导的微分方程


def theta_deriv(v_f, theta_f, y_f):
    global alpha, g, m
    a = np.array([[(v_f/340)**2, v_f/340, 1]])
    b = np.array([[-0.026],
                  [0.0651],
                  [0.4913]])
    Cy = np.matmul(a, b)*alpha
    ro = 1.225*math.e**(-0.00015*y_f)
    L = 0.5*ro*v_f**2*s*Cy[-1][-1]
    theta1 = -g*math.cos(theta_f)/v_f+L/(m*v_f)
    return theta1
# 定义全部导数


def param_deriv(v_f, theta_f, y_f):
    '''
    description:同时对x,y,v,theta进行求导
    '''
    x1 = x_deriv(v_f, theta_f)
    y1 = y_deriv(v_f, theta_f)
    v1 = v_deriv(v_f, theta_f, y_f)
    theta1 = theta_deriv(v_f, theta_f, y_f)
    return x1, y1, v1, theta1

# 定义Runge-Kutta方程


def iter(step, x0, y0, v0, theta0, kx1, ky1, kv1, ktheta1):
    '''
    description:龙格库塔的K的一次更新
    '''
    x1 = x0 + 0.5*step*kx1
    y1 = y0 + 0.5*step*ky1
    v1 = v0 + 0.5*step*kv1
    theta1 = theta0 + 0.5*step*ktheta1
    return x1, y1, v1, theta1


def RungeKutta(step):
    '''
    description:龙格库塔的一次更新
    '''
    v0, x0, y0, theta0 = v[-1], x[-1], y[-1], theta[-1]
    
    kx1, ky1, kv1, ktheta1 = param_deriv(v0, theta0, y0)
    x1, y1, v1, theta1 = iter(step, x0, y0, v0, theta0, kx1, ky1, kv1, ktheta1)
    
    kx2, ky2, kv2, ktheta2 = param_deriv(v1, theta1, y1)
    x2, y2, v2, theta2 = iter(step, x0, y0, v0, theta0, kx2, ky2, kv2, ktheta2)
    
    kx3, ky3, kv3, ktheta3 = param_deriv(v2, theta2, y2)
    x3, y3, v3, theta3 = iter(step, x0, y0, v0, theta0,
                              2*kx3, 2*ky3, 2*kv3, 2*ktheta3)
    
    kx4, ky4, kv4, ktheta4 = param_deriv(v3, theta3, y3)

    v_t = v0+step/6*(kv1 + 2*kv2 + 2*kv3 + kv4)
    theta_t = theta0+step/6*(ktheta1+2*ktheta2+2*ktheta3+ktheta4)
    x_t = x0+step/6*(kx1 + 2*kx2 + 2*kx3 + kx4)
    y_t = y0+step/6*(ky1 + 2*ky2 + 2*ky3 + ky4)
    print("now pos: " + str(x_t) + "  " + str(y_t))
    return v_t, theta_t, x_t, y_t

# 计算弹道


def calculate(step, x_new, y_new, theta_new, v_new):
    '''
    description: 判断落地为y是否大于0,落地后跳出循环，一次循环即一次迭代。
                 先同时做一次龙格库塔,然后依次更新。
                 用于定步长计算。
    '''
    global x, y, v, theta, M, h, cnt, x_target, y_target
    init(step, x_new, y_new, theta_new, v_new)
    
    min_dist = 99999999
    while(y[-1] > 0):
        v_temp, theta_temp, x_temp, y_temp = RungeKutta(step)
        x.append(x_temp)
        y.append(y_temp)
        v.append(v_temp)
        theta.append(theta_temp)
        M = v[-1]/340
        
        min_dist = min(min_dist, math.sqrt( (x_target - x[-1])**2 + (y_target - y[-1])**2) )
        
        if (x[-1] > x_target) and (y[-1] > y_target):
            # print("Hit speed = " + str(v[-1]) + "m/s")
            return [min_dist, 1]
        elif (x[-1] > x_target) and (y[-1] < y_target):
            # print("Hit speed = " + str(v[-1]) + "m/s")
            return [min_dist, 0]
        cnt += 1
        
    # print('命中用时：'+str(round(cnt*step*10)/10)+'s')
    # print("Hit speed = " + str(v[-1]) + "m/s")
    return [min_dist, 0]


# 定步长龙格库塔
def RungeKutta_fixed(step, x_new, y_new, theta_new, v_new):
    global x_result, y_result, v_result, theta_result, h
    ret = calculate(step, x_new, y_new, theta_new, v_new)
    
    # 记录一倍步长
    x_result = x
    y_result = y
    theta_result = theta
    v_result = v
    # 生成时间序列
    for i in range(0, len(x)):
        time.append(h*i)
    return ret
   
    
def draw():
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    # 画图
    # f, ax = plt.subplots(2, 2)

    # 速度曲线
    # plt.subplot(2, 4, 2)
    # plt.plot(time, v_result, markersize=1, color=(0.5, 0., 0.))
    # plt.title("导弹速度曲线", fontsize=20)
    # plt.xlabel("时间/s", fontsize=20)
    # plt.ylabel("速度/(m/s)", fontsize=20)
    # plt.xlim(xmax=7, xmin=0)
    # plt.ylim(ymax=25, ymin=0)

    # 角度曲线
    # plt.subplot(2, 4, 5)
    # for i in range(0, len(theta_result)):
    #     angle.append(theta_result[i]/(2*math.pi)*360)
    # plt.plot(time, angle, markersize=1, color=(0.5, 0., 0.))
    # plt.title("θ角度曲线", fontsize=20)
    # plt.xlabel("时间/s", fontsize=20)
    # plt.ylabel("θ/°", fontsize=20)
    # plt.xlim(xmax=7, xmin=0)

    # 加速度曲线
    # plt.subplot(2, 4, 6)
    # for i in range(0, len(theta_result)):
    #     v1.append(v_deriv(v_result[i], theta_result[i], y_result[i]))
    # plt.plot(time, v1, markersize=1, color=(0.5, 0., 0.))
    # plt.title("加速度曲线", fontsize=20)
    # plt.xlabel("时间/s", fontsize=20)
    # plt.ylabel("加速度a/(m/s2)", fontsize=20)
    # plt.xlim(xmax=7, xmin=0)

    # 轨迹曲线
    
    # plt.subplot(1, 2, 2)
    plt.plot(x_result, y_result, markersize=1, color=(0.5, 0., 0.))
    plt.plot([y_target] * 15)
    
    plt.title("导弹弹道轨迹", fontsize=20)
    plt.xlabel("距离/m", fontsize=20)
    plt.ylabel("高度/m", fontsize=20)
    plt.xlim(xmax=12, xmin=0)
    plt.ylim(ymax=12, ymin=0)

    plt.show()
    
    


# 主函数（初始值需要在这里重新设置）
# h, x, y, theta, v
l = 0.
r = 90.
bst_deg = 45.

dist = 9999999
for i in range(1, 2):
    print("**********")
    now_deg = (l + r) / 2.
    res = RungeKutta_fixed(0.01, 0., 0.5, now_deg/360*2*math.pi, 16.)
    
    if (res[1] == 0):
        l = now_deg
    else:
        r = now_deg
    
    
    now_dist = res[0]
    # draw()
    print(now_deg)
    print(now_dist)
    print("**********")
    # print(now_dist)
    if now_dist < dist:
        dist = now_dist
        bst_deg = now_deg

print("Min_dist = " + str(dist))
print("The best angle is " + str(bst_deg) + " degree. ")


'''

'''