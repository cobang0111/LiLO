import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import minimize
import scipy.io

# 거리 계산 함수
def distance(x1, x2, y1, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# 베지어 곡선 포인트를 생성하는 함수
def generate_bezier_points(start_x, start_y, end_x, end_y, n):
    P0 = np.array([start_x, start_y])
    P1 = np.array([(2*start_x + end_x) / 3, start_y])  # First control point
    P2 = np.array([(start_x + 2*end_x) / 3, end_y])    # Second control point
    P3 = np.array([end_x, end_y])
    
    t = np.linspace(0, 1, n)
    bezier_points = np.array([(1-t)**3 * P0 + 3*(1-t)**2 * t * P1 + 3*(1-t) * t**2 * P2 + t**3 * P3 for t in t])
    
    x_points = bezier_points[:, 0]
    y_points = bezier_points[:, 1]
    
    return x_points, y_points

def find_intersection(x1, y1, r1, x2, y2, r2):
    d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    if d > r1 + r2:
        return None  # 교점이 없음
    if d < abs(r1 - r2):
        return None  # 원이 포함됨
    if d == 0 and r1 == r2:
        return None  # 원이 일치함
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(r1**2 - a**2)
    x3 = x1 + a * (x2 - x1) / d
    y3 = y1 + a * (y2 - y1) / d
    x4_1 = x3 + h * (y2 - y1) / d
    y4_1 = y3 - h * (x2 - x1) / d
    x4_2 = x3 - h * (y2 - y1) / d
    y4_2 = y3 + h * (x2 - x1) / d
    return (x4_1, y4_1), (x4_2, y4_2)

def is_valid_intersection(x1, y1, x2, y2, x3, y3):
    cross_product = (x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)
    return cross_product > 0

def end_effector_position(thetas, l1):
    theta_knee, theta_hip = thetas
    theta_knee = np.radians(theta_knee)
    theta_hip = np.radians(theta_hip)
    
    x0, y0 = 0, 0  # Origin
    x1, y1 = l1 * -np.sin(theta_knee + theta_hip), l1 * -np.cos(theta_knee + theta_hip) 
    x2, y2 = L2 * -np.sin(theta_hip), L2 * -np.cos(theta_hip) 

    intersections = find_intersection(x1, y1, L3, x2, y2, L4)
    if intersections:
        (x4_1, y4_1), (x4_2, y4_2) = intersections
        if is_valid_intersection(x1, y1, x2, y2, x4_1, y4_1):
            x3, y3 = x4_2, y4_2
        else:
            x3, y3 = x4_1, y4_1
        x4, y4 = x3, y3

        dx, dy = x4 - x2, y4 - y2
        distance = np.sqrt(dx**2 + dy**2)
        if distance != 0:
            dx, dy = dx / distance, dy / distance
        x5, y5 = x2 - dx * L4_end, y2 - dy * L4_end

        return x1, y1, x2, y2, x3, y3, x4, y4, x5, y5
    else:
        return None, None, None, None, None, None, None, None, None, None

def objective_function(thetas, target_x, target_y, l1):
    x1, y1, x2, y2, x3, y3, x4, y4, x5, y5 = end_effector_position(thetas, l1)
    if x5 is None or y5 is None:
        return float('inf')
    current_x = x5
    current_y = y5

    dist = distance(current_x, target_x, current_y, target_y)
    
    return dist

# 시스템 파라미터
L1_0 = 0.045  # m (파란색 링크)
L2 = 0.250  # m (빨간색 링크)
L3 = 0.250  # m (검정색 링크)
L4 = 0.050  # m (초록색 링크)
L4_end = 0.250  # m (초록색 링크의 연장)
x0 = 0
y0 = 0

# 초기 각도 설정
theta_knee_init = -90  # Knee 모터 각도
theta_knee_init_rad = np.deg2rad(theta_knee_init)
theta_hip_init = -45  # Hip 모터 각도
x1_0, y1_0, x2_0, y2_0, x3_0, y3_0, x4_0, y4_0, x5_0, y5_0 = end_effector_position([theta_knee_init, theta_hip_init], L1_0)
print("x5_0, y5_0 = ", x5_0, y5_0)

theta1_init = np.rad2deg(np.arctan2(y1_0, x1_0) - np.arctan2(y2_0, x2_0))

print(theta1_init)

final_target_y = -0.25
vec_length = 400
#final_target_l1_rot=0.5 #2mm

# 각도 저장 리스트
theta_hip_list = []
theta_knee_list = []
theta_l1_list = []
res_x = []
res_y1 = []
res_y2 = []

x_points, y_points = generate_bezier_points(x5_0, y5_0, x5_0, final_target_y, vec_length)



initial_guess = [theta_knee_init, theta_hip_init]

for i in range(vec_length):
    #l1 = (l1_theta_points[i])/1000 + L1_0
    #print(l1)
    result = minimize(objective_function, initial_guess, args=(x_points[i], y_points[i], L1_0), method='Nelder-Mead')
    
    x1, y1, x2, y2, x3, y3, x4, y4, x5, y5 = end_effector_position(result.x, L1_0)

    # 각도를 리스트에 저장
    theta_knee = np.deg2rad(result.x[0])
    theta1 = np.arctan2(y1, x1) - np.arctan2(y2, x2)
    
    theta_knee_list.append(round(theta1-np.deg2rad(theta1_init), 6))
    theta_hip_list.append(-round(np.deg2rad(result.x[1]-theta_hip_init),6))

    initial_guess = result.x  # Use the last result as the initial guess for the next step

theta_knee_rad = theta_knee_list + theta_knee_list[::-1] + [0 for i in range(vec_length)]
theta_hip_rad = theta_hip_list + theta_hip_list[::-1] + [0 for i in range(vec_length)]
theta_l1_rad = [0 for _ in range(len(theta_hip_rad))]
print(len(theta_knee_rad), len(theta_hip_rad), len(theta_l1_rad))


t = [0.002*i for i in range(vec_length*3)]

print(len(theta_knee_rad), len(theta_hip_rad), len(theta_l1_rad))
plt.plot(t, theta_knee_rad)
plt.title("t-knee")
plt.show()

plt.plot(t, theta_hip_rad)
plt.title("t-hip")
plt.show()

#plt.plot(t1, theta_l1_rad)
plt.plot(t, theta_l1_rad)
plt.title("t-l1_theta")
plt.show()


# MATLAB 파일로 저장
scipy.io.savemat('target_theta_vector.mat', {'theta_knee_rad': theta_knee_rad, 'theta_hip_rad': theta_hip_rad, 'theta_l1_rad': theta_l1_rad})