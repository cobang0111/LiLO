import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.widgets import TextBox
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from scipy.optimize import minimize

# 시스템 파라미터
L1 = 0.045  # mm (파란색 링크)
L2 = 0.250  # mm (빨간색 링크)
L3 = 0.250  # mm (검정색 링크)
L4 = 0.050  # mm (초록색 링크)
L4_end = 0.250  # mm (초록색 링크의 연장)

# 초기 각도 설정
theta_knee_init = -90  # Knee 모터 각도
theta_hip_init = -45  # Hip 모터 각도

# 그래프 설정
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.15, bottom=0.25)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)

# 링크들 그리기
link1, = plt.plot([], [], 'b-', linewidth=3)
link2, = plt.plot([], [], 'r-', linewidth=3)
link3, = plt.plot([], [], 'k-', linewidth=3)
link4, = plt.plot([], [], 'g-', linewidth=3)
link4_end, = plt.plot([], [], 'g-', linewidth=3)

# 스크롤 바 설정
axcolor = 'lightgoldenrodyellow'
ax_knee = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_hip = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

slider_knee = Slider(ax_knee, 'Knee Angle', -180, 180, valinit=theta_knee_init)
slider_hip = Slider(ax_hip, 'Hip Angle', -180, 180, valinit=theta_hip_init)

text = plt.text(1.05, 0.7, '', transform=ax.transAxes)

def find_intersection(x1, y1, r1, x2, y2, r2):
    # 두 원의 교점을 찾는 함수
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
    # 교점이 유효한지 확인하는 함수
    cross_product = (x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)
    return cross_product > 0

def end_effector_position(thetas):
    theta_knee, theta_hip = thetas
    theta_knee = np.radians(theta_knee)
    theta_hip = np.radians(theta_hip)
    
    # Joint positions 계산
    x0, y0 = 0, 0  # Origin
    x1, y1 = L1 * -np.sin(theta_knee + theta_hip), L1 * -np.cos(theta_knee + theta_hip) 
    x2, y2 = L2 * -np.sin(theta_hip), L2 * -np.cos(theta_hip) 

    # 두 원의 교점 계산
    intersections = find_intersection(x1, y1, L3, x2, y2, L4)
    if intersections:
        (x4_1, y4_1), (x4_2, y4_2) = intersections
        # L2와 L3가 교차하지 않는 교점을 선택
        if is_valid_intersection(x1, y1, x2, y2, x4_1, y4_1):
            x3, y3 = x4_2, y4_2
        else:
            x3, y3 = x4_1, y4_1
        x4, y4 = x3, y3

        # End effector 계산
        dx, dy = x4 - x2, y4 - y2
        distance = np.sqrt(dx**2 + dy**2)
        if distance != 0:
            dx, dy = dx / distance, dy / distance
        x5, y5 = x2 - dx * L4_end, y2 - dy * L4_end

        return x5, y5
    else:
        return None, None


def update(val):
    theta_knee = np.radians(slider_knee.val)
    theta_hip = np.radians(slider_hip.val)
    

    # Joint positions 계산
    x0, y0 = 0, 0  # Origin
    x1, y1 = L1 * -np.sin(theta_knee + theta_hip), L1 * -np.cos(theta_knee + theta_hip) 
    x2, y2 = L2 * -np.sin(theta_hip), L2 * -np.cos(theta_hip) 

    # 두 원의 교점 계산
    intersections = find_intersection(x1, y1, L3, x2, y2, L4)
    if intersections:
        (x4_1, y4_1), (x4_2, y4_2) = intersections
        # L2와 L3가 교차하지 않는 교점을 선택
        if is_valid_intersection(x1, y1, x2, y2, x4_1, y4_1):
            x3, y3 = x4_2, y4_2
        else:
            x3, y3 = x4_1, y4_1
        x4, y4 = x3, y3

        # End effector 계산
        dx, dy = x4 - x2, y4 - y2
        distance = np.sqrt(dx**2 + dy**2)
        if distance != 0:
            dx, dy = dx / distance, dy / distance
        x5, y5 = x2 - dx * L4_end, y2 - dy * L4_end

        # 링크들 업데이트
        link1.set_data([x0, x1], [y0, y1])
        link2.set_data([x0, x2], [y0, y2])
        link3.set_data([x1, x3], [y1, y3])
        link4.set_data([x2, x4], [y2, y4])
        link4_end.set_data([x2, x5], [y2, y5])

        text.set_text('x position:{:.6f}\ny position:{:.6f}'.format(x5, y5))

        fig.canvas.draw_idle()
    else:
        print("Error: 교점이 없거나 원이 교차합니다.")

slider_knee.on_changed(update)
slider_hip.on_changed(update)

update(0)  # 초기 상태 업데이트
plt.show()