import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
from matplotlib.animation import FuncAnimation
import math
from matplotlib.widgets import Slider
from matplotlib.lines import Line2D
from scipy.optimize import minimize

# 시스템 파라미터
L1 = 0.042 # mm (파란색 링크)
L2 = 0.250  # mm (빨간색 링크)
L3 = 0.250  # mm (검정색 링크)
L4 = 0.050  # mm (초록색 링크)
L4_end = 0.250  # mm (초록색 링크의 연장)
x0 = 0
y0 = 0

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
text = plt.text(1.05, 0.7, '', transform=ax.transAxes)

# distance
def distance(x1, x2, y1, y2):
    dist = math.sqrt(abs(x2-x1)**2+abs(y2-y1)**2)
    return dist

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

def end_effector_position(thetas, l1):
    theta_knee, theta_hip = thetas
    theta_knee = np.radians(theta_knee)
    theta_hip = np.radians(theta_hip)
    
    # Joint positions 계산
    x0, y0 = 0, 0  # Origin
    x1, y1 = l1 * -np.sin(theta_knee + theta_hip), l1 * -np.cos(theta_knee + theta_hip) 
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

        return x1, y1, x2, y2, x3, y3, x4, y4, x5, y5
    else:
        return None, None, None, None, None, None, None, None, None, None

x1_0, y1_0, x2_0, y2_0, x3_0, y3_0, x4_0, y4_0, x5_0, y5_0 = end_effector_position([theta_knee_init, theta_hip_init], L1)  # Hip 모터 각도
target_x, target_y = x5_0, y5_0  # 초기 목표 위치 설정

# 각도 저장 리스트
theta_hip_list = []
theta_knee_list = []


def objective_function(thetas, target_x, target_y, l1):
    
    x1, y1, x2, y2, x3, y3, x4, y4, x5, y5 = end_effector_position(thetas, l1)

    current_x = x5
    current_y = y5

    # 목표 위치까지의 거리 계산
    dist = distance(current_x, target_x, current_y, target_y)
    
    return dist

# 경로 그리기
real_points_x = []
real_points_y = []
scatter = ax.scatter(real_points_x, real_points_y, s=10, color = 'gray', alpha = 0.3)

initial_guess = [theta_knee_init, theta_hip_init]  # 초기 추측값

# 애니메이션 업데이트 함수
def update(frame):
    global x0, real_points_x, real_points_y

    l1 = sliders['L1'].val
    result = minimize(objective_function, initial_guess, args=(x_points[frame], y_points[frame], l1), method='Nelder-Mead')
    
    x1, y1, x2, y2, x3, y3, x4, y4, x5, y5 = end_effector_position(result.x, l1)

    real_points_x.append(x5)
    real_points_y.append(y5)
    scatter.set_offsets(np.c_[real_points_x, real_points_y])

    link1.set_data([x0, x1], [y0, y1])
    link2.set_data([x0, x2], [y0, y2])
    link3.set_data([x1, x3], [y1, y3])
    link4.set_data([x2, x4], [y2, y4])
    link4_end.set_data([x2, x5], [y2, y5])

    # 각도를 리스트에 저장
    theta_knee_list.append(round(np.deg2rad(result.x[0]),6))
    theta_hip_list.append(round(np.deg2rad(result.x[1]),6))
    text.set_text('x position:{:.6f}\ny position:{:.6f}\ntheta_knee:{:.6f}\ntheta_hip:{:.6f}'.format(x5, y5, result.x[0], result.x[1]))

    return scatter,

# 입력 처리 함수
def submit_x(text):
    global target_x
    target_x = float(text)
    update_path()

def submit_y(text):
    global target_y
    target_y = float(text)
    update_path()

def update_path():
    global x_points, y_points, ani, real_points_x, real_points_y

    x_points, y_points = generate_bezier_points(x5_0, y5_0, target_x, target_y, 250)
    
    real_points_x = []
    real_points_y = []
    theta_knee_list = []
    theta_hip_list = []

    # 기존 애니메이션 중지 및 새로운 애니메이션 객체 생성
    ani.event_source.stop()
    ani = FuncAnimation(fig, update, frames=range(len(x_points)), blit=False, interval=30, repeat=False)
    ani.event_source.start()
    
    # 애니메이션이 끝났을 때 각도 출력
    ani._stop = print_angles()

def print_angles():
    print("Theta Knee Angles:", theta_knee_list)
    print("Theta Hip Angles:", theta_hip_list)


# 사용자 입력 인터페이스
text_box_x = TextBox(plt.axes([0.15, 0.05, 0.3, 0.075]), 'Target X:')
text_box_x.on_submit(submit_x)
text_box_y = TextBox(plt.axes([0.6, 0.05, 0.3, 0.075]), 'Target Y:')
text_box_y.on_submit(submit_y)

# 슬라이더 설정 및 생성
axcolor = 'lightgoldenrodyellow'
sliders = {
    'L1': Slider(plt.axes([0.1, 0.15, 0.75, 0.03], facecolor=axcolor), 'L1', 0.015, 0.045, valinit=L1)
}

# 초기 경로 생성 및 애니메이션 설정
x_points, y_points = generate_bezier_points(x5_0, y5_0, target_x, target_y, 30)

ani = FuncAnimation(fig, update, frames=range(len(x_points)), blit=False, interval=30)
ani.event_source.stop()  # 초기에는 애니메이션을 멈춘 상태로 시작

plt.axis('equal')
plt.show()
