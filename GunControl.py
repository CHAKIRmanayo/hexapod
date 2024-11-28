import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import time
start_time = time.time()

def mix(a,b,t):
    return a*t+ b*(1-t)

class POINT():
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z

def calculateTrajectory():
    T= 3
    dt = 0.01
    x_arr = []
    z_arr = []
    a = 1
    b = a/0.9
    fi = np.arccos(0.8)
    f3 = mix(2*np.pi - fi, 3*np.pi + fi, 0)

    for t in np.arange(0, T, dt):
        
      if 0<=t<T/2:
            aa = (t - T/4) / (T/4)
            x = aa
            z = 0
            
      elif T/2<=t<5*T/8:
            aa = (t - T/2) / (T/8)
            x = 1 + (t - T/2) / (T/4)
            z = aa
            
      elif 5*T/8<=t<7*T/8:
            aa = (T/8 - t + 5*T/8) / (T/12)
            x = aa
            print(aa)
            z = 1

      elif 7*T/8<=t<=T:
            aa = (t - 7*T/8) / (T/8)
            x = -1 - (T-t)/(T/4)
            z = 1 - aa
            # aa = (t - T/2) / (T/2)
            # f = mix(2*np.pi - fi, 3*np.pi + fi, aa)
            # x = -b * np.sign(np.cos(f)) * (np.sqrt(np.abs(np.cos(f))))
            # z = np.power(0.5 * (np.sin(f) + 1), 3) - np.power(0.5 * (np.sin(f3) + 1), 3)
            # print(f-2.*np.pi)

        
      x_arr.append(x)
      z_arr.append(z)
    x_arr.append(-1)
    z_arr.append(0)
    plt.plot(x_arr,z_arr)
    plt.show()
        
def calcT(t1):
    T= 3
    V_z = np.sin(time.time()*0.1)
    a = np.sqrt(1)
    b = a/0.9
    fi = np.arccos(0.8)
    t = t1%T
    if 0<=t<T/2:
            aa = (t - T/4) / (T/4)
            x = aa
            z = 0
            
    elif T/2<=t<5*T/8:
            aa = (t - T/2) / (T/8)
            x = 1 + (t - T/2) / (T/4)
            z = aa
            
    elif 5*T/8<=t<7*T/8:
            aa = (T/8 - t + 5*T/8) / (T/12)
            x = aa
            
            z = 1

    elif 7*T/8<=t<=T:
            aa = (t - 7*T/8) / (T/8)
            x = -1 - (T-t)/(T/4)
            z = 1 - aa
    return x,z

class LEG():
    def __init__(self, link_lengths, relative_angle_leg_basis, pos_start_zero, pos_end_zero, offset_time=0, angle_trajectory=0):
        self.relative_angle_leg_basis = relative_angle_leg_basis
        self.link_lengths = link_lengths
        self.pos_start_zero = pos_start_zero
        self.pos_end_zero = pos_end_zero
        self.offset_time = offset_time
        self.angle_trajectory = angle_trajectory
        self.thetas = np.zeros(3)
        self.joints = np.zeros(4)
        self.inverse_kinematics(pos_start_zero, pos_end_zero)
        # self.forward_kinematics()

    def generate_trajectory(self, time):
        t = time + self.offset_time
        tau, norm = calcT(t)
        # fi = np.pi*np.sin(time*0.1)
        fi = angle_trajektory
        x0, y0, z0 = self.pos_end_zero
        y = y0 + tau * np.sin(fi)
        x = x0 + tau * np.cos(fi) 
        z = z0 + norm
        self.joints[-1] = np.array([x,y,z])
        # print(self.joints[-1])
    
    def inverse_kinematics(self, start = None, end = None):
        L1, L2, L3 = self.link_lengths
        x0, y0, z0 = start or self.joints[0]
        x1, y1, z1 = end or self.joints[-1]

        theta1 = np.arctan2(y1-y0, x1-x0) + self.relative_angle_leg_basis
        x_prime = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        z_prime = z0 - z1 - L1
        d = np.sqrt(x_prime**2 + z_prime**2)

        if d > L2 + L3:
            raise ValueError("Target point is unreachable")
    
        theta2_part1 = np.arctan2(x_prime, z_prime)
        theta2_part2 = np.arccos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
        theta2 = theta2_part1 + theta2_part2 - np.pi / 2
        theta3 = np.arccos((L3**2 + L2**2 - d**2) / (2 * L3 * L2)) - np.pi
        self.thetas = [theta1, theta2, theta3]
        
    
    def forward_kinematics(self, thetas):
        L1, L2, L3 = self.link_lengths
        t1 = self.thetas[0] - thetas[0]
        t2 = self.thetas[1] - thetas[1]
        t3 = self.thetas[2] - thetas[2]

        rotation_z = np.array([
            [np.cos(t1), -np.sin(t1), 0],
            [np.sin(t1), np.cos(t1), 0],
            [0, 0, 1]
        ])
        rotation_x = np.array([
            [1, 0, 0],
            [0, np.cos(t2), -np.sin(t2)],
            [0, np.sin(t2), np.cos(t2)]
        ])
        rotation_x2 = np.array([
            [1, 0, 0],
            [0, np.cos(t3), -np.sin(t3)],
            [0, np.sin(t3), np.cos(t3)]
        ])
        T1 = rotation_z @ rotation_x
        T2 = rotation_x2

        joint0 = np.array(self.pos_start_zero)
        joint1 = joint0 + np.array([0, 0, -L1])
        joint2 = joint1 + T1 @ np.array([0, L2, 0])
        joint3 = joint2 + T1 @ T2 @ np.array([0, L3, 0])
        self.joints = [joint0, joint1, joint2, joint3]





##----------------------------------------------------------------------------------------------------------------------##
Period = 3
halfPeriod = Period/2
linkLength = [1,3,3]
r_angle_relative = np.radians(-90)
l_angle_relative = np.radians(-90)
angle_trajektory = np.radians(90)


def animate_legs(LEGS):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    lines = []
    targets = []

    for _ in range(6):
        line, = ax.plot([], [], [], 'o-', lw=2)
        target, = ax.plot([], [], [], 'ro')
        lines.append(line)
        targets.append(target)

    def update(frame):
      current_time = time.time() - start_time
      
      for i, id in enumerate(LEGS):
        leg = LEGS[id]
        leg.generate_trajectory(current_time)
        leg.inverse_kinematics()
        leg.forward_kinematics()

        joint_x = [j[0] for j in leg.joints]
        joint_y = [j[1] for j in leg.joints]
        joint_z = [j[2] for j in leg.joints]

        lines[i].set_data(joint_x, joint_y)
        lines[i].set_3d_properties(joint_z)

        x,y,z = leg.joints[-1]
        targets[i].set_data([x], [y])
        targets[i].set_3d_properties([z])
        

      return lines, targets
    
    ani = FuncAnimation(fig, update, frames=1, interval=50, blit=False, repeat=True)

    plt.show()


LEGS = {'L1': LEG(linkLength, l_angle_relative, [-1, 3, 0],  [-4.67423461,  3,  -3.12132034], halfPeriod, angle_trajektory),
        'L2': LEG(linkLength, l_angle_relative, [-2, 0, 0],  [-5.67423461,  0, -3.12132034],  0,          angle_trajektory),
        'L3': LEG(linkLength, l_angle_relative, [-1, -3, 0], [-4.67423461, -3,  -3.12132034], halfPeriod, angle_trajektory),
        'R1': LEG(linkLength, r_angle_relative, [1, 3, 0],   [ 4.67423461,  3,  -3.12132034], 0,          angle_trajektory),
        'R2': LEG(linkLength, r_angle_relative, [2, 0, 0],   [ 5.67423461,  0, -3.12132034],  halfPeriod, angle_trajektory),
        'R3': LEG(linkLength, r_angle_relative, [1, -3, 0],  [ 4.67423461, -3,  -3.12132034], 0,          angle_trajektory)
        }

# calculateTrajectory()

# animate_legs(LEGS)

def animate_folding(LEGS, paths):
    """
    Анимация складывания ног робота по заданным траекториям.
    
    LEGS: словарь с объектами ног.
    paths: словарь с траекториями для каждой ноги.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    lines = []
    targets = []

    for _ in range(len(LEGS)):
        line, = ax.plot([], [], [], 'o-', lw=2)
        target, = ax.plot([], [], [], 'ro')
        lines.append(line)
        targets.append(target)

    start_time = time.time()
    
    def update(frame):
        current_time = time.time() - start_time

        for i, (id, leg) in enumerate(LEGS.items()):
            # Достаем путь для текущей ноги
            path = paths[id]
            
            
                # Устанавливаем целевые координаты
                


                # Выполняем кинематические вычисления
                
            leg.forward_kinematics(path)

                # Обновляем положение суставов
            joint_x = [j[0] for j in leg.joints]
            joint_y = [j[1] for j in leg.joints]
            joint_z = [j[2] for j in leg.joints]

            lines[i].set_data(joint_x, joint_y)
            lines[i].set_3d_properties(joint_z)

            x, y, z = leg.joints[-1]
            targets[i].set_data([x], [y])
            targets[i].set_3d_properties([z])

        return lines + targets

    ani = FuncAnimation(fig, update, frames=len(next(iter(paths.values()))), interval=200, blit=False, repeat=False)
    plt.show()


# Пример вызова
paths = {
    'L1': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [3.925, 1.255, 0.9], [3.925, 1.542, 1.876]],
    'L2': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 1.895], [0.0, 0.096, 1.895], [-2.356, 0.096, 1.895]],
    'L3': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [-3.925, 1.255, 0.9], [-3.925, 1.542, 1.876]],
    'R1': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [-3.925, 1.255, 0.9], [-3.925, 1.542, 1.876]],
    'R2': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 1.895], [0.0, 0.096, 1.895], [-2.356, 0.096, 1.895]],
    'R3': [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [3.925, 1.255, 0.9], [3.925, 1.542, 1.876]],
}

# animate_folding(LEGS, paths)

def animate_folding(LEGS, paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    lines = []
    targets = []

    for _ in range(6):
        line, = ax.plot([], [], [], 'o-', lw=2)
        target, = ax.plot([], [], [], 'ro')
        lines.append(line)
        targets.append(target)

    start_time = time.time()

    def update(frame):
        current_time = time.time() - start_time
        for i, (id, path) in enumerate(paths.items()):
            leg = LEGS[id]
            
            # Get current target point from the path
            target_index = frame % len(path)
            target_point = path[target_index]
            
            # Set the leg's end effector to the target point
            
            
            # Perform inverse kinematics and forward kinematics to calculate joint positions
            
            leg.forward_kinematics(target_point)

            # Extract joint positions
            joint_x = [j[0] for j in leg.joints]
            joint_y = [j[1] for j in leg.joints]
            joint_z = [j[2] for j in leg.joints]

            # Update line (leg segments) and target (end effector)
            lines[i].set_data(joint_x, joint_y)
            lines[i].set_3d_properties(joint_z)

            x, y, z = leg.joints[-1]
            targets[i].set_data([x], [y])
            targets[i].set_3d_properties([z])

        return lines + targets

    ani = FuncAnimation(fig, update, frames=len(next(iter(paths.values()))), interval=300, blit=False, repeat=True)
    plt.show()


# Define paths for folding


# Call the animation function
animate_folding(LEGS, paths)
