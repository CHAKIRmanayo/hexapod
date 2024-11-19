import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_fixed_aspect_3d(ax, aspect=1):
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d()
    ])
    span = limits[:, 1] - limits[:, 0]
    max_range = max(span) / 2.0

    # Центрируем график
    centers = np.mean(limits, axis=1)
    ax.set_xlim3d([centers[0] - max_range, centers[0] + max_range])
    ax.set_ylim3d([centers[1] - max_range, centers[1] + max_range])
    ax.set_zlim3d([centers[2] - max_range, centers[2] + max_range])

def calculateJoints(motorId):
    theta1, theta2, theta3 = motors[motorId]['angles']
    theta1-=np.pi/2
    rotation_z = np.array([
        [np.cos(theta1), -np.sin(theta1), 0],
        [np.sin(theta1), np.cos(theta1), 0],
        [0, 0, 1]
    ])
    rotation_x = np.array([
        [1, 0, 0],
        [0, np.cos(theta2), -np.sin(theta2)],
        [0, np.sin(theta2), np.cos(theta2)]
    ])
    rotation_x2 = np.array([
        [1, 0, 0],
        [0, np.cos(theta3), -np.sin(theta3)],
        [0, np.sin(theta3), np.cos(theta3)]
    ])
    T1 = rotation_z @ rotation_x
    T2 = rotation_x2

    joint0 = np.array(motors[motorId]['position'])
    joint1 = joint0 - [0, 0, link_lengths[0]]
    joint2 = joint1 + T1 @ np.array([0, link_lengths[1],0])
    joint3 = joint2 + T1 @ T2 @ np.array([0, link_lengths[2], 0])
    return joint0, joint1, joint2, joint3
    

def hexapod_model(center_mass, motors):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(*center_mass, color='red', label='Center of Mass', s=50)

    for id in motors:
        joint0, joint1, joint2, joint3 = calculateJoints(id)
        ax.plot([joint0[0], joint1[0]], [joint0[1], joint1[1]], [joint0[2], joint1[2]], color='blue')
        ax.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], [joint1[2], joint2[2]], color='green')
        ax.plot([joint2[0], joint3[0]], [joint2[1], joint3[1]], [joint2[2], joint3[2]], color='purple')
        #print(joint3)
        ax.scatter(*joint3, color='black', s=20)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    set_fixed_aspect_3d(ax)
    ax.legend()
    ax.set_title("Hexapod Model")
    plt.show()

# Пример входных данных
center_mass = np.array([0, 0, 0])

alpha = np.pi/12
motors = {
    'L1': {
            'angles' : [np.pi, np.pi/12, -np.pi/2],
            'position': [-1, 3, 0]
          },
    'L2': {
            'angles' : [np.pi, np.pi/12, -np.pi/2],
            'position': [-2, 0, 0]
          },
    'L3': {
            'angles' : [np.pi, np.pi/12, -np.pi/2],
            'position': [-1, -3, 0]
          },
    'R1': {
            'angles' : [0, np.pi/12, -np.pi/2],
            'position': [1, 3, 0]
          },
    'R2': {
            'angles' : [0, np.pi/12, -np.pi/2],
            'position': [2, 0, 0]
          },
    'R3': {
            'angles' : [0, np.pi/12, -np.pi/2],
            'position': [1, -3, 0]
          }
}
link_lengths = [1, 3, 3]

hexapod_model(center_mass, motors)

# ОЗК

ledCoordinates = {
    'L1': {
            'position': [-1, 3, 0],
            'endPosition': [-4.67423461,  3,  -3.12132034]
          },
    'L2': {
            'position': [-2, 0, 0],
            'endPosition': [-5.67423461,  0, -3.12132034]
          },
    'L3': {
            'position': [-1, -3, 0],
            'endPosition': [-4.67423461, -3,  -3.12132034]
          },
    'R1': {
            'position': [1, 3, 0],
            'endPosition': [ 4.67423461,  3,  -3.12132034]
          },
    'R2': {
            'position': [2, 0, 0],
            'endPosition': [ 5.67423461,  0, -3.12132034]
          },
    'R3': {
            'position': [1, -3, 0],
            'endPosition': [ 4.67423461, -3,  -3.12132034]
          }
}


def inverseKinematics(ledCoordinates1, link_lengths1):

    L1, L2, L3 = link_lengths1
    for id in ledCoordinates1:
      x0, y0, z0 = ledCoordinates1[id]['position']
      x, y, z = ledCoordinates1[id]['endPosition']

      thet1 = np.atan2(y - y0, x - x0)
  
    # Проекция на плоскость XZ
      x_prime = np.sqrt((x - x0)**2 + (y - y0)**2)
      
      z_prime = z0 - z - L1
      
    # Расстояние до целевой точки
      d = np.sqrt(x_prime**2 + z_prime**2)
      
    # Проверка досягаемости
      if d > L2 + L3:
          raise ValueError("Целевая точка недостижима")
    
    # Угол второго сочленения
      thet2_part1 = np.atan2(x_prime, z_prime)
      thet2_part2 = np.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
      thet2 = thet2_part1 + thet2_part2 - np.pi/2
    
    # Угол третьего сочленения
      thet3 = np.pi - np.acos((L3**2 + L2**2 - d**2) / (2 * L3 * L2))
    
      print(thet1, thet2, thet3)

inverseKinematics(ledCoordinates, link_lengths)