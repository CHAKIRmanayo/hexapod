import numpy as np
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class BONE():
  def __init__(self, staticOffset, length, start):
    self.offset = staticOffset #offset of the radius vector of the bone[i] relative to the motor[i-1]
    self.start = start
    self.length = length


class JOINT():
  def __init__(self, staticRotate, position=0, angle=0):
    self.rotate = staticRotate #angle of rotation of the motor[i] relative to the radius vector of the bone[i-1]
    self.positon = position
    self.angle = angle

  def setAngle(self, angle):
    self.angle = angle


class LINK():
  def __init__(self, joint=None, bone = None):
    self.joint = joint
    self.bone = bone

  def setJoint(self, rot, pos=0, ang=0):
    self.joint = JOINT(rot, pos, ang)
  
  def setBone(self, rot, len, start=0):
    self.bone = BONE(rot, len, start)


class ARMATURE:
  def __init__(self):
    self.start = 0
    self.end = 0
    self.joints = []
    self.bones =  []
    self.matrix = [] #transition matrix 

  def setJoint(self, relativeRotate, position):
    self.joints.add(JOINT(relativeRotate, ))
  
  def addBone(self, relativeRotate, length):
    self.bones.add(BONE(relativeRotate, length, self.start))
    self.start = self.joints[-1]
  
  

  

class CalculateArmature:
  def __init__(self, armature):
    self.arm = armature

  def calc_direct_kinematics(self):
    position = np.array()
  
  def calc_inverse_kinematics(self):

    return 
  
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
# link_lengths = [3, 3, 3]

# hexapod_model(center_mass, motors)

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


link_lengths = [1, 3, 3]

# Траектория движения конечной точки
def generate_trajectory():
    x_vals = []
    y_vals = []
    z_vals = []
    fi = 0
    for t in np.arange(0, np.pi * 2, 0.1):
        x_prime = np.sign(np.cos(t)) * (np.sqrt(np.abs(3 * np.cos(t))))
        y = x_prime * np.cos(fi)
        x = x_prime * np.sin(fi)
        z = np.power(0.5 * (np.sin(t) + 1), 3)
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)

    return np.array(x_vals), np.array(y_vals), np.array(z_vals)

# Обратная кинематика
def inverse_kinematics_single(x1, y1, z1, base_position, link_lengths):
    L1, L2, L3 = link_lengths
    x0, y0, z0 = base_position

    theta1 = np.arctan2(y1 - y0, x1 - x0)
    x_prime = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    z_prime = z0 - z1 - L1
    d = np.sqrt(x_prime**2 + z_prime**2)

    if d > L2 + L3:
        raise ValueError("Target point is unreachable")

    theta2_part1 = np.arctan2(x_prime, z_prime)
    theta2_part2 = np.arccos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
    theta2 = theta2_part1 + theta2_part2 - np.pi / 2

    theta3 = np.pi - np.arccos((L3**2 + L2**2 - d**2) / (2 * L3 * L2))
    return theta1, theta2, -theta3

# Прямая кинематика
def forward_kinematics(theta1, theta2, theta3, base_position, link_lengths):
    L1, L2, L3 = link_lengths
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

    joint0 = np.array(base_position)
    joint1 = joint0 + np.array([0, 0, -L1])
    joint2 = joint1 + T1 @ np.array([0, L2, 0])
    joint3 = joint2 + T1 @ T2 @ np.array([0, L3, 0])

    return joint0, joint1, joint2, joint3

# Анимация движения
def animate_leg(trajectory, link_lengths):
    x_vals, y_vals, z_vals = trajectory
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
      i=0
      for id in ledCoordinates:
        
        base_position = ledCoordinates[id]['position']
        x0, y0, z0 = ledCoordinates[id]['endPosition']
        x, y, z = x_vals[frame] + x0, y_vals[frame] + y0, z_vals[frame] + z0
        theta1, theta2, theta3 = inverse_kinematics_single(x, y, z, base_position, link_lengths)
        joints = forward_kinematics(theta1, theta2, theta3, base_position, link_lengths)

        joint_x = [j[0] for j in joints]
        joint_y = [j[1] for j in joints]
        joint_z = [j[2] for j in joints]

        lines[i].set_data(joint_x, joint_y)
        lines[i].set_3d_properties(joint_z)

        targets[i].set_data([x], [y])
        targets[i].set_3d_properties([z])
        i+=1

      return lines, targets

    ani = FuncAnimation(fig, update, frames=len(x_vals), interval=50, blit=False, repeat=True)
    plt.show()

# Базовая позиция ноги



# Генерация траектории
trajectory = generate_trajectory()

# Анимация
animate_leg(trajectory, link_lengths)