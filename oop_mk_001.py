import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import time
import os
start_time = time.time()

def vec3(x,y,z): return np.array([x,y,z])
def clearConsole(): os.system('cls' if os.name == 'nt' else 'clear')

def rotationZ (theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
        ])

def rotationX (theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
        ])

def rotationY (theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)], 
        ])

class Meha:
    def __init__(self, position, matrix):
        self.OldPosition = position
        self.NewPosition = position
        self.OldMatrix = matrix
        self.NewMatrix = matrix
        self.OldFi = 0
        self.NewFi = 0
        self.legs = {}
        
    def addLeg(self, key, joint_start, angles_leg):
        self.legs[key] = LEG(self, joint_start, angles_leg)

    def getStatus(self):
        offs = 10 * ' '
        status = ''
        for legId in self.legs:
            status+=f'\n\n{30 * "-"}'
            cnf = self.legs[legId].config
            for param in cnf:
                status+=f'\n\n{str.upper(legId)} {param}:'
                if param == 'JOINTS':
                    for key in cnf[param]:
                        status+=f'\n{offs}{key}: {np.round(cnf[param][key]+self.NewPosition, 3)}'
                else:
                    for key in cnf[param]:
                        status+=f'\n{offs}{key}: {np.round(cnf[param][key], 3)}'
        print(status)


    def updateStatus(self):
        clearConsole()
        self.getStatus()

    def draw(self, isAnimate = True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)
        ax.set_zlim(-1000, 1000)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        lines = []
        for _ in range(7):
            line, = ax.plot([], [], [], 'o-', lw=2)
            lines.append(line)
        start_time = time.time() + 0.23
        rows = 1000
        trajectory = []
        # k = 0
        

        def update(frame):  
            t = isAnimate*(time.time() - start_time)
            
            self.OldPosition = self.NewPosition 
            self.NewPosition = 50*vec3(np.cos(t), np.sin(t),0)
            dS = self.NewPosition - self.OldPosition
            dS = np.linalg.inv(self.OldMatrix) @ dS
 
            self.OldMatrix = self.NewMatrix
            self.OldFi = self.NewFi
            dFi = np.sin(t)*np.pi/8 - self.OldFi
            self.NewFi = self.OldFi + dFi
            dT = rotationX(dFi/2)     
            self.NewMatrix = self.OldMatrix @ dT
            
            x0,y0,z0 = self.NewPosition
            lines[0].set_data([x0], [y0])
            lines[0].set_3d_properties([z0])

            for i, id in enumerate(self.legs):
                leg = self.legs[id]   
                leg.move(dS, dT)
                trajectory.append((leg.ANGLES))
                np.save("motor_positions_log.npy", np.array(trajectory))
                points = {'x': [],'y': [],'z': []}
                for j, joint in enumerate(leg.JOINTS):
                    
                    x,y,z = self.NewPosition + self.NewMatrix @ joint
                    points['x'].append(x)
                    points['y'].append(y)
                    points['z'].append(z)
                    # if id=='R1' and j==3:
                    #     print(frame,':',x,y,z)
 
                # self.updateStatus()
                lines[i+1].set_data(points['x'], points['y'])
                lines[i+1].set_3d_properties(points['z'])
                
            return lines
            
        ani = FuncAnimation(fig, update, frames=10, interval=50, blit=False, repeat=True)
        
        plt.show()
        


class LEG():
    def __init__(self, meha, pos, angles):
        self.meha = meha
        self.pos = pos
        self.isLeft = self.pos[1]>0
        self.BaseT10 = np.linalg.inv(rotationZ(self.isLeft*np.pi) @ rotationX(np.pi))
        self.BaseT21 = np.linalg.inv(rotationX(-np.pi/2) @ rotationZ(-np.pi/2))
        self.BaseT32 = np.eye(3)
        self.lenBonds = [180,330,330]
        self.vecBonds = [vec3(0,0,self.lenBonds[0]), 
                         vec3(self.lenBonds[1],0,0), 
                         vec3(self.lenBonds[2],0,0)]
        self.setupAttributes()
        self.setAngles(*angles)
        self.calcJoints()
        

    def setupAttributes(self):
        self.config = { 
                'JOINTS':                {'J0': 0,'J1': 0,'J2': 0,'J3': 0,},
                'JOINT_VELOCITYS':       {'v1': 0,'v2': 0,'v3': 0,},
                'JOINT_ACCELERATIONS':   {'a1': 0,'a2': 0,'a3': 0,},
                'ANGLES':                {'A1': 0,'A2': 0,'A3': 0,},
                'ANGULAR_VELOCITYS':     {'w1': 0,'w2': 0,'w3': 0,},
                'ANGULAR_ACCELERATIONS': {'e1': 0,'e2': 0,'e3': 0,}
            }
        cnf = self.config
        for param in cnf: self.__setattr__(param, [cnf[param][key] for key in cnf[param]])
        
    def updateParameter(self, param, *args):
        self.__setattr__(param, [*args])
        for i, paramId in enumerate(self.config[param]):
            self.config[param][paramId] = args[i]

    def setAngles(self,a1,a2,a3): 
        self.T10 = rotationZ(a1)
        self.T21 = rotationZ(a2) 
        self.T32 = rotationZ(a3)
        self.updateParameter('ANGLES',a1,a2,a3)

    def calcJoints(self):
        R1 = self.BaseT10 @ self.T10
        R2 = R1 @ self.BaseT21 @ self.T21
        R3 = R2 @ self.BaseT32 @ self.T32
        v1,v2,v3 = self.vecBonds
        j0 = self.pos
        j1 = j0 + R1 @ v1
        j2 = j1 + R2 @ v2
        j3 = j2 + R3 @ v3
        
        self.updateParameter('JOINTS',j0,j1,j2,j3)
    
    def move(self, dS, T):
        self.check_Leg_On_Earth()
        if self.isEarth: 
            self.moveEarth(dS,T)
        else: 
            self.moveAir(dS)

    def check_Leg_On_Earth(self):
        self.isEarth = True

    def moveEarth(self, dS, T):

        a = self.JOINTS[0]
        vec = (self.BaseT10)@(np.linalg.inv(T) @ (self.JOINTS[3] - dS) - a)
        a1,a2,a3 = self.inverseKinematics(vec) 
        self.setAngles(a1,a2,a3)
        self.calcJoints()

    def moveAir(self, dS, dFi):
        a1,a2,a3 = self.inverseKinematics(self.config['JOINTS']['J0'], self.config['JOINTS']['J3']+dS) 
        self.setAngles(0,0,0)
        self.calcJoints()
    
    def inverseKinematics(self, v):
        s1, s2, s3 = self.lenBonds
        A1 = -np.arctan2(v[0],v[1])
        x_prime = np.sqrt((v[0])**2 + (v[1])**2)
        z_prime = v[2] - s1
        d = np.sqrt(x_prime**2 + z_prime**2)
        theta2_part1 = np.arctan2(x_prime, z_prime)
        theta2_part2 = np.arccos((s2**2 + d**2 - s3**2) / (2 * s2 * d))
        A2 = -theta2_part1 - theta2_part2 + np.pi / 2
        A3 = -np.arccos((s3**2 + s2**2 - d**2) / (2 * s3 * s2)) + np.pi
        return A1,A2,A3

R_Angle = 0
L_Angle = np.pi
pos = np.array([0,0,0])
basis = np.eye(3)

meha = Meha(pos, basis)
meha.addLeg('R1', joint_start = [415, -150, 0],  angles_leg = [-np.pi/6, -np.pi/8, np.pi/2]   )
meha.addLeg('R2', joint_start = [0, -250, 0],  angles_leg = [0, -np.pi/8, np.pi/2],        )
meha.addLeg('R3', joint_start = [-415, -150, 0], angles_leg = [np.pi/6, -np.pi/8, np.pi/2]  )
meha.addLeg('L1', joint_start = [415, 150, 0],   angles_leg = [np.pi/6, -np.pi/8, np.pi/2]  )
meha.addLeg('L2', joint_start = [0, 250, 0],   angles_leg = [0, -np.pi/8, np.pi/2],        )
meha.addLeg('L3', joint_start = [-415, 150, 0],  angles_leg = [-np.pi/6, -np.pi/8, np.pi/2 ]  ) 


meha.draw(True)


# 4 : 5.049038105676669 -4.54903810567662 -3.098076211353371
# meha.getStatus()