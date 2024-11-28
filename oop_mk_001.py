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


class Meha:
    def __init__(self, position, basis):
        self.position = position
        self.basis = basis
        self.legs = {}

    def changePosition(self, ds):
        self.position += ds
    
    def changeBasis(fi):
        pass
    
    def addLeg(self, key, pos, angle):
        self.legs[key] = LEG(self, pos, angle)

    def getStatus(self):
        offs = 10 * ' '
        status = ''
        for legId in self.legs:
            status+=f'\n\n{30 * "-"}'
            cnf = self.legs[legId].config
            for param in cnf:
                status+=f'\n\n{str.upper(legId)} {param}:'
                for key in cnf[param]:
                    status+=f'\n{offs}{key}: {np.round(cnf[param][key], 3)}'
        print(status)

    def updateStatus(self):
        clearConsole()
        self.getStatus()

    def draw(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        lines = []
        for _ in range(6):
            line, = ax.plot([], [], [], 'o-', lw=2)
            lines.append(line)

        def update(frame):  
            t = (time.time() - start_time)
            ax.set_xlim(-10+t, 10+t)
            ax.set_ylim(-10, 10)
            ax.set_zlim(-10, 10)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            #self.position = vec3(t, 0,0)
            x0,y0,z0 = self.position
            ax.plot([x0], [y0], [z0], 'ro')

            self.updateStatus()     
            for i, id in enumerate(self.legs):
                leg = self.legs[id]
                leg.animate(t)
                
                points = {'x': [],'y': [],'z': []}
                
                for jointId in leg.config['JOINTS']:
                    x,y,z = leg.config['JOINTS'][jointId]
                    points['x'].append(x)
                    points['y'].append(y)
                    points['z'].append(z)

                lines[i].set_data(points['x'], points['y'])
                lines[i].set_3d_properties(points['z'])
                
            return lines
            
        ani = FuncAnimation(fig, update, frames=1, interval=50, blit=False, repeat=True)
        plt.show()
        


class LEG():
    def __init__(self, meha, pos, angle):
        self.meha = meha
        self.pos = pos
        self.BaseT10 = np.linalg.inv(rotationZ(angle) @ rotationX(np.pi))
        self.BaseT21 = np.linalg.inv(rotationX(-np.pi/2) @ rotationZ(-np.pi/2))
        self.BaseT32 = np.eye(3)
        self.shoulders = [vec3(0,0,2), vec3(3,0,0), vec3(3,0,0)]
        self.setup()
        self.rotate_All_Sustavs(0,0,0)
        self.calculate_Joints_Relative_Meha()

    def setup(self):
        self.config = { 
                'JOINTS':                {'J0': 0,'J1': 0,'J2': 0,'J3': 0,},
                'JOINT-VELOCITYS':       {'v1': 0,'v2': 0,'v3': 0,},
                'JOINT-ACCELERATIONS':   {'a1': 0,'a2': 0,'a3': 0,},
                'ANGLES':                {'A1': 0,'A2': 0,'A3': 0,},
                'ANGULAR-VELOCITYS':     {'w1': 0,'w2': 0,'w3': 0,},
                'ANGULAR-ACCELERATIONS': {'e1': 0,'e2': 0,'e3': 0,}
            }
    
    def updateParametr(self, param, *args):
        for i, paramId in enumerate(self.config[param]):
            self.config[param][paramId] = args[i]

    def rotate_All_Sustavs(self,a1,a2,a3):
        a1, a2, a3 = a1, a2, a3
        self.T10 = rotationZ(a1)
        self.T21 = rotationZ(a2) 
        self.T32 = rotationZ(a3)
        self.updateParametr('ANGLES',a1,a2,a3)

    def calculate_Joints_Relative_Meha(self):
        R1 = self.BaseT10 @ self.T10
        R2 = R1 @ self.BaseT21 @ self.T21
        R3 = R2 @ self.BaseT32 @ self.T32
        j0 = self.pos + self.meha.position
        j1 = j0 + R1 @ self.shoulders[0]
        j2 = j1 + R2 @ self.shoulders[1]
        j3 = j2 + R3 @ self.shoulders[2]
        self.updateParametr('JOINTS',j0,j1,j2,j3)
    
    def animate(self, time):
        self.check_Leg_On_Earth()
        if self.isEarth: self.moveEarth()
        else: self.moveAir()
        self.calculate_Joints_Relative_Meha()

    def check_Leg_On_Earth(self):
        self.isEarth = True

    def moveEarth(self, dS=0, dFi=0):
        a = np.sin(time.time())
        a1, a2, a3 = np.pi/6, -np.pi/6, -np.pi/6
        self.rotate_All_Sustavs(a1,a2,a3)

    def moveAir(self, dS=0, dFi=0):
        a = np.sin(time.time())
        a1, a2, a3 = np.pi/6*a, np.pi/6*a + np.pi/6,  np.pi/6*a + np.pi/6
        self.rotate_All_Sustavs(a1,a2,a3)
    
    def inverseKinematics(self, p_start, p_end):
        s1, s2, s3 = 2, 3, 3 #переделать
        x0, y0, z0 = p_start
        x1, y1, z1 = p_end

        A1 = np.arctan2(x0-x1,y0-y1)
        x_prime = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        z_prime = z0 - z1 - s1
        d = np.sqrt(x_prime**2 + z_prime**2)
        theta2_part1 = np.arctan2(x_prime, z_prime)
        theta2_part2 = np.arccos((s2**2 + d**2 - s3**2) / (2 * s2 * d))
        A2 = -theta2_part1 - theta2_part2 + np.pi / 2
        A3 = -np.arccos((s3**2 + s2**2 - d**2) / (2 * s3 * s2)) + np.pi
        self.updateParametr('ANGLES', A1, A2, A3)

    
R_Angle = 0
L_Angle = np.pi
pos = np.array([0,0,0])
basis = np.eye(3)

meha = Meha(pos, basis)
meha.addLeg('R1',[3, -1, 0],   R_Angle)
meha.addLeg('R2',[0, -2, 0],   R_Angle)
meha.addLeg('R3',[-3, -1, 0],  R_Angle)
meha.addLeg('L1',[3, 1, 0],  L_Angle)
meha.addLeg('L2',[0, 2, 0],  L_Angle)
meha.addLeg('L3',[-3, 1, 0], L_Angle) 


meha.draw()
meha.getStatus()






