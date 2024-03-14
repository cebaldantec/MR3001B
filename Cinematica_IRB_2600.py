import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
from spatialmath.base import tr2rpy
np.set_printoptions(
    formatter={'float': lambda x: f"{0:8.4g}" if abs(x) < 1e-10 else f"{x:8.4g}"})

bot= rtb.DHRobot(
[
    rtb.RevoluteDH(d=0.445, a=0.150, alpha= np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
    rtb.RevoluteDH(d=0, a=0.900, alpha=0, offset= np.pi/2, qlim=[np.deg2rad(-95), np.deg2rad(155)]),
    rtb.RevoluteDH(d=0, a=0.115, alpha= np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(75)]),
    rtb.RevoluteDH(d=0.795, a=0, alpha=-np.pi/2, qlim=[np.deg2rad(-400), np.deg2rad(400)]),
    rtb.RevoluteDH(d=0, a=0, alpha= np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
    rtb.RevoluteDH(d=0.085, a=0, alpha=0, offset= np.pi, qlim=[np.deg2rad(-400), np.deg2rad(400)]),

], name="ABB IRB 2600-12/1.85", base=SE3([0,0,0]))

#Ponemos el TCP
bot.tool=SE3.OA([0, 1, 0], [0, 0, 1])
bot.qr = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0)]

# bot.plot(q=[0,0,0,0,0,0], limits=[-0.2,1.6,-0.8,0.8,-0.1,2], eeframe=True,backend='pyplot', shadow=True, jointaxes=True, block=True)
# bot.teach(q=[0,0,0,0,0,0])
T_init = bot.fkine(q=[0,0,0,0,0,0]) #Verificacion ok
print(f"La matriz en home: \n {T_init}")

T = np.array([
   [1.230, 0, 1.460, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)], #1XHome
   [1.230, 0, 1.460, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)], #1XHome
   [.600, .900, .294, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #1A
   [.600, .900, .164, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #1B
      [.900, -.600, .445, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #4H

   [-.150, -.550, .443, np.deg2rad(90), np.deg2rad(0), np.deg2rad(180)], #2.1C
   [-.150, -1.520, .433, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.1D
   [-.150, -1.520, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.1E
   [-.150, -1.790, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.1F
   [-.150, -1.790, .185, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.1G
   [-.150, -1.790, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.2F
   [-.150, -1.520, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.2E
   [-.150, -1.520, .433, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.2D
   [-.150, -.550, .443, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #2.2C
   [-.150, -1.520, .433, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.1D
   [-.150, -1.520, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.1E
   [-.150, -1.790, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.1F
   [-.150, -1.790, .185, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.1G
   [-.150, -1.790, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.2F
   [-.150, -1.520, .283, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.2E
   [-.150, -1.520, .433, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.2D
   [-.150, -.550, .443, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #3.2C
   [.900, -.600, .445, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #4H
   [1.280, .855, .446, np.deg2rad(-90), np.deg2rad(0), np.deg2rad(180)], #4I
   [1.230, 0, 1.460, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(0)], #4X (home)
])


via = np.empty((0, 6))
print(via)
for pose in T:
    via = np.vstack((via, pose)) #Append filas a puntos en via
print(f"Los puntos añadidos fueron {len(via)}")
xyz_traj = rtb.mstraj(via, qdmax=0.5, dt=0.2, tacc=0.15).q
print(f"Los puntos de trayectoria son {len(xyz_traj)}, \n siendo el primero {xyz_traj[0]}")

# # Grafica de trayectoria
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
plt.plot(xyz_traj[:,0], xyz_traj[:,1], xyz_traj[:,2])
ax.scatter(xyz_traj[0, 0], xyz_traj[0, 1], xyz_traj[0, 2], color='red', marker='*') #Inicio
ax.scatter(xyz_traj[-1, 0], xyz_traj[-1, 1], xyz_traj[-1, 2], color='blue', marker='o') #Fin
plt.show()

##Convertir los puntos, x,y,z, a_x, a_y, a_z a matriz 4x4
R=SE3().RPY(xyz_traj[:,3:], order="xyz").A #Matriz rotacion 3x3
# print(f"La lista es {R[0]}, {type(R)}")
R=np.array(R)
# print(f"La matriz es \n{R[0]}, {type(R)}")
R=np.round(R, decimals=2)
#Ahora meter R en la de transformacion
T_tool=np.zeros((len(xyz_traj),4,4))
T_tool[:,:,:] = R
T_tool[:, :3, 3]= xyz_traj[:, :3]
T_tool[:,3,3] = 1
print(f"La primera matriz es\n {T_tool[0]}, {type(T_tool)}\n que debe coincidir con la inicial fkine")

#Ya que lo tenemos la matriz de la forma deseada...
sol = bot.ikine_LM(T_tool,tol=0.0025) 
# sol= bot.ikine_LM(T_tool,tol=0.0025, joint_limits=False) 
print(sol.success)
bot.plot(q=sol.q, limits=[-1.5,1.5,-1.5,1.5,-0,1.5], eeframe=True, \
       backend='pyplot', shadow=True, jointaxes=False, block=True, movie='TransCubo.gif')
# # # #Guardar imagen
# # # # robot.plot(q=sol.q, limits=[-0.3,0.6,-0.6,0.6,-0.1,1], eeframe=True, \
# # # #     backend='pyplot', shadow=True, jointaxes=True, block=True, movie='TransCubo.gif')
