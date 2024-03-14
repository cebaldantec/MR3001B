
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
#Para usar el DH
import roboticstoolbox as rtb

#Ahora usando el DH con Toolbox *************

bot= rtb.DHRobot(
[
    rtb.RevoluteDH(d=0.445, a=0.150, alpha= np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(180)]),
	rtb.RevoluteDH(d=0, a=0.900, alpha=0, offset= np.pi/2, qlim=[np.deg2rad(-95), np.deg2rad(155)]),
	rtb.RevoluteDH(d=0, a=0.115, alpha= np.pi/2, qlim=[np.deg2rad(-180), np.deg2rad(75)]),
	rtb.RevoluteDH(d=0.795, a=0, alpha=-np.pi/2, qlim=[np.deg2rad(-400), np.deg2rad(400)]),
    rtb.RevoluteDH(d=0, a=0, alpha= np.pi/2, qlim=[np.deg2rad(-120), np.deg2rad(120)]),
    rtb.RevoluteDH(d=0.085, a=0, alpha=0, qlim=[np.deg2rad(-400), np.deg2rad(400)]),

], name="ABB IRB 2600-12/1.85", base=SE3(0,0,0))
print(bot)

#Para modificar los angulos comodamente
joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0) 
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)

T06DH=bot.fkine([joint1, joint2, joint3, joint4, joint5, joint6]) 
print(T06DH)


q=np.array([[joint1, joint2, joint3, joint4, joint5, joint6],])

#Graficar con posiciones q, una cada 3 segundos
#bot.plot(q=q, backend='pyplot',dt=3, limits=[-0.8,0.8,-0.8,0.8,-0.4,1.5], shadow=True, jointaxes=True, block=True)

# Graficar con controlador
q1=np.array([[0, np.pi/2, 0, 0, 0, 0]])
bot.teach(q1)



