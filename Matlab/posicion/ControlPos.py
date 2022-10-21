import math
# import rospy as rp
import numpy as np
#import tictoc.py
from tictoc import *

# tiempo de simulacion
tf = 15 # tiempo final
ts = 0.01 # tiempo de muestreo
t = np.arange(0,tf,ts) # vector de tiempo
Q = len(t)

###############| parametros del sistema |################

hx = np.zeros((Q+1)) # posicion inicial en x [m]
hy = np.zeros((Q+1)) # posicion inicial en y [m]
phi = np.zeros((Q+1)) # orientacion inicial [rad]

###############| Posicion deseada |###############

hxd = 1.5 # posicion deseada en x [m]
hyd = 0.9 # posicion deseada en y [m]
phid = -np.pi/4 # orientacion deseada [rad]

uMeas = np.zeros((1,Q)) # velocidad lineal maxima [m/s]
wMeas = np.zeros((1,Q)) # velocidad angular maxima [rad/s]

uRef = np.zeros((1,Q)) # velocidad lineal maxima [m/s]
wRef = np.zeros((1,Q)) # velocidad angular maxima [rad/s]

l = np.zeros((1,Q))
rho = np.zeros((1,Q))
theta = np.zeros((1,Q))

tiempo = np.zeros((1,Q))

COM = 'COM3' # puerto serial

###############| Comunicacion ROS |################




###############| Controlador |################
for k in range(Q):
    tic()
    
    ###############| Control |###############
    #a) Calculo de errores
    l[k] = np.sqrt(((hxd-hx[k])**2)+((hyd-hy[k])**2))
    rho[k] = np.arctan2((hyd-hy[k]),(hxd-hx[k]))-phi[k]
    theta[k] = rho[k]+phi[k]-phid
    
    #b) Control de parametros
    k1 = 0.1 + (0.5*(k/Q))
    k2 = 0.1 + (0.8*(k/Q))
    
    #c) ley de control
    uRef[k] = k1*l[k]*np.cos(rho[k])
    wRef[k] = k2*rho[k]+(k1/rho[k])*np.cos(rho[k])*np.sin(rho[k])*(rho[k]+theta[k])
    
    ############### Variables ROS ################
    trama = [str(np.round(uRef[k],2)) + ',' + str(np.round(wRef[k],2))]
    
    #TODO update values of uMeas and wMeas
    
    #a) simulated Robot (kinematics model)
    phi[k+1] = phi[k] + wMeas[k]*ts # orientacion real [rad]
    hxp = uMeas[k]*np.cos(phi[k]) # velocidad real [m/s]
    hyp = uMeas[k]*np.sin(phi[k]) # velocidad real [m/s]
    
    #b) numerical integral (Euler method)
    hx[k+1] = hx[k] + hxp*ts # posicion central real en x [m]
    hy[k+1] = hy[k] + hyp*ts # posicion central real en y [m]
    
    tiempo[k] = toc()
    compa = toc()
    
    while(compa < ts):
        pass
    
###############| Close Ros Connection |###############
#TODO close ros connection