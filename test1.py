# -*- coding: utf-8 -*-
"""
Created on Thu Jun  6 17:52:12 2019

@author: simon
"""

import pylab
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import Robot as rb

#Parametri robot
vel_max=0.22 #m/s
rot_max=2.84 #rad/s
whelebase=0.160 #m
robot_radius=0.105 #m


class Ostacolo:
    def __init__(self, centro, raggio, k_rep):
        self.centro=centro
        self.raggio=raggio
        self.k_rep=k_rep

def forza_attrattiva(k_att, posizione, target):
    dx = target[0] - posizione[0]
    dy = target[1] - posizione[1]
    
    #angolo verso l'obiettivo
    target_heading = math.atan2(dy, dx)
    
    #distanza dall'obiettivo
    distance = math.hypot(dx, dy)
    
    #forza attrattiva e componenti
    f_att=k_att*distance
    f_att_x=f_att*math.cos(target_heading)
    f_att_y=f_att*math.sin(target_heading)
    
    return f_att_x, f_att_y

def forza_repulsiva_ostacolo(ostacolo, posizione, soglia):
    d_x = ostacolo.centro[0] - posizione[0] 
    d_y = ostacolo.centro[1] - posizione[1] 
    
    #la distanza minima è la distanza della circonferenza, cioè distanza_dal_centro-raggio-raggio_robot
    d=math.hypot(d_x, d_y)-ostacolo.raggio - robot_radius

    if d<soglia:      
        f_rep=ostacolo.k_rep*((1/d**3)-(1/((d**2)*soglia)))
        #k_rep * ((1/d)-(1/soglia))*1/d**2
        #print(f"distanza {d}, frep {f_rep}  ")
        angolo=math.atan2(d_y, d_x)
    
        if angolo > abs(math.pi/2):
            f_rep_x=-f_rep*math.cos(angolo)
        else:
            f_rep_x=+f_rep*math.cos(angolo)
            
        if angolo > 0:
            f_rep_y=-f_rep*math.sin(angolo)
        else:
            f_rep_y=f_rep*math.sin(angolo)
            
#        f_rep_x=-f_rep*math.cos(angolo)
#        f_rep_y=+f_rep*math.sin(angolo)
        #print(f"angolo {angolo} repx {f_rep_x}, repy {f_rep_y}, t {t} ,d {d}")
    else:
        f_rep_x=0
        f_rep_y=0
        
    return f_rep_x, f_rep_y


def forza_repulsiva_bordo_x(posizione, soglia):
    f_rep_x=0
    f_rep_y=0
    k_rep=0.5
    #distanza lato inferiore
    d0=posizione[1]
    if d0<soglia:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_y=+f_rep
    
    #distanza lato superiore
    d0=2-posizione[1]
    if d0<soglia:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_y=-f_rep
        
    #distanza lato sinistro
    d0=posizione[0]
    if d0<soglia:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_x=+f_rep
    
    #distanza lato destro
    d0=3-posizione[0]
    if d0<soglia:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_x=-f_rep
        
    return f_rep_x, f_rep_y

class PotentialFieldControl:

    def __init__(self, robot, kp_lin, sat_lin, kp_angular, sat_angular, soglia, ostacoli, k_att):
        self.kp_lin = kp_lin
        self.sat_lin = sat_lin
        self.kp_angular = kp_angular
        self.sat_angular = sat_angular
        self.robot = robot
        self.soglia=soglia
        self.ostacoli=ostacoli
        self.k_att=k_att       

    def evaluate(self, target_x, target_y):

        f_tot_x=0
        f_tot_y=0
        
        #forza repulsiva
        for ostacolo in self.ostacoli:            
            f_rep_x, f_rep_y=forza_repulsiva_ostacolo(ostacolo=ostacolo, 
                                             posizione=(self.robot.x, self.robot.y),
                                             soglia=self.soglia)           
            f_tot_x+=f_rep_x
            f_tot_y+=f_rep_y
            
        f_rep_x, f_rep_y = forza_repulsiva_bordo_x(posizione=(self.robot.x, self.robot.y),
                                             soglia=self.soglia)
        f_tot_x+=f_rep_x
        f_tot_y+=f_rep_y
        #forza attrattiva
        f_att_x, f_att_y=forza_attrattiva(k_att=self.k_att, 
                                          posizione=(self.robot.x, self.robot.y), 
                                          target=(target_x, target_y))
        
        #print(f_att_x,f_att_y,)
        f_tot_x+=f_att_x
        f_tot_y+=f_att_y
        
        
        f_tot=math.hypot(f_tot_x, f_tot_y)
        
        ang=math.atan2(f_tot_y, f_tot_x)        
        heading_error = ang - robot.theta
        
        
        #controllo PSat
        v = self.kp_lin * f_tot
        w = self.kp_angular * heading_error

        if v > self.sat_lin:
            v = self.sat_lin
        elif v < -self.sat_lin:
            v = - self.sat_lin

        if w > self.sat_angular:
            w = self.sat_angular
        elif w < -self.sat_angular:
            w = - self.sat_angular

        vl = v - (w * self.robot.wheelbase / 2)
        vr = v + (w * self.robot.wheelbase / 2)
        return (vl, vr)


k_repulsiva = 0.5
ostacoli=[Ostacolo((1.5,1.0), 0.2, k_repulsiva),
          Ostacolo((2.8,1.0), 0.2, k_repulsiva),
          Ostacolo((2.0,0.8), 0.2, k_repulsiva)]
START=(0.105,0.105)
TARGET=(2.8, 1.8)
robot = rb.Robot(1.0, 5.0, whelebase)
robot.setPose(START[0], START[1], 0)

p = PotentialFieldControl(robot,1, 0.22,5, 2.84, soglia=0.1, ostacoli=ostacoli, k_att=1)

t = 0
time_array = [ ]
pos_array = [ ]
theta = [ ]
vl_array = [ ]
vr_array = [ ]
while t < 20:
    (vl, vr) = p.evaluate(TARGET[0],TARGET[1])

    robot.evaluate(vl, vr, rb.delta_t)

    time_array.append(t)
    pos_array.append((robot.x, robot.y))
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + rb.delta_t

#plotting
ax=plt.gcf().gca()
plt.axis([-0.2, 3.2, -0.2, 2.2])
for (x,y) in pos_array:
    cicle=plt.Circle((x,y), robot_radius, color='r')
    plt.gcf().gca().add_artist(cicle) 
for ostacolo in ostacoli:
    cicle=plt.Circle(ostacolo.centro, ostacolo.raggio, color='b')
    ax.add_artist(cicle)

ax.add_artist(plt.Circle(START, robot_radius,color='y'))
ax.add_artist(plt.Circle(TARGET,0.1,color='g'))

rect1 = matplotlib.patches.Rectangle((0,0), 3, 2, color="c") 
ax.add_patch(rect1)

plt.show()