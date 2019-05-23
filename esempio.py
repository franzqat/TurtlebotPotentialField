# -*- coding: utf-8 -*-
"""
Created on Wed May 22 12:20:02 2019

@author: simon
"""
import pylab
import math
import numpy as np
import matplotlib.pyplot as plt

import distance_control
import turtle

class PISat:

    def __init__(self, kp, ki, sat):
        self.kp = kp
        self.ki = ki
        self.saturation = sat
        self.integral = 0
        self.saturation_flag = False

    def evaluate(self, target, current, delta_t):
        error = target - current
        if not(self.saturation_flag):
            self.integral = self.integral + error * delta_t
        output = self.kp * error + self.ki * self.integral
        if output > self.saturation:
            output = self.saturation
            self.saturation_flag = True
        elif output < -self.saturation:
            output = -self.saturation
            self.saturation_flag = True
        else:
            self.saturation_flag = False
        return output


class Massa:

    def __init__(self, _M, _b):
        self.M = _M
        self.b = _b
        # variabili di stato, p = posizione, v = velocita'
        self.p = 0
        self.v = 0

    def evaluate(self, _input, dt):
        self.p = self.p + delta_t * self.v
        self.v = (1 - self.b * dt/self.M) * self.v + dt / self.M * _input

    def get_position(self):
        return self.p

    def get_speed(self):
        return self.v


class Robot:

    def __init__(self, _M, _b, _wheelbase):
        self.__left_w = Massa(_M / 2.0, _b)
        self.__right_w = Massa(_M / 2.0, _b)
        self.wheelbase = _wheelbase
        self.x = 0
        self.y = 0
        self.theta = 0
        # posizioni precedenti (servono per calcolare l'odometria)
        self.__prev_pL = 0
        self.__prev_pR = 0
        # controllori velocita'
        self.__speed_pi_left = PISat(150,400,200)
        self.__speed_pi_right = PISat(150,400,200)
        # velocita' attuali
        self.current_vl = 0
        self.current_vr = 0

    def evaluate(self, vl, vr, delta_t):
        # aggiornamento odometria
        pL = self.__left_w.get_position()
        pR = self.__right_w.get_position()

        delta_L = pL - self.__prev_pL
        delta_R = pR - self.__prev_pR

        self.__prev_pL = pL
        self.__prev_pR = pR

        delta_theta = (delta_R - delta_L) / self.wheelbase
        delta_linear = (delta_L + delta_R) / 2.0

        self.x = self.x + delta_linear * math.cos(self.theta + delta_theta / 2.0)
        self.y = self.y + delta_linear * math.sin(self.theta + delta_theta / 2.0)
        self.theta = self.theta + delta_theta

        if self.theta > math.pi:
            self.theta = self.theta - 2*math.pi
        if self.theta < -math.pi:
            self.theta = 2*math.pi + self.theta

        # calcolo controllori di velocita' ruote
        self.current_vl = self.__left_w.get_speed()
        self.current_vr = self.__right_w.get_speed()
        output_L = self.__speed_pi_left.evaluate(vl, self.current_vl, delta_t)
        output_R = self.__speed_pi_right.evaluate(vr, self.current_vr , delta_t)

        # applicazione dell'output ai motori
        self.__left_w.evaluate(output_L, delta_t)
        self.__right_w.evaluate(output_R, delta_t)

    def getPose(self):
        return self.x, self.y, self.theta


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
            f_rep_x, f_rep_y=forza_repulsiva(ostacolo=ostacolo, 
                                             posizione=(self.robot.x, self.robot.y),
                                             soglia=self.soglia)           
            f_tot_x+=f_rep_x
            f_tot_y+=f_rep_y
            
        #forza attrattiva
        f_att_x, f_att_y=forza_attrattiva(k_att=self.k_att, 
                                          posizione=(self.robot.x, self.robot.y), 
                                          target=(target_x, target_y))
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

def forza_repulsiva(ostacolo, posizione, soglia):
    d_x = ostacolo.centro[0] - posizione[0]
    d_y = ostacolo.centro[1] - posizione[1]
    #la distanza minima è la distanza dlla circonferenza, cioè distanza_dal_centro-raggio
    d=math.hypot(d_x, d_y)-ostacolo.raggio
    
    if d<soglia:
        f_rep=ostacolo.k_rep*((1/d**3)-(1/(d**2)*soglia))
        angolo=math.atan2(d_y, d_x)
        f_rep_x=f_rep*math.cos(angolo)
        f_rep_y=f_rep*math.sin(angolo)
    else:
        f_rep_x=0
        f_rep_y=0
        
    return f_rep_x, f_rep_y



delta_t = 0.01

distanza_ruote=0.3
robot = Robot(6.0, 25.0, distanza_ruote)

t = 0
time_array = [ ]
pos_array = [ ]
theta = [ ]
vl_array = [ ]
vr_array = [ ]

ostacoli=[Ostacolo((10,10), 2, 10),
          Ostacolo((10,30), 2, 20),
          Ostacolo((15,20), 3, 20),
          Ostacolo((20,10), 1, 20),          
          Ostacolo((20,40), 1, 20),
          Ostacolo((25,25), 1, 20),
          Ostacolo((25,20), 2, 20),
          Ostacolo((30,10), 2, 20),
          Ostacolo((30,35), 3, 20),
          Ostacolo((35,20), 1, 20),
          Ostacolo((40,40), 2, 20),
          Ostacolo((45,25), 4, 20)]
          
p = PotentialFieldControl(robot,1, 1.5,5, 4, 10, ostacoli, k_att=1)



while t < 60:

    (vl, vr) = p.evaluate(50, 50)

    robot.evaluate(vl, vr, delta_t)

    time_array.append(t)
    pos_array.append((robot.x, robot.y))
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + delta_t


raggio_robot=distanza_ruote/2+0.1
fig, ax = pylab.subplots()

for ostacolo in ostacoli:
    cicle=plt.Circle(ostacolo.centro, ostacolo.raggio, color='b')
    plt.gcf().gca().add_artist(cicle)
plt.axis([0, 60, 0, 60])
for (x,y) in pos_array:
    cicle=plt.Circle((x,y), raggio_robot, color='r')
    plt.gcf().gca().add_artist(cicle) 
plt.show()



