# -*- coding: utf-8 -*-
"""
Created on Wed May 22 12:20:02 2019

@author: simon
"""
import pylab
import math

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

    def __init__(self, robot, kp_lin, sat_lin, kp_angular, sat_angular, soglia, ostacoli, k_att, k_rep):
        self.kp_lin = kp_lin
        self.sat_lin = sat_lin
        self.kp_angular = kp_angular
        self.sat_angular = sat_angular
        self.robot = robot
        self.soglia=soglia
        self.ostacoli=ostacoli
        self.k_att=k_att
        self.k_rep=k_rep        

    def evaluate(self, target_x, target_y):

        f_tot_x=0
        f_tot_y=0
        
        for (x,y) in self.ostacoli:
            d_x = x - self.robot.x
            d_y = y - self.robot.y
            d=math.hypot(d_x, d_y)
            if(d<self.soglia):
                f_rep=self.k_rep*((1/d**3)-(1/(d**2)*self.soglia))
                angolo=math.atan2(d_y, d_x)
                f_rep_x=f_rep*math.cos(angolo)
                f_rep_y=f_rep*math.sin(angolo)
                f_tot_x+=f_rep_x
                f_tot_y+=f_rep_y
            
            
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y

        target_heading = math.atan2(dy, dx)
        distance = math.hypot(dx, dy)

        

        f_att=self.k_att*distance
        f_att_x=f_att*math.cos(target_heading)
        f_att_y=f_att*math.sin(target_heading)
        f_tot_x+=f_att_x
        f_tot_y+=f_att_y
        
        f_tot=math.hypot(f_tot_x, f_tot_y)
        ang=math.atan2(f_tot_y, f_tot_x)
        
        heading_error = ang - robot.theta
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


delta_t = 0.001
robot = Robot(6.0, 25.0, 0.3)

t = 0
time_array = [ ]
x_array = [ ]
y_array = [ ]
theta = [ ]
vl_array = [ ]
vr_array = [ ]

ostacoli=[(10,10),(25,20), (30,30),(40,40)]
p = PotentialFieldControl(robot,1, 1.5,5, 4, 10, ostacoli, k_att=1, k_rep=10)



while t < 100:

    (vl, vr) = p.evaluate(50, 50)

    robot.evaluate(vl, vr, delta_t)

    time_array.append(t)
    x_array.append(robot.x)
    y_array.append(robot.y)
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + delta_t

print(robot.getPose())

"""   
while t < 30:

    (vl, vr) = p.evaluate(0, 5)

    robot.evaluate(vl, vr, delta_t)

    time_array.append(t)
    x_array.append(robot.x)
    y_array.append(robot.y)
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + delta_t

print(robot.getPose())    
while t < 45:

    (vl, vr) = p.evaluate(6, 6)

    robot.evaluate(vl, vr, delta_t)

    time_array.append(t)
    x_array.append(robot.x)
    y_array.append(robot.y)
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + delta_t

print(robot.getPose()) 
  
while t < 60:

    (vl, vr) = p.evaluate(0, 2)

    robot.evaluate(vl, vr, delta_t)

    time_array.append(t)
    x_array.append(robot.x)
    y_array.append(robot.y)
    vl_array.append(robot.current_vl)
    vr_array.append(robot.current_vr)

    t = t + delta_t
"""

pylab.figure(1)
pylab.plot(x_array, y_array, 'r-+', label='posizione robot')
for (x,y) in ostacoli:
    pylab.plot(x, y, 'b-+')
pylab.legend()

#pylab.figure(2)
#pylab.plot(time_array, vl_array, 'r-+', label='vl')
#pylab.plot(time_array, vr_array, 'b-+', label='vr')
#pylab.legend()

pylab.show()