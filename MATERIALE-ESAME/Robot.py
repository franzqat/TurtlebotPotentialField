# -*- coding: utf-8 -*-

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

    def evaluate(self, _input, delta_t):
        self.p = self.p + delta_t * self.v
        self.v = (1 - self.b * delta_t/self.M) * self.v + delta_t / self.M * _input

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
        self.__speed_pi_left = PISat(30,110,100)
        self.__speed_pi_right = PISat(30,110,100) 
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
    
    def setPose(self, x, y, theta):
        self.x=x
        self.y=y
        self.theta=theta