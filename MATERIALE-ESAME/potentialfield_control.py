#
# potential_control
#

import threading
import time
import math


#Parametri robot
vel_max=0.22 #m/s
rot_max=2.84 #rad/s
wheelebase=0.160 #m
robot_radius=0.105 #m
#Parametri terreno
#base=1.5 #m
#altezza=3 #m
k_rep_bordi=0.5
k_repulsiva_ostacoli = 0.5


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
    
    #la distanza minima e' la distanza della circonferenza, cioe' distanza_dal_centro-raggio-raggio_robot
    d=math.hypot(d_x, d_y)-ostacolo.raggio - robot_radius

    if d<soglia and d!=0:
        f_rep=ostacolo.k_rep*((1/d**3)-(1/((d**2)*soglia)))
        angolo=math.atan2(d_y, d_x)

            
        f_rep_x=-f_rep*math.cos(angolo)
        f_rep_y=-f_rep*math.sin(angolo)
    else:
        f_rep_x=0
        f_rep_y=0
        
    return f_rep_x, f_rep_y


def forza_repulsiva_bordo_x(posizione, base, altezza, k_rep_b, soglia):
    f_rep_x=0
    f_rep_y=0
    k_rep=k_rep_b
    #distanza lato inferiore
    d0=posizione[1]
    if d0<soglia and d0!=0:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_y=+f_rep
    
    #distanza lato superiore
    d0=altezza-posizione[1]
    if d0<soglia and d0!=0:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_y=-f_rep
        
    #distanza lato sinistro
    d0=posizione[0]
    if d0<soglia and d0!=0:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_x=+f_rep
    
    #distanza lato destro
    d0=base-posizione[0]
    if d0<soglia and d0!=0:
        f_rep=k_rep*((1/d0**3)-(1/((d0**2)*soglia)))
        f_rep_x=-f_rep
        
    return f_rep_x, f_rep_y

class PotentialFieldController(threading.Thread):

    def __init__(self, turtle_if, base, altezza, delta_t, kp_lin, sat_lin, kp_ang, sat_ang, soglia, ostacoli, k_att):
        threading.Thread.__init__(self)
        self.turtle = turtle_if
        self.setDaemon(True)
        p = self.turtle.getPose()
        print("p ",p)
        self.target_pos = (float(p.x)/1000, float(p.y)/1000)
        print("target pos", self.target_pos)
        self.mutex = threading.Lock()
        self.delta_t = delta_t
        self.base=base
        self.altezza=altezza
        self.kp_lin = kp_lin
        self.sat_lin = sat_lin
        self.kp_ang = kp_ang
        self.sat_ang = sat_ang
        self.soglia=soglia
        self.ostacoli=ostacoli
        self.k_att=k_att
        self.cnt = 0
	
    def setTarget(self, tx, ty):
        self.mutex.acquire()
        self.target_pos = (tx, ty)
        self.turtle.clearDistances()
        self.mutex.release()

    def start(self):
        self.isRunning = True
        threading.Thread.start(self)

    def stop(self):
        self.isRunning = False
        
    def run(self):
        while self.isRunning:
            time.sleep(self.delta_t)

            self.mutex.acquire()
            p = self.turtle.getPose()

            f_tot_x=0
            f_tot_y=0
            
            for ostacolo in self.ostacoli:            
                f_rep_x, f_rep_y=forza_repulsiva_ostacolo(ostacolo=ostacolo, 
                                                         posizione=(float(p.x)/1000, float(p.y)/1000),
                                                         soglia=self.soglia)
                f_tot_x+=f_rep_x
                f_tot_y+=f_rep_y
            
            
            f_rep_x, f_rep_y = forza_repulsiva_bordo_x(posizione=(float(p.x)/1000, float(p.y)/1000),
                                                   base=self.base,
                                                   altezza=self.altezza,
                                                   k_rep_b=k_rep_bordi,
                                                   soglia=self.soglia)
            
            
            
            f_tot_x+=f_rep_x
            f_tot_y+=f_rep_y
            
            f_att_x, f_att_y=forza_attrattiva(k_att=self.k_att, 
                                              posizione=(float(p.x)/1000, float(p.y)/1000), 
                                              target=(self.target_pos[0], self.target_pos[1]))
            f_tot_x+=f_att_x
            f_tot_y+=f_att_y
            
            f_tot=math.hypot(f_tot_x, f_tot_y)
        
            ang=math.atan2(f_tot_y, f_tot_x)        
            heading_error = ang - p.theta
            
            
            #controllo PSat
            v = self.kp_lin * f_tot
            w = self.kp_ang * heading_error
    
            if v > self.sat_lin:
                v = self.sat_lin
            elif v < -self.sat_lin:
                v = - self.sat_lin
    
            if w > self.sat_ang:
                w = self.sat_ang
            elif w < -self.sat_ang:
                w = - self.sat_ang
    
            vl = v - (w * wheelebase / 2)
            vr = v + (w * wheelebase / 2)
            if vl > self.sat_lin:
                vl = self.sat_lin
            elif vl < -self.sat_lin:
                vl = - self.sat_lin
            if vr > self.sat_lin:
                vr = self.sat_lin
            elif vr < -self.sat_lin:
                vr = - self.sat_lin

            #quiuiif self.cnt %150 == 0:
                #print("f_tot",f_tot,"vl vr", (vl,vr), "v",v)
                #print(self.target_pos)
            
###TRESHOLD blocca le ruote quando arriva al target anche se ancora sotto influsso degli ostacoli
            if (abs(self.target_pos[0] - (float(p.x)/1000) < 0.05) and (abs(self.target_pos[1] - (float(p.y)/1000))) < 0.05):
                vl = 0
                vr = 0
                if self.cnt %150 == 0:
                    print('freno!',self.target_pos )

            self.cnt+=1
            self.mutex.release()
            self.turtle.setSpeeds(vl*1000, vr*1000)



