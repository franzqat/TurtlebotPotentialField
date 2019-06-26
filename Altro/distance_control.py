#
# distance_control.py
#

import threading
import time

class DistanceController(threading.Thread):

    def __init__(self, turtle_if, delta_t, kp, sat):
        threading.Thread.__init__(self)
        self.turtle = turtle_if
        self.setDaemon(True)
        p = self.turtle.getPose()
        self.target_pos = p.linear
        self.mutex = threading.Lock()
        self.delta_t = delta_t
        self.kp = kp
        self.sat = sat

    def setTarget(self, tp):
        self.mutex.acquire()
        self.target_pos = tp
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
            current_pos = p.linear
            
            error = self.target_pos - current_pos

            output = error * self.kp
            if (output < -self.sat):
                output = -self.sat
            elif (output > self.sat):
                output = self.sat

            self.mutex.release()
            self.turtle.setSpeeds(output, output)

