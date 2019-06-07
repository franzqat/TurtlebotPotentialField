#
# position_control_task.py
#
import sys
from turtle import *
from PotentialField_control import *

if __name__ == "__main__":

    t = Turtlebot()
    t.open()

    d = PotentialFieldControl(t,
                              1, #bot
                              0.22, ##base
                              5,  #altezza
                              2.84, #deltat
                              ,# kp_lin
                              , #sat_lineare
                              ,#kp ang
                              ,#sat angolare
                              
                              
                              soglia=0.26,#soglia
                              ostacoli=ostacoli, #ostacoli
                              k_att=1)#katt)
    
    d.start()

    while True:
        cmd_string = raw_input("Command> ")
        commands = cmd_string.split()
        if commands == []:
            continue
        
        if commands[0] == "help":
            print "Commands:"
            print "quit          exit the program"
            print "pose          return the pose of the robot"
            print "pset x y th   set the pose of the robot"
            print "speed l r     set the speed of left and right wheels"
            print "stop          stop the robot"
            
        elif commands[0] == "pose":
            print t.getPose()
            
        elif commands[0] == "quit":
            d.stop()
            sys.exit(0)
            
        elif commands[0] == "speed":
            print t.setSpeeds(int(commands[1]), int(commands[2]))
            
        elif commands[0] == "stop":
            print t.setSpeeds(0,0)
            
        elif commands[0] == "pset":
            print t.setPose(int(commands[1]), int(commands[2]), int(commands[3]))
            
        elif commands[0] == "clear":
            t.clearDistances()
            print "ok"
            
        elif commands[0] == "go":
            d.setTarget(int(commands[1]))
            
        else:
            print "Invalid command"



