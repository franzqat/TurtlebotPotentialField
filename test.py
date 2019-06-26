#
# position_control_task.py
#
import sys
from turtlebb import *
from potentialfield_control import *

#init ambiente statico
k_repulsiva_ostacoli = 0.5
ostacoli=[ Ostacolo((0.8,0.5), 0.1, k_repulsiva_ostacoli),
          Ostacolo((1.2,1), 0.1, k_repulsiva_ostacoli),
          Ostacolo((0.8,2), 0.1, k_repulsiva_ostacoli),
          Ostacolo((0.2,1.5), 0.1, k_repulsiva_ostacoli)]
base=1.5 #m
altezza=3 #m



if __name__ == "__main__":

    t = Turtlebot()
    t.open()
    t.setPose(105,105,0)
    d = PotentialFieldController(t, #bot
                              base, 
                              altezza,
                              0.01, #deltat
                              1,# kp_lin
                              0.22, #sat_lineare
                              5,#kp ang
                              2.84,#sat angolare                        
                              soglia=0.2,#soglia
                              ostacoli=ostacoli, #ostacoli
                              k_att=1)#katt
    
    
    d.start()

    while True:
        cmd_string = raw_input("Command> ")
        commands = cmd_string.split()
        if commands == []:
            continue
        
        if commands[0] == "help":
            print("Commands:")
            print("quit          	exit the program")
            print("pose          	return the pose of the robot")
            print("pset x y th   	set the pose of the robot")
            print("speed l r     	set the speed of left and right wheels")
            print("printo			print ostacoli")
            print("stop            stop the robot")

        elif commands[0] == "pose":
            print(t.getPose())

        elif commands[0] == "quit":
            d.stop()
            sys.exit(0)

        elif commands[0] == "speed":
            print(t.setSpeeds(int(commands[1]), int(commands[2])))

        elif commands[0] == "stop":
            print(t.setSpeeds(0,0))

        elif commands[0] == "pset":
            print(t.setPose(int(commands[1]), int(commands[2]), int(commands[3])))

        elif commands[0] == "clear":
            t.clearDistances()
            print("ok")

        elif commands[0] == "go":
            setx=(int(commands[1]))
            sety=(int(commands[2]))
            d.setTarget(float(setx)/1000, float(sety)/1000)		

        elif commands[0] == "printo":
            print(ostacoli)

        else:
            print("Invalid command")



