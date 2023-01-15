#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math
import sys, select, termios, tty
import numpy as np
import time

joints = ['coxa','femur','tibia','pitch','roll','wheel']
sides = ['l', 'r']
legs = []
leg = []
for i in range(6):
    for j in joints:
        z = '/athlete_rover/' + j + '_' + str(i+1) + '_controller/command'
        leg.append(rospy.Publisher(z,Float64,queue_size=10))
    legs.append(leg)
    leg = []


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def home():
    pos = math.radians(0)
    if rospy.is_shutdown() == False:
        rospy.loginfo("Homing joints")
        for i in range(len(legs)):
            for j in range(len(joints)):
                legs[i][j].publish(pos)
    return

def store():
    pos = math.radians(-90)
    if rospy.is_shutdown() == False:
        rospy.loginfo("Robot storing")
        for i in range(len(legs)):
            for j in range(5):
                legs[i][j].publish(pos)
    return

def coxa(angle):
    if rospy.is_shutdown() == False:
        #rospy.loginfo("coxa joints")
        for i in range(6):
            legs[i][0].publish(angle)
    return

def femur(angle):
    if rospy.is_shutdown() == False:
        #rospy.loginfo("femur joints")
        for i in range(6):
            legs[i][1].publish(angle)
    return

def tibia(angle):
    if rospy.is_shutdown() == False:
        #rospy.loginfo("tibia joints")
        for i in range(6):
            legs[i][2].publish(angle)
    return

def pitch(angle):
    if rospy.is_shutdown() == False:
        #rospy.loginfo("pitch joints")
        for i in range(6):
            legs[i][3].publish(angle)
    return

def roll(angle):
    if rospy.is_shutdown() == False:
        #rospy.loginfo("roll joints")
        for i in range(6):
            legs[i][4].publish(angle)
    return

def wheel(velocity):
    if rospy.is_shutdown() == False:
        rospy.loginfo("wheel initiated")
        for i in range(6):
            legs[i][5].publish(velocity)
    return

def forward(velocity):
    if rospy.is_shutdown() == False:
        rospy.loginfo("wheel joints")
        legs[0][4].publish(math.radians(-60))
        legs[2][4].publish(math.radians(60))
        legs[3][4].publish(math.radians(-60))
        legs[5][4].publish(math.radians(60))
        legs[0][5].publish(-velocity)
        legs[1][5].publish(-velocity)
        legs[2][5].publish(-velocity)
        legs[3][5].publish(velocity)
        legs[4][5].publish(velocity)
        legs[5][5].publish(velocity)

def reverse(velocity):
    if rospy.is_shutdown() == False:
        rospy.loginfo("wheel joints")
        legs[0][4].publish(math.radians(-60))
        legs[2][4].publish(math.radians(60))
        legs[3][4].publish(math.radians(-60))
        legs[5][4].publish(math.radians(60))
        legs[0][5].publish(velocity)
        legs[1][5].publish(velocity)
        legs[2][5].publish(velocity)
        legs[3][5].publish(-velocity)
        legs[4][5].publish(-velocity)
        legs[5][5].publish(-velocity)

def athlete_stand():
    femur_angle = math.radians(30)
    tibia_angle = math.radians(30)
    pitch_angle = math.radians(35)

    if rospy.is_shutdown() == False:
        rospy.loginfo("Athlete standing")
        pitch(pitch_angle)
        tibia(tibia_angle)
        femur(femur_angle)
    return

def walk_stance():
    if rospy.is_shutdown() == False:
        rospy.loginfo("walking stance initiated")
        legs[0][1].publish(math.radians(-60))
        legs[0][2].publish(math.radians(90))
        time.sleep(3)
        legs[0][0].publish(math.radians(-40))
        time.sleep(3)
        legs[0][1].publish(math.radians(30))
        legs[0][2].publish(math.radians(30))
        time.sleep(3)
        legs[2][1].publish(math.radians(-60))
        legs[2][2].publish(math.radians(90))
        time.sleep(3)
        legs[2][0].publish(math.radians(40))
        time.sleep(3)
        legs[2][1].publish(math.radians(30))
        legs[2][2].publish(math.radians(30))
        time.sleep(3)
        legs[3][1].publish(math.radians(-60))
        legs[3][2].publish(math.radians(90))
        time.sleep(3)
        legs[3][0].publish(math.radians(-40))
        time.sleep(3)
        legs[3][1].publish(math.radians(30))
        legs[3][2].publish(math.radians(30))
        time.sleep(3)
        legs[5][1].publish(math.radians(-60))
        legs[5][2].publish(math.radians(90))
        time.sleep(3)
        legs[5][0].publish(math.radians(40))
        time.sleep(3)
        legs[5][1].publish(math.radians(30))
        legs[5][2].publish(math.radians(30))
    return

def walk():
    for i in range(6):
        if rospy.is_shutdown() == False:
            walk_stance()
            rospy.loginfo("Rover walking initiated")
            legs[5][1].publish(math.radians(-60))
            legs[5][2].publish(math.radians(90))
            time.sleep(2)
            legs[5][0].publish(math.radians(0))
            time.sleep(2)
            legs[5][1].publish(math.radians(30))
            legs[5][2].publish(math.radians(30))
            time.sleep(2)
            legs[1][1].publish(math.radians(-60))
            legs[1][2].publish(math.radians(90))
            time.sleep(2)
            legs[1][0].publish(math.radians(40))
            time.sleep(2)
            legs[1][1].publish(math.radians(30))
            legs[1][2].publish(math.radians(30))
            time.sleep(2)
            legs[3][1].publish(math.radians(-60))
            legs[3][2].publish(math.radians(90))
            time.sleep(2)
            legs[3][0].publish(math.radians(-80))
            time.sleep(2)
            legs[3][1].publish(math.radians(30))
            legs[3][2].publish(math.radians(30))
            time.sleep(2)
            legs[0][1].publish(math.radians(-60))
            legs[0][2].publish(math.radians(90))
            time.sleep(2)
            legs[0][0].publish(math.radians(0))
            time.sleep(2)
            legs[0][1].publish(math.radians(30))
            legs[0][2].publish(math.radians(30))
            time.sleep(2)
            legs[2][1].publish(math.radians(-60))
            legs[2][2].publish(math.radians(90))
            time.sleep(2)
            legs[2][0].publish(math.radians(80))
            time.sleep(2)
            legs[2][1].publish(math.radians(30))
            legs[2][2].publish(math.radians(30))
            time.sleep(2)
            legs[4][1].publish(math.radians(-60))
            legs[4][2].publish(math.radians(90))
            time.sleep(2)
            legs[4][0].publish(math.radians(-40))
            time.sleep(2)
            legs[4][1].publish(math.radians(30))
            legs[4][2].publish(math.radians(30))
            time.sleep(2)
            legs[0][0].publish(math.radians(-40))
            legs[1][0].publish(math.radians(0))
            legs[2][0].publish(math.radians(40))
            legs[3][0].publish(math.radians(-40))
            legs[4][0].publish(math.radians(0))
            legs[5][0].publish(math.radians(40))
    return

        
def commander():
    while not rospy.is_shutdown():
        key_press = getKey()
        if key_press == 'u':
            athlete_stand()
        elif key_press == 'h':
            home()
        elif key_press == 'p':
            store()
        elif key_press == 'a':
            roll(0)
            wheel(1.0)
        elif key_press == 'd':
            roll(0)
            wheel(-1.0)
        elif key_press == 's':
            wheel(0)
        elif key_press == 'g':
            walk_stance()
        elif key_press == 'v':
            walk()
        elif key_press == 'w':
            forward(1.5)
        elif key_press == 'x':
            reverse(1.5)
        elif key_press == 'c':
            break

    return
            

if __name__ == '__main__':
    # it is good practice to maintain
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('athlete_commander')
    rospy.loginfo("athlete commander node initiated")
    print("press following keys to command the rover:")
    print("h -> homing the joints")
    print("p -> storing all the legs")
    print("u -> to stand")
    print("a -> to turn right")
    print("d -> to turn left")
    print("w -> to move forward")
    print("x -> to move reverse")
    print("s -> stop wheels")
    print("w -> use w for walking stance before v")
    print("v -> Start walking")
    # a 'try'-'except' clause
    try:
        commander()
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
