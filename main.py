#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Tank Bot Program
----------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#building-expansion
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Configure 2 motors on Ports B and C.  Set the motor directions to
# counterclockwise, so that positive speed values make the robot move
# forward.  These will be the left and right motors of the Tank Bot.
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

# The wheel diameter of the Tank Bot is about 54 mm.
WHEEL_DIAMETER = 43.2

# The axle track is the distance between the centers of each of the
# wheels.  This is about 200 mm for the Tank Bot.
AXLE_TRACK = 200

# The Driving Base is comprised of 2 motors.  There is a wheel on each
# motor.  The wheel diameter and axle track values are used to make the
# motors move at the correct speed when you give a drive command.
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# Set up the Gyro Sensor. 
gyro_sensor = GyroSensor(Port.S4)


# Initialize the steering and overshoot variables.
steering = 60
overshoot = 5

# turns no. of degrees towards the right direction 
def turn_right_angle(degrees):
    # Reset the Gyro Sensor angle.
    gyro_sensor.reset_angle(0)

    # Turn clockwise until the angle is 90 degrees.
    robot.drive(0, steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() < degrees - overshoot:
        wait(1)
    robot.drive(0, 0)
    wait(100)

# turns no. of degrees towards the left direction 
def turn_left_angle(degrees):
    # Reset the Gyro Sensor angle.
    gyro_sensor.reset_angle(0)

    # Turn counter-clockwise until the angle is 90 degrees.
    robot.drive(0, -steering)

    ev3.speaker.beep()

    while gyro_sensor.angle() > overshoot - degrees:
        wait(1)
    robot.drive(0, 0)
    wait(100)

# def barcode():
    
#     total = 0
#     item_num = 0

#     for x in range (0:3)

#         while True
#             color = color_sensor.color()

#             if color in POSSIBLE_COLORS:
#                 break
        
#         if color == Color.BLACK or color == Color.None :
#             if color == Color.BLACK:
#                 i = 1
#             else if color == Color.None:
#                 i = 0
        
#             total = (i * (2^(i))) + total
#             wait(100)
            
#         # medium motor moves up

#     if total == 8:
#         item_num = 1
#     else if total == 10:
#         item_num = 2
#     else if total == 12 :
#         item_num = 3
#     else if total == 9 :
#         item_num = 4

#     return item_num

# To convert 'in' to 'mm' multiply 33
# 'n' is in 'inches'

def forward(n):
    distance = 33*n
    target = gyro_sensor.angle()
    gain = 2
    robot.reset()
    while robot.distance() <= distance:
        correction = (target - gyro_sensor.angle()) * gain
        robot.drive(200, correction)
        wait(10)
    robot.stop()

forward(12)

turn_left_angle(90) 

forward(96)



