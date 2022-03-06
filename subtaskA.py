from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor,InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from time import sleep

# Initialize
right_motor = Motor(Port.A)
left_motor = Motor(Port.D)
medium_motor = Motor(Port.C)
gyro_sensor = GyroSensor(Port.S4)
ev3 = EV3Brick()


robot = DriveBase(left_motor, right_motor, wheel_diameter=70, axle_track=95)
n = 1
y = 60
GSPK = 2.5
speed = 250

for i in range(1, 1+n):
    gyro_sensor.reset_angle(0)

    #forward
    while robot.distance() <= y:
        correction = (0 - gyro_sensor.angle()) * GSPK
        robot.drive(speed, correction)

    #backward
    while robot.distance() <= -y:
        correction = (0 - gyro_sensor.angle()) * GSPK
        robot.drive(-speed, correction)

robot.stop()
left_motor.brake()
right_motor.brake()