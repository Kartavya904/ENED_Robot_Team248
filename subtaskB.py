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
degrees = 180
GSPK = 2.5
speed = 250

wait(100)
gyro_sensor.reset_angle(0)

for i in range(1, 1 + (2 * n)):
    gyro_sensor.reset_angle(0)

    robot.straight(y * 10)
    robot.turn(degrees)
    while gyro_sensor.angle() < degrees:
        robot.drive(0, 80)
        correction = (0 - gyro_sensor.angle()) * GSPK
        robot.drive(speed, correction)
    

robot.stop()
left_motor.brake()
right_motor.brake()