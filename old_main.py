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


forward(ev3, robot, gyro_sensor, 1, 50)
turn(ev3, robot, 180, gyro_sensor, 1, 10)

robot.stop()