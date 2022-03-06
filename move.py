from pybricks.tools import wait

def forward(ev3, robot, gyro_sensor, n, y, GSPK, speed):
    GSPK = 2.5
    speed = 250

    for i in range(1, 1+n):

        gyro_sensor.reset_angle(0)
    
        while robot.distance() <= y:
            correction = (0 - gyro_sensor.angle()) * GSPK
            robot.drive(speed, correction)

    robot.stop()
    left_motor.brake()
    right_motor.brake()
        
def backward(ev3, robot, gyro_sensor, n, y, GSPK, speed):


    for i in range(1, 1+n):
        
        gyro_sensor.reset_angle(0)
    
        while robot.distance() <= -y:
            correction = (0 - gyro_sensor.angle()) * GSPK
            robot.drive(-speed, correction)
    
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    


def turn(ev3, robot, degrees, gyro_sensor, n, y):
    # while gyro_sensor.angle() != 0:
    #     gyro_sensor.reset_angle(0)
    
    # for i in range(1, 1+n):
    #     while gyro_sensor.angle() != 0:
    #         robot.turn(gyro_sensor.angle() * -1)
            
            
    #     robot.straight(y*10)

    #     while gyro_sensor.angle() != degrees:
    #         robot.turn(gyro_sensor.angle() * -1)
    
    wait(100)
    gyro_sensor.reset_angle(0)

    for i in range(1, 1 + (2 * n)):
        gyro_sensor.reset_angle(0)

        robot.straight(y * 10)
        robot.turn(degrees)
        while gyro_sensor.angle() < degrees:
            robot.drive(0, 80)
        


