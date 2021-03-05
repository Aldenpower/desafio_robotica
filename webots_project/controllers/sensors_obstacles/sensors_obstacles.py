"""sensors_obstacles controller."""

from controller import Robot
from controller import DistanceSensor

# Create the Robot instance.
robot = Robot()

# Get the time step, max speed and sensor numbers.
TIME_STEP = 64 # ms 64
MAX_SPEED = 6.28 # 6.28
MAX_SENSOR_NUMBER = 16

# Initialize sonar distance sensors
ds_ = []
dsname = ['so0', 'so1', 'so2', 'so3', 'so4',
          'so5', 'so6', 'so7', 'so8', 'so9',
          'so10', 'so11', 'so12', 'so13',
          'so14', 'so15'
         ]
         
for i in range(MAX_SENSOR_NUMBER):
    ds_.append(robot.getDevice(dsname[i]))
    ds_[i].enable(TIME_STEP)
    
print(ds_)

# GetDevice functions and velocity
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Laser callback
def laser_callback(laser_list):
    pass

#RUN

while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    dsValues = []
    for i in range(MAX_SENSOR_NUMBER):
        dsValues.append(ds_[i].getValue())
    
    print(dsValues)
    
    laser_callback(dsValues)
     
    # detect obstacles
    #right_obstacle = dsValues[15] < 900 or dsValues[0] < 900
    #left_obstacle = dsValues[8] < 900 or dsValues[7] < 900

    '''
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        print('Left obstacle')
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        print('Right obstacle')
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    

    print(obstacles['Front'], obstacles['Fleft'])
    '''
    pass