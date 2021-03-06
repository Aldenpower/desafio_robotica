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

#RUN

while robot.step(TIME_STEP) != -1:

    # Read the sensors:
    dsValues = []
    # Get sensor values
    for i in range(MAX_SENSOR_NUMBER):
        dsValues.append(round(ds_[i].getValue(), 1))
    
    # Switch case sonar laser regions
    def Sonar_distance(argument):
        switcher = {
            'front' : max(dsValues[3], dsValues[4]),
            'fleft' : max(dsValues[0], dsValues[1], dsValues[2]),
            'fright' : max(dsValues[5], dsValues[6], dsValues[7])
        }
        return switcher.get(argument, 'nothing')
    
    if Sonar_distance('front') < 860 and Sonar_distance('fleft') < 860 and Sonar_distance('fright') < 860:
        # Move straight
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED

    elif Sonar_distance('front') > 860 and Sonar_distance('fleft') < 860 and Sonar_distance('fright') < 860:
        # Front obstacle YES Left NO Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif Sonar_distance('front') < 860 and Sonar_distance('fleft') < 860 and Sonar_distance('fright') > 860:
        # Front obstacle NO left NO Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    elif Sonar_distance('front') < 860 and Sonar_distance('fleft') > 860 and Sonar_distance('fright') < 860:
        # Front obstacle NO left YES Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif Sonar_distance('front') > 860 and Sonar_distance('fleft') < 860 and Sonar_distance('fright') > 860:
        # Front obstacle YES left NO Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    elif Sonar_distance('front') > 860 and Sonar_distance('fleft') > 860 and Sonar_distance('fright') < 860:
        # Front obstacle YES left YES Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED

    elif Sonar_distance('front') > 860 and Sonar_distance('fleft') > 860 and Sonar_distance('fright') > 860:
        # Front obstacle YES left YES Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif Sonar_distance('front') < 860 and Sonar_distance('fleft') > 860 and Sonar_distance('fright') > 860:
        # Front obstacle NO left YES Right YES
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    else:
        print('Unknown case')
        
    # Set velocity
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    print('Distance sensor values ', dsValues)

    pass