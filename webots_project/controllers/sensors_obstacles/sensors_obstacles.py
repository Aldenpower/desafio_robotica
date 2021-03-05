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

# Sonar laser regions callback
def s_c(laser_list, tag = '', pr = False):
    lasers_range = {
    'right': max(laser_list[7], laser_list[8]),
    'left': max(laser_list[0], laser_list[15]),
    'fright': max(laser_list[5], laser_list[6]),
    'front': max(laser_list[3], laser_list[4]),
    'fleft': max(laser_list[1], laser_list[2]),
    }

    if tag == 'right':
        return lasers_range['right']
    if tag == 'left':
        return lasers_range['left']
    if tag == 'fright':
        return lasers_range['fright']
    if tag == 'front':
        return lasers_range['front']
    if tag == 'fleft':
        return lasers_range['fleft']
    
    if pr:
        print(lasers_range)
        return lasers_range

#RUN

while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    dsValues = []
    for i in range(MAX_SENSOR_NUMBER):
        dsValues.append(round(ds_[i].getValue(), 1))
    
    if s_c(dsValues, 'front') < 860 and s_c(dsValues, 'fleft') < 860 and s_c(dsValues, 'fright') < 860:
        # Move straight
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED

    elif s_c(dsValues, 'front') > 860 and s_c(dsValues, 'fleft') < 860 and s_c(dsValues, 'fright') < 860:
        # Front obstacle YES Left NO Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif s_c(dsValues, 'front') < 860 and s_c(dsValues, 'fleft') < 860 and s_c(dsValues, 'fright') > 860:
        # Front obstacle NO left NO Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    elif s_c(dsValues, 'front') < 860 and s_c(dsValues, 'fleft') > 860 and s_c(dsValues, 'fright') < 860:
        # Front obstacle NO left YES Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif s_c(dsValues, 'front') > 860 and s_c(dsValues, 'fleft') < 860 and s_c(dsValues, 'fright') > 860:
        # Front obstacle YES left NO Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    elif s_c(dsValues, 'front') > 860 and s_c(dsValues, 'fleft') > 860 and s_c(dsValues, 'fright') < 860:
        # Front obstacle YES left YES Right NO
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED

    elif s_c(dsValues, 'front') > 860 and s_c(dsValues, 'fleft') > 860 and s_c(dsValues, 'fright') > 860:
        # Front obstacle YES left YES Right YES
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
    
    elif s_c(dsValues, 'front') < 860 and s_c(dsValues, 'fleft') > 860 and s_c(dsValues, 'fright') > 860:
        # Front obstacle NO left YES Right YES
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
    
    else:
        print('Unknown case')
    

    


    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    print(dsValues)
    s_c(dsValues, pr = True)


    # detect obstacles
    #right_obstacle = dsValues[15] < 860 or dsValues[0] < 860
    #left_obstacle = dsValues[8] < 860 or dsValues[7] < 860

    '''
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.6 * MAX_SPEED
    rightSpeed = 0.6 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.6 * MAX_SPEED
        rightSpeed = -0.6 * MAX_SPEED
        print('Left obstacle')
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.6 * MAX_SPEED
        rightSpeed = 0.6 * MAX_SPEED
        print('Right obstacle')
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    

    print(obstacles['Front'], obstacles['Fleft'])
    '''
    pass