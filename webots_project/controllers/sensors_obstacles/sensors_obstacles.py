"""sensors_obstacles controller."""

from controller import Robot
from controller import DistanceSensor
from controller import PositionSensor

# Create the Robot instance.
robot = Robot()

# Get the time step, max speed and sensor numbers.
TIME_STEP = 64 # ms 64
MAX_SPEED = 8 # 6.28
MAX_SENSOR_NUMBER = 16
MAX_SENSOR_POSITON = 2

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
    
# Initializa position sensor
ps_ = []
psname = ['left wheel sensor', 'right wheel sensor']

for i in range(MAX_SENSOR_POSITON):
    ps_.append(robot.getDevice(psname[i]))
    ps_[i].enable(TIME_STEP)

# GetDevice functions and velocity
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

print(leftMotor)

#RUN

while robot.step(TIME_STEP) != -1:

    # Get sensor values
    dsValues = []
    for i in range(MAX_SENSOR_NUMBER):
        dsValues.append(round(ds_[i].getValue(), 1))
    
    # Get position sensor values
    psValues = []
    for i in range(MAX_SENSOR_POSITON):
        psValues.append(ps_[i].getValue())
    
    # Switch case sonar laser regions
    def Sonar_distance(argument):
        switcher = {
            'front' : max(dsValues[3], dsValues[4]),
            'fleft' : max(dsValues[0], dsValues[1], dsValues[2]),
            'fright' : max(dsValues[5], dsValues[6], dsValues[7])
        }
        return switcher.get(argument, 'nothing')
    
    # Obstacles flag
    dist_param = 860 #860
    percentile_velocity = 0.9 #0.8

    front_obstacle = Sonar_distance('front') > dist_param or Sonar_distance('front') == 0
    fright_obstacle = Sonar_distance('fright') > dist_param
    fleft_obstacle = Sonar_distance('fleft') > dist_param

    front_no_obstacle = Sonar_distance('front') < dist_param
    fright_no_obstacle = Sonar_distance('fright') < dist_param
    fleft_no_obstacle = Sonar_distance('fleft') < dist_param
    
    print('Positon values', psValues)
    print('Distance sensor values')

    # Conditonal avoiding obstacles
    if front_no_obstacle and fleft_no_obstacle and fright_no_obstacle:
        print('Case 1 - Nothing')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED

    elif front_obstacle and fleft_no_obstacle and fright_no_obstacle:
        print('Case 2 - Front')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = -percentile_velocity * MAX_SPEED
    
    elif front_no_obstacle and fleft_no_obstacle and fright_obstacle:
        print('Case 3 - Fright')
        leftSpeed  = -percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED
    
    elif front_no_obstacle and fleft_obstacle and fright_no_obstacle:
        print('Case 4 - Fleft')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = -percentile_velocity * MAX_SPEED
    
    elif front_obstacle and fleft_no_obstacle and fright_obstacle:
        print('Case 5 - Front and Fright')
        leftSpeed  = -percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED
    
    elif front_obstacle and fleft_obstacle and fright_no_obstacle:
        print('Case 6 - Front and Fleft')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = -percentile_velocity * MAX_SPEED
    
    elif front_obstacle and fleft_obstacle and fright_obstacle:
        print('Case 7 - Front and Fleft and Fright')
        leftSpeed  = -0.8 * MAX_SPEED
        rightSpeed = -0.8 * MAX_SPEED
    
    elif front_no_obstacle and fleft_obstacle and fright_obstacle:
        print('Case 6 - Fleft and Fright')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED
    
    else:
        print('Unknown case')
        
    # Set velocity
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # Printing options
    c = 0
    for dist in dsValues:
        print(f's{c} {dist}')
        c += 1