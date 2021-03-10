"""sensors_obstacles controller."""

from controller import Robot
from controller import DistanceSensor
from controller import PositionSensor
from controller import LightSensor

# Creating the Robot instance.
robot = Robot()

# Getting the time step, max speed, and sensors quantity.
TIME_STEP = 64 # ms 64
MAX_SPEED = 8 # 6.28
MAX_SONAR_SENSOR_NUMBER = 16
MAX_POSITION_SENSOR_NUMBER = 2

# Initializing sonar distance sensors
ds_ = []
dsname = ['so0', 'so1', 'so2', 'so3', 'so4',
          'so5', 'so6', 'so7', 'so8', 'so9',
          'so10', 'so11', 'so12', 'so13',
          'so14', 'so15'
         ]
         
for i in range(MAX_SONAR_SENSOR_NUMBER):
    ds_.append(robot.getDevice(dsname[i]))
    ds_[i].enable(TIME_STEP)

# Initiazling light sensor
ls = robot.getDevice('light sensor')
ls.enable(TIME_STEP)

# Initializing position sensors
ps_ = []
psname = ['left wheel sensor', 'right wheel sensor']

for i in range(MAX_POSITION_SENSOR_NUMBER):
    ps_.append(robot.getDevice(psname[i]))
    ps_[i].enable(TIME_STEP)

# Getting wheel devices
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')

# Setting the motors position
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Setting the motors velocity
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#RUNNING SIMULATION

average_sonar_sensor_acum = [1000, 0, 1000, 0, 1000, 1000, 0, 1000, 0, 1000]
loop_counter_acum = []
counter = 0

while robot.step(TIME_STEP) != -1:

    # Getting sonar sensor values
    dsValues = []
    for i in range(MAX_SONAR_SENSOR_NUMBER):
        dsValues.append(round(ds_[i].getValue(), 1))
    
    average_sonar_sensor = round(sum(dsValues) / MAX_SONAR_SENSOR_NUMBER, 2)
    average_sonar_sensor_acum.append(average_sonar_sensor)

        # Function for standard deviation calculation of n last values
    def std_dev(n):
        valuesn = []
        s_ = 0
            # Picking n last elements of the average_sonar_sensor_acum
        for c in range(-1, -n -1, -1):
            valuesn.append(average_sonar_sensor_acum[c])

            # Average
        av = sum(valuesn) / n
            # x - average
        for k in valuesn:
            partial = (k - av) ** 2
            s_ += partial
        
            # Standard deviation
        s_ = (s_ / n) ** 0.5
        return s_
    
    k = std_dev(10)
    print(f'Standard deviation of 10 last average laser values {k}')

    # Getting position sensor values
    psValues = []
    for i in range(MAX_POSITION_SENSOR_NUMBER):
        psValues.append(ps_[i].getValue())

    # Getting light sensor values
    ls_value = ls.getValue()

    # Switch for sonar laser regions
    def Sonar_distance(argument):
        switcher = {
            'front' : max(dsValues[3], dsValues[4]),
            'fleft' : max(dsValues[0], dsValues[1], dsValues[2]),
            'fright' : max(dsValues[5], dsValues[6], dsValues[7])
        }
        return switcher.get(argument, 'nothing')
    
    # Obstacles boolean flag
    dist_param = 893 #860 890
    percentile_velocity = 0.9 #0.8

    front_obstacle = Sonar_distance('front') > dist_param or Sonar_distance('front') == 0
    fright_obstacle = Sonar_distance('fright') > dist_param
    fleft_obstacle = Sonar_distance('fleft') > dist_param

    front_no_obstacle = Sonar_distance('front') < dist_param
    fright_no_obstacle = Sonar_distance('fright') < dist_param
    fleft_no_obstacle = Sonar_distance('fleft') < dist_param
    
    print('Positon values', psValues)
    print('Distance sensor values')
    # Creating conditonal obstacles avoiding and setting motor velocity
    if front_no_obstacle and fleft_no_obstacle and fright_no_obstacle:
        print('Case 1 - Nothing')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED

        # Condition for obstacle edges
        if k < 1:
            print('I am blocked by an obstacle edge!')
            leftSpeed  = - 1 * MAX_SPEED
            rightSpeed = - 1 * MAX_SPEED

            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)

            leftSpeed  = percentile_velocity * MAX_SPEED
            rightSpeed = - percentile_velocity *MAX_SPEED

            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)

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

    # STOP CONDITION!
    if ls_value > 850:
        leftSpeed  = 0
        rightSpeed = 0

        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
     
        break
        
    # Setting velocity
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    # Loop counter
    counter += 1
    loop_counter_acum.append(counter)

    # Printing options
    c = 0
    for dist in dsValues:
        print(f's{c} {dist}')
        c += 1
    
    print(f'Light sensor value {ls_value}')
    print(f'Average sonar sensor distance {round(average_sonar_sensor, 2)}')
    print(f'Loop counter {loop_counter_acum[-1]}')