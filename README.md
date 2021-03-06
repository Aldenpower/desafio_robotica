# DESAFIO DE ROBOTICA

To solve the challenge, i created a controller called sensors_obstacles.py loc
ated at the folder webots_project/controllers/sensor_obstacles/

# THE CONTROLLER

from controller import Robot
from controller import DistanceSensor
from controller import PositionSensor

First i imported the classes that i will use on the script:

    - The Robot class is a basis node for building a robot.

    - The DistanceSensor class is a node that can be used
    to model generic sensor. In that case, the Pioneer3dx
    robot uses a sonar sensor.

    - The PositionSensor class (...)

robot = Robot()

From the class Robot i created a object called robot.

TIME_STEP = 64 # ms 64
MAX_SPEED = 8 # 6.28
MAX_SENSOR_NUMBER = 16
MAX_SENSOR_POSITON = 2

At this point i created global variables that will be used
to set parameters of the robot.

    - TIME_STEP (...)

    - MAX_SPEED will set the maximum velocity ot the robot

    - MAX_SENSOR_NUMBER will set the number of sonar dist
    ance sensor of the robot

    - MAX_SENSOR_POSITION (...)

ds_ = []
dsname = ['so0', 'so1', 'so2', 'so3', 'so4',
          'so5', 'so6', 'so7', 'so8', 'so9',
          'so10', 'so11', 'so12', 'so13',
          'so14', 'so15'
         ]

Than i set a empty list to receive the object of the sonar
distance sensors and a list with the name of the distance
sensors of the Piorneer3dx robot.

for i in range(MAX_SENSOR_NUMBER):
    ds_.append(robot.getDevice(dsname[i]))
    ds_[i].enable(TIME_STEP)

With the for loop in the range of the total sensor numbers
i put the objects distance sensor in the list with the .get
Device robot method than enabled them.

leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')

The leftMotor is a device of the robot that represents
the left wheel of the Pioneer3dx, the rightMotor is a
device of the robot that represents the right wheel of
the Pioneer3dx. Through the getDevice method of the robot
i called them in two variables.

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

With the setVelocity method i set the position of the
motors to float('inf') for no movement of the robot and
set the setVelocity to 0.

while robot.step(TIME_STEP) != -1:

With the TIME_STEP = 64 the robot will enter in the while 
loop and repeat them until receive an exit command.

dsValues = []
    for i in range(MAX_SENSOR_NUMBER):
        dsValues.append(round(ds_[i].getValue(), 1))

The dsValues empty list will receive through the for loop,
sensor values of the sonar distance sensor. The loop will
get the name of the sensor and iterate with the getValue
method to append the list with the values.

def Sonar_distance(argument):
        switcher = {
            'front' : max(dsValues[3], dsValues[4]),
            'fleft' : max(dsValues[0], dsValues[1], dsValues[2]),
            'fright' : max(dsValues[5], dsValues[6], dsValues[7])
        }
        return switcher.get(argument, 'nothing')

Then i created a function to get the name of the region of
the robot lasers. In the dictionary i set the respective
maximum value of a laser range that i defined with the names
'front' for the front of the robot, 'fleft' for the front lef
t of the robot and 'fright' for the front right of the robot.
The get dictionary method return the respective value of the
maximum value sensor.

dist_param = 860 #860
percentile_velocity = 0.9 #0.8

The value dist_param simulate the maximum value of the robot
sensor before colide with an obstacle, values closer to 1000
will take robot closer to the obstacles. The percntile_veloc
ity simply get a portion of the total velocity of the robot
for future alterations.

front_obstacle = Sonar_distance('front') > dist_param or 
Sonar_distance('front') == 0
fright_obstacle = Sonar_distance('fright') > dist_param
fleft_obstacle = Sonar_distance('fleft') > dist_param
front_no_obstacle = Sonar_distance('front') < dist_param
fright_no_obstacle = Sonar_distance('fright') < dist_param
fleft_no_obstacle = Sonar_distance('fleft') < dist_param

With the variables front_obstacle, fright_obstacle, fleft_
obstacle, front_no_obstacle, fright_no_obstacle, fleft_no_
obstacle and receive a boolean value that will represents
the presence of obstacles in the regions of the sensor la
sers.

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

With conditional if statement i test the respective cases
of the state of the robot relatively to the obstacles
and set the velocity of the robot based on them. The vari
ables leftSpeed and rightSpeed modify the wheel velocity
for the convinient movement of the robot.

leftMotor.setVelocity(leftSpeed)
rightMotor.setVelocity(rightSpeed)

Finally the velocity will be updated based on the conditi
onal statement and move the robot through the maze avoidi
ng obstacles.