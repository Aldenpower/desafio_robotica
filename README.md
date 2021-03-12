# Robotics challenge
### INTRODUCTION
This respository contains files for simulating a world at the webots simulator
and a main controller to guide the robot through the world. The controller was
created for solving the maze in the world file. The robot starts at the start board
and go through the maze avoiding obstacles and must stop at the light point closer
to the stop board.
#### _File organization_
- resources > Base files
- webots_project > Webots simulator files

#### _Setup_
- Follow the instalation instructions of the webots simulator https://cyberbotics.com/doc/guide/installing-webots
- Clone this repository with git clone https://github.com/Aldenpower/desafio_robotica.git
- Open the world file (webots_project/worlds/pioneer3dx_desafio.wbt) inside the webots and run the simulation 
### IMPLEMENTATION DETAILS
- The controller : webots_project/controllers/sensors_obstacles/sensors_obstacles.py
#### _Importing python classes_
```sh
from controller import Robot
from controller import DistanceSensor
from controller import LightSensor
```

- Robot > The Robot class is a basis node for building a robot.
- DistanceSensor > The DistanceSensor class is a node that can be used
    for modeling generic sensor. In this case, the Pioneer3dx
    robot uses a sonar sensor.
- LightSensor > The light sensor nodes is used for measure the irradiance
of light in a given direction

#### _Creating the robot object_
```sh
robot = Robot()
```
#### _Global variables_
At this point i created global variables that will be used to set parameters of the robot
```sh
TIME_STEP = 64 # ms 64
MAX_SPEED = 8 # 6.28
MAX_SENSOR_NUMBER = 16
```
- TIME_STEP > Time step increment used by Webots to advance the virtual
time in miliseconds (ms) 
- MAX_SPEED > Maximum velocity ot the robot
- MAX_SENSOR_NUMBER > Number of the sonar sensors in the Pioneed3dx

#### _Initializing sonar sensors_
Than i set an empty list for receiving the object of the sonar distance sensors and a list
with the name of the distance sensors of the Piorneer3dx robot
```sh
ds_ = []
dsname = ['so0', 'so1', 'so2', 'so3', 'so4',
          'so5', 'so6', 'so7', 'so8', 'so9',
          'so10', 'so11', 'so12', 'so13',
          'so14', 'so15'
         ]
```
With the for loop in the range of the total sensor number i put the objects distance sensor
in the list with the .getDevice robot method than enabled them
```sh
for i in range(MAX_SONAR_SENSOR_NUMBER):
    ds_.append(robot.getDevice(dsname[i]))
    ds_[i].enable(TIME_STEP)
```
#### _Running simulation_

```sh
average_sonar_sensor_acum = [1000, 0, 1000, 0, 1000,
                             1000, 0, 1000, 0, 1000]
loop_counter_acum = []
counter = 0
```
- average_sonar_sensor_acum > Definig a list with values that result in a standard
deviation greater than 1 for conditional considerations
- loop_counter_acum > Defining an empty list for counting the while loops
- counter > Setting the counter to 0

Starting the while loop with a True condition
```sh
while robot.step(TIME_STEP) != -1:
```
The dsValues empty list will receive through the for loop, sensor values of the sonar distance
sensor. The loop will get the name of the sensor and iterate with the getValue method to append
the list with the values.
```sh
dsValues = []
for i in range(MAX_SONAR_SENSOR_NUMBER):
    dsValues.append(round(ds_[i].getValue(), 1))
```
Appending the average_sonar_sensor with average values of the sonar distance sensors
```sh
average_sonar_sensor = round(sum(dsValues) / MAX_SONAR_SENSOR_NUMBER, 2)
average_sonar_sensor_acum.append(average_sonar_sensor)
```
- average_sonar_sensor > Defining the average value of the distances of the sensors

Function for standard deviation calculation of n last values of the laser values
```sh
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
```
Defining the calculation of the standard deviation of the 10 last values of the laser
sonar laser values 
```sh
k = std_dev(10)
print(f'Standard deviation of 10 last average laser values {k}')
```
Getting light sensor values
```sh
ls_value = ls.getValue()
```
Creating a function to get the name of the region of the robot lasers. In the dictionary
i set the respective maximum value of a laser range that i defined with the names 'front' for
the front of the robot, 'fleft' for the front lef t of the robot and 'fright' for the front right
of the robot. The get dictionary method return the respective value of the maximum sensor
value.

```sh
def Sonar_distance(argument):
    switcher = {
        'front' : max(dsValues[3], dsValues[4]),
        'fleft' : max(dsValues[0], dsValues[1], dsValues[2]),
        'fright' : max(dsValues[5], dsValues[6], dsValues[7])
    }
    return switcher.get(argument, 'nothing')
```
The value dist_param simulate the maximum value of the robot sensor before colide with an obstacle,
values closer to 1000 will take robot closer to the obstacles. The percntile_velocity simply get
a portion of the total velocity of the robot for future alterations.
```sh
dist_param = 893 #860 890
percentile_velocity = 0.9 #0.8
```
With the variables front_obstacle, fright_obstacle, fleft_ obstacle, front_no_obstacle, fright_no_obstacle,
fleft_no_obstacle receive a boolean value that will represents the presence or not of obstacles in the regions
of the sensor lasers.
```sh
front_obstacle = Sonar_distance('front') > dist_param or Sonar_distance('front') == 0
fright_obstacle = Sonar_distance('fright') > dist_param
fleft_obstacle = Sonar_distance('fleft') > dist_param

front_no_obstacle = Sonar_distance('front') < dist_param
fright_no_obstacle = Sonar_distance('fright') < dist_param
fleft_no_obstacle = Sonar_distance('fleft') < dist_param
    
print('Positon values', psValues)
print('Distance sensor values')
```
With conditional if statement, i tested the respective cases of the state of the robot relatively to the obstacles
and set the velocity of the robot based on them. The variables leftSpeed and rightSpeed modify the wheel velocity
for the convinient movement of the robot.
```sh
if front_no_obstacle and fleft_no_obstacle and fright_no_obstacle:
        print('Case 1 - Nothing')
        leftSpeed  = percentile_velocity * MAX_SPEED
        rightSpeed = percentile_velocity * MAX_SPEED


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
```
The last if conditional will stop de robot and break the loop if the light sensor value is greater than 850
```sh
if ls_value > 850:
   leftSpeed  = 0
   rightSpeed = 0
   leftMotor.setVelocity(leftSpeed)
   rightMotor.setVelocity(rightSpeed)
     
   break
```
Finally the velocity will be updated based on the conditional statement and move the robot through
the maze avoiding obstacles.
```sh
leftMotor.setVelocity(leftSpeed)
rightMotor.setVelocity(rightSpeed)

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
```
### CONCLUSION
This repository was created for solving the challenge that refers to Robótica e Sistemas Autônomos - SENAI 2021

The controller can be changed, feel free to contribute with this repository!
