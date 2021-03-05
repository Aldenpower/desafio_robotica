"""move_forward controller."""

from controller import Robot
from controller import Motor

# get the time step of the current world.
TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# GetDevice function
leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')

# The wheel once rotated 10 radians
leftMotor.setPosition(10.0)
rightMotor.setPosition(10.0)

# Main loop:
while robot.step(TIME_STEP) != -1:
    pass