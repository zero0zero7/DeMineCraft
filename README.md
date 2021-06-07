# DeMineCraft

# SnakeJoints
Adapted from "A Modified Serpenoid Equation for Snake Robots" (https://www.researchgate.net/publication/251884840_A_Modified_Serpenoid_Equation_for_Snake_Robots)

## Objective
The joints of the snake robot will need to be controlled, and due to the limitations of the motors, and the high gear ratios, they have to be controlled by angular velocity. This paper convertes the serpenoid curve to a discrete form, and provides an algorithm for the angular velocity for each joint. 

## Control
The motion of the snake is controlled by 4 parameters, refer to Fig.2 in the attached paper: 
- Speed (not velocity)
- Undulation, a
- Period, b
- Angular speed, c

## Usage (not tested)
1. Initialise instance
```
python 
## initialize parameters
a = math.pi/3
b = math.pi/2
c = 0
target_speed = 50
length_of_snake = 1500 # in mm
number_of_segments = 12

# instantiate object
snake_joints = SnakeJoints(a, b, c, number_of_segments, length_of_snake, target_speed)
snake_joints.start_movement() # timer will start here
```

2. Getting target_velocities
```python
phis = snake_joints.get_phis() # increment the timer and calculate the target angular velocity (in rad/s)values to be published
```

3. Controlling the snake
```python
# setting new parameters, mainly c to turn the snake
snake_joints.set_parameters(new_a, new_b, new_c)

# setting target speed
snake_joints.set_target_speed(new_target_speed)
```

## TODO
Wrap this module in a publisher in ROS. 

This is currently 2D movement on a plane, should be able to adapt to 3D. 


