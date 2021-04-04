# turtle_lawnmower

## Important information
ROS package expanding turtlesim options - allows to run turtlesim with a turtle imitating a lawnmower. The turtle starts in the lower left corner and continues going straight until it reaches the end, then it turns in a circular arc and continues in the opposite direction. This goes until the turtle covers the whole area.

### Prerequisites
To start with this project, ROS noetic needs to be installed and sourced on the computer.

### Running the program
After cloning this repository to your computer (and to your workspace's 'src' directory), simply run the command in a terminal:
```
roslaunch turtle_lawnmower turtlemower.launch
```
You might need to source the workspace:
```
source devel/setup.bash
```

### Authors
* **Emilia Szymańska**
