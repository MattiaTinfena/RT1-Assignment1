# Assignment1_rt

## Project Description

This project involves the development of a ROS package named `assignment1_rt`, which includes two main nodes:

1. **UI node**: 
   - Allows the user to control turtles `turtle1` and `turtle2` within the turtlesim environment.
   - Enables the user to set the speed of the selected turtle and send movement commands for 1 second.

2. **Distance node**: 
   - Computes the relative distance between `turtle1` and `turtle2` and publishes this information on a topic.
   - Stops the moving turtle if it gets too close to the other turtle or the environment boundaries.

## Features

### Node 1: UI
- **Spawns a new turtle** (`turtle2`) in the environment.
- A simple textual interface for:
  - Selecting the turtle to control (`turtle1` or `turtle2`).
  - Setting linear and angular velocities.
  - Sending movement commands for 1 second.
- After the command, the robot stops, and the user can input a new command.

### Node 2: Distance
- Computes and publishes the distance between `turtle1` and `turtle2` on a ROS topic.
- Stops the moving turtle if:
  - The distance between the two turtles is below a predefined threshold.
  - The turtles coordinates exceed the environment limits



## Prerequisites

1. **ROS** (Noetic) installed.
2. **Turtlesim**
 

## Installation

Clone the repository in your ROS enviroment:

```bash
git clone https://github.com/MattiaTinfena/RT1-Assignment1.git
```
Compile the nodes launching from the enviroment folder:

```bash
catkin_make
```
## Execution

Launch ROS:

```bash
roscore
```

Launch the turtlesim environment:

```bash
rosrun turtlesim turtlesim_node
```

Launch the UI node:

```bash
rosrun assignment_rt1 UInode
```
Launch the distance node:

```bash
rosrun assignment1_rt distanceNode
```
Monitor the topic for the distance between the 2 turtles:

```bash
rostopic echo /turtle_distance 
```
## Interact with the UI node to control the turtles

1. select 1 or 2 to control the correspondent turtle
2. digit the linear speed along the x axis and press enter
3. digit the linear speed along the y axis and press enter
4. digit the angular speed and press enter
5. digit q to quit

## Improvements

- [ ] Optimize and clean the code.
- [ ] Avoid that the distance node publishes the stop message more than once.

Feel free to contribute!