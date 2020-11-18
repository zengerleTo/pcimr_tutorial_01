
## Tutorial 01: Linux, Software & ROS

### Description

Here is implementation of the first tutorial.
I have created the node with the two subscribers and the publisher in grid_world/src/grid_navigation_node.
The callback for the position subscriber is only used to determine wether the robot reached the goal cell and prints a success message if it does.
In the sensor measurement callback I implemented the navigation policy.
Unless the distance to the northern wall is zero the robot will always go north.
In case it is zero the robot will go east. This will lead the robot to the goal state in the given setup.
