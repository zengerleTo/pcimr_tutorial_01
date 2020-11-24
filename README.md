
## Tutorial 02

### Does the controller work out of the box?
No it doesn't the topic for subscriber and publisher need to be changed.

### Updates to the Controller
My initial implementation considered only the distance to objects in front of the robot.
For the use with omnidirectional motion I needed to change the code to use the distance in direction of the motion.

## Tutorial 01: Linux, Software & ROS

Here is my implementation of the first tutorial.
I have created the node with the two subscribers and the publisher in grid_world/src/grid_navigation_node.
The callback for the position subscriber is only used to determine wether the robot reached the goal cell and prints a success message if it does.
In the sensor measurement callback I implemented the navigation policy.
Unless the distance to the northern wall is zero the robot will always go north.
In case it is zero the robot will go east. This will lead the robot to the goal state in the given setup.
