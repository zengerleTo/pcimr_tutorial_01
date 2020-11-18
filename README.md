
## Tutorial 01: Linux, Software & ROS

Disclaimer: I didn't get my node to run, so I could not verify if it works as intended.
Additionally I had issues with referencing the InitPos service, which is why the service call for the initial position is commented in my code.

### Description

Here is my implementation of the first tutorial.
I have created the node with the two subscribers and the publisher in grid_world/src/grid_navigation_node.
The callback for the position subscriber is only used to determine wether the robot reached the goal cell and prints a success message if it does.
In the sensor measurement callback I implemented the navigation policy.
Unless the distance to the northern wall is zero the robot will always go north.
In case it is zero the robot will go east. This will lead the robot to the goal state in the given setup.
(Since I couldn't run the node to check which sensor is responsible for which direction I just took the sensor value at index 1 as the northern value)
