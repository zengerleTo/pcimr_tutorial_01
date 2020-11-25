
## Tutorial 02

The node for this tutorial is the "tutorial_kinematics_node" in the "tutorial_kinematics" package.

I'm not sure wether this is the intended way of running the node, but here are the steps that I took to get the node running with the correctly loaded parameters:

First call "roslaunch tutorial_kinematics launchfile.launch"
While this call crashes for me  with the error "cannot launch node of type ..." this does initialize the parameters.

When I then call "rosrun tutorial_kinematics tutorial_kinematics_node" the parameters are correctly set and the node starts.

### Does the controller work out of the box (for the Robotino)?
No it doesn't the topic for subscriber and publisher need to be changed. I handled this by adding another param "cmd_out_topic" that can either be set to "/cmd_vel" or "/pioneer/cmd_vel".

### Updates to the controller for holonomic movement
My initial implementation considered only the distance to objects in front of the robot.
For the use with omnidirectional motion I needed to change the code to use the distance in direction of the motion.

### Does the robot hit the wall?
As my tests were concerned the robot did not hit any walls, as long as it was moving in a direction covered by the sensor.
