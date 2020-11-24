#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

double vel_max;
double dist_decelerate;
double dist_stop;

ros::Publisher vel_pub;
geometry_msgs::Twist current_input_vel;

void sensorMeasurementCallback(const sensor_msgs::LaserScan &scan)
{
   geometry_msgs::Twist vel_out;
   vel_out.angular = current_input_vel.angular;
   vel_out.linear = current_input_vel.linear;
   
   //set the speed to vel_max if the robot has linear velocity
   double magnitude = std::sqrt(current_input_vel.linear.x*current_input_vel.linear.x + current_input_vel.linear.y*current_input_vel.linear.y + current_input_vel.linear.z*current_input_vel.linear.z);
   if(magnitude != 0.0){
      vel_out.linear.x = current_input_vel.linear.x / magnitude * vel_max;
      vel_out.linear.y = current_input_vel.linear.y / magnitude * vel_max;
      vel_out.linear.z = current_input_vel.linear.z / magnitude * vel_max;
   }
   //check the smallest distance for rougly 30 degree angle around the axis of motion
   double distance = scan.ranges[57];
   for(int i=58; i < 88; i++){
      if(scan.ranges[i] < distance){
         distance = scan.ranges[i];
      }
   }
   //std::cout << "Distance: " << distance << std::endl;
   
   //set the linear velocity to zero, if robot is too close to an obstacle
   if(distance < dist_stop){
      vel_out.linear.x = 0.0;
      vel_out.linear.y = 0.0;
      vel_out.linear.z = 0.0;
   }

   //use linear velocity scaling for distances between dist_decelerate and dist_stop
   else if(distance < dist_decelerate){
      double scaling = (distance - dist_stop)/(dist_decelerate-dist_stop);
      vel_out.linear.x = current_input_vel.linear.x * scaling;
      vel_out.linear.y = current_input_vel.linear.y * scaling;
      vel_out.linear.z = current_input_vel.linear.z * scaling;
      //std::cout << "Scaling Factor: " << scaling << std::endl;
   }
   vel_pub.publish(vel_out);

   std::cout << vel_out.linear << std::endl;
}

void inputVelocityCallback(const geometry_msgs::Twist &vel_in)
{
    current_input_vel = vel_in;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "tutorial_kinematics_node");
   ros::NodeHandle n("~");
   ros::Subscriber scan_sub = n.subscribe("/scan", 10, sensorMeasurementCallback);
   ros::Subscriber vel_sub = n.subscribe("/input/cmd_vel", 10, inputVelocityCallback);
   
   vel_pub = n.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1);
   
   ros::param::param<double>("/vel_max", vel_max, 0.5);
   ros::param::param<double>("/dist_stop", dist_stop, 0.3);
   ros::param::param<double>("/dist_decelerate", dist_decelerate, 2.1);
   std::cout << "Param: " << vel_max << std::endl;
   
   ros::Rate loop_rate(50);
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}