#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
//#include "/home/tobias/ros_ws/tutorial_ws/devel/include/pcimr_simulation/InitPos.h"
#include <pcimr_simulation/InitPos.h>
bool print_b;
ros::Publisher move_pub;
int frames_passed = 0;
bool finished = false;

void sensorMeasurementCallback(const sensor_msgs::LaserScan &scan)
{
    if(finished){
        return;
    }
   //set direction to north as default
   std::string direction= "N";

   if(scan.ranges[2] <= 1){
       direction = "E";
   }
   std::cout << "Moving in Direction: " << direction << std::endl;

   std_msgs::String move_value;
   move_value.data = direction;
   move_pub.publish(move_value);
   frames_passed++;
}

void robotPosCallback(const geometry_msgs::Point &pos)
{
    if(finished){
        return;
    }
    
    std::cout << "The robot is currently at position: x=" << pos.x << ", y=" << pos.y << std::endl;

   if(pos.x == 16 && pos.y==12)
   {    
      finished = true;
      std::cout << "Successfully reached target" << std::endl;
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "grid_navigation_node");
   ros::NodeHandle n("~");
   ros::Subscriber scan_sub = n.subscribe("/scan", 10, sensorMeasurementCallback);
   ros::Subscriber pos_sub = n.subscribe("/robot_pos", 10, robotPosCallback);
   
   move_pub = n.advertise<std_msgs::String>("/move", 1);
   
   ros::ServiceClient client = n.serviceClient<pcimr_simulation::InitPos>("/init_pos");
   pcimr_simulation::InitPos srv;
   srv.request.x=2;
   srv.request.y=0;
   
   std::cout << "Starting Navigation" << std::endl;
   client.call(srv);
   ros::Rate loop_rate(50);
   while (ros::ok())
   {
      ros::spinOnce();
      if (frames_passed > 100)
      {
         frames_passed = 0;
      }
      loop_rate.sleep();
   }
}
