#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
//#include <pcimr_simulation::InitPos>

bool print_b;
ros::Publisher move_pub;
int frames_passed = 0;

void robotPosCallback(const sensor_msgs::LaserScan &scan)
{
   //set direction to north as default
   std::string direction= "N";

   if(scan.ranges[1] == 0){
       direction = "E";
   }

   std::cout << "Moving in Direction: " << direction << std::endl;

   std_msgs::String move_value;
   move_value.data = direction;
   move_pub.publish(move_value);
   frames_passed++;
}

void sensorMeasurementCallback(const geometry_msgs::Point &pos)
{
   if(pos.x == 16 && pos.y==12)
   {
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
   
   //ros::ServiceClient client = n.serviceClient<pcimr_simulation::InitPos>("/init_pos");
   //pcimr_simulation::InitPos srv;
   //srv.x=2;
   //srv.y=0;
   
   ros::Rate loop_rate(50);
   while (ros::ok())
   {
      ros::spinOnce();
      if (frames_passed > 100)
      {
         frames_passed = 0;
         //client.call(srv);
      }
      loop_rate.sleep();
   }
}
