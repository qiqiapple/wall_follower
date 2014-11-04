#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
  
#include <sstream>

void TurnCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
 {
   distance_sensor_left = msg->ch1; 
   distance_sensor_right = msg->ch3; 
   distance_sensor_front = msg->ch5;
   ROS_INFO("left: [%d], right: [%d], front: [%d]", distance_sensor_left, distance_sensor_right, distance_sensor_front);
 }

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "turn_controller");
   ros::NodeHandle n;

   ros::Subscriber turn_sub = n.subscribe("kobuki/adc", 1, TurnCallback);
   //  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
   ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  
   thres_front = 10; //HERE TO CHANGE!
   if (distance_sensor_front < thres_front){
     if (distance_sensor_left<distance_sensor_right)
       turn_flag = 1; //left
     else
       turn_flag = -1; //right
   }

   ros::Rate loop_rate(10);
   int count = 0;
   while (ros::ok() && count<10)
   {
     geometry_msgs::Twist msg;

     msg.linear.x = 0.314;
     msg.linear.y = 0;
     msg.linear.z = 0;
     msg.angular.x = 0;
     msg.angular.y = 0;
     msg.angular.z = turn_flag * 0.314; //finish the turn in 5 seconds
  
     ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);

     twist_pub.publish(msg);
  
     ros::spinOnce(); 
     loop_rate.sleep();
     ++count;
   }
   return 0;
 }
