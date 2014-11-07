#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
  
#include <sstream>

static int distance_sensor_leftfront, distance_sensor_rightfront, distance_sensor_leftback, distance_sensor_rightback;

void DistanceCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
 {
   distance_sensor_leftfront = msg->ch1;
   distance_sensor_rightfront = msg->ch2;
   distance_sensor_leftback = msg->ch3;
   distance_sensor_rightback = msg->ch4;
   ROS_INFO("dlf: [%d], drf: [%d], dlb: [%d], drb: [%d]", distance_sensor_leftfront, distance_sensor_rightfront, distance_sensor_leftback, distance_sensor_rightback);
 }

 int main(int argc, char **argv)
 {
   double alpha = -0.01; // To tune
   double beta = 0.01;  // To tune
   double diff_distance1, diff_distance2, angular_vel;
   ros::init(argc, argv, "wall_following_controller");
   ros::NodeHandle n;
   ros::Subscriber distance_sub = n.subscribe("/kobuki/adc", 1, DistanceCallback);
   ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

   geometry_msgs::Twist msg;
   msg.linear.x = 0.1;
   msg.linear.y = 0;
   msg.linear.z = 0;
   msg.angular.x = 0;
   msg.angular.y = 0;
  // msg.angular.z = 100;
   ros::Rate loop_rate(10);

   int count = 0;
   while (ros::ok())
   {
     ros::Subscriber distance_sub = n.subscribe("/kobuki/adc", 1, DistanceCallback);
     diff_distance1 = (double)(distance_sensor_rightfront - distance_sensor_rightback);
     diff_distance2 = (double)(distance_sensor_rightfront - distance_sensor_leftfront);
     angular_vel = alpha*diff_distance1 + beta*diff_distance2;
     msg.angular.z = angular_vel;
     twist_pub.publish(msg);
     ROS_INFO("v: %f, w: %f", msg.linear.x,msg.angular.z);

     ros::spinOnce(); 
     loop_rate.sleep();
     ++count;
   }
  
   return 0;
 }
