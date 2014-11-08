#include "std_msgs/String.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
  
#include <sstream>

int d1, d2, d3, d4, d5;
//ros::NodeHandle n;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;

    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
        //loop_rate(10);
    }

    void MazeCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
     {
       d1 = msg->ch1;
       d2 = msg->ch2;
       d3 = msg->ch3;
       d4 = msg->ch4;
       d5 = msg->ch5;
       ROS_INFO("d1: [%d], d2: [%d], d3: [%d], d4: [%d], d5: [%d]", d1, d2, d3, d4, d5);
     }

    void TurnLeft()
    {
        //ros::Rate loop_rate(10);
       int count = 0;
       /*while (ros::ok() && count<10)
       {
         geometry_msgs::Twist msg;
*/
         msg.linear.x = 0;
         msg.linear.y = 0;
         msg.linear.z = 0;
         msg.angular.x = 0;
         msg.angular.y = 0;
         msg.angular.z = 0.314;

         ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
/*
         twist_pub.publish(msg);

         ros::spinOnce();
         loop_rate.sleep();
         ++count;
       }*/
    }

    void TurnRight()
    {
        //ros::Rate loop_rate(10);
       int count = 0;
       /*while (ros::ok() && count<10)
       {
         geometry_msgs::Twist msg;
*/
         msg.linear.x = 0;
         msg.linear.y = 0;
         msg.linear.z = 0;
         msg.angular.x = 0;
         msg.angular.y = 0;
         msg.angular.z = -0.314;

         ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
/*
         twist_pub.publish(msg);

         ros::spinOnce();
         loop_rate.sleep();
         ++count;
       }*/

    }

    void StraightLeft()
    {
        //ros::Rate loop_rate(10);
       double alpha = 0.05;
       double diff_distance, angular_vel;
       //geometry_msgs::Twist msg;
       msg.linear.x = 0;
       msg.linear.y = 0;
       msg.linear.z = 0;
       msg.angular.x = 0;
       msg.angular.y = 0;
       int count = 0;
       /*while (ros::ok())
       {
         //ros::Subscriber distance_sub = n.subscribe("/kobuki/adc", 1, MazeCallback);*/
         diff_distance = (double)(d1 - d3);
         angular_vel = alpha*diff_distance;
         msg.angular.z = angular_vel;
         /*
         twist_pub.publish(msg);
         ROS_INFO("v: %f, w: %f", msg.linear.x,msg.angular.z);

         ros::spinOnce();
         loop_rate.sleep();
         ++count;
       }*/

    }

    void StraightRight()
    {
       double alpha = -0.05;
       double diff_distance, angular_vel;
        //ros::Rate loop_rate(10);
       //geometry_msgs::Twist msg;
       msg.linear.x = 0;
       msg.linear.y = 0;
       msg.linear.z = 0;
       msg.angular.x = 0;
       msg.angular.y = 0;
       int count = 0;
       /*while (ros::ok())
       {
           //ros::Subscriber distance_sub = n.subscribe("/kobuki/adc", 1, MazeCallback);*/
         diff_distance = (double)(d2 - d4);
         angular_vel = alpha*diff_distance;
         msg.angular.z = angular_vel;
         /*
         twist_pub.publish(msg);
         ROS_INFO("v: %f, w: %f", msg.linear.x,msg.angular.z);

         ros::spinOnce();
         loop_rate.sleep();
         ++count;
       }*/

    }

    void stay() {
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
    }

    void publishMsg() {
        twist_pub.publish(msg);
    }

private:
    //ros::Rate loop_rate(10);
    geometry_msgs::Twist msg;
};

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "maze_follower");

   MazeController mc = MazeController();

   int thres_front = 10; //HERE TO CHANGE!
   int state = 0;

   while (ros::ok())
   {
       ros::spinOnce();

       if (d5 < thres_front && d5 > 0){
         if (d1 > d2)
           state = 1;
         else
           state = 2;
       }
       else{
         if(d2 < 30 && d2 > 0)
           state = 3;
         else if (d1 < 30 && d1 > 0)
           state = 4;
         else
             state = 0;
       }

       ROS_INFO("State: %d", state);

       switch(state){
        case 0:
           mc.stay();
           continue;
        case 1:
             //turn left;
           mc.TurnLeft();
           continue;
         case 2:
             //turn right;
           mc.TurnRight();
           continue;
         case 3:
             //walk straight along the right wall
           mc.StraightRight();
           continue;
         case 4:
             //walk straight along the left wall
           mc.StraightLeft();
        }
       mc.publishMsg();
       state = 0;
   }

   return 0;
 }
