#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
  
#include <sstream>

int d1, d2, d3, d4, d5;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;

    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
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
       ros::Rate loop_rate(10);

       msg.linear.x = 0;
       msg.linear.y = 0;
       msg.linear.z = 0;
       msg.angular.x = 0;
       msg.angular.y = 0;
       msg.angular.z = 0.314;
       for (int i = 0; i < 30; i++) {
           if (i < 10) {
               msg.angular.z = 1.57;
               publishMsg();
            } else {
            msg.angular.z = 0;
            publishMsg();
           }
           //ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
           loop_rate.sleep();
        }

    }

    void TurnRight()
    {
        ros::Rate loop_rate(10);

        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = -0.314;

         for (int i = 0; i < 30; i++) {

            if (i < 10) {
                 msg.angular.z = -1.57;
                publishMsg();
             } else {
                msg.angular.z = 0;
                publishMsg();
            }
            //ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
            loop_rate.sleep();
          }

    }

    void StraightLeft()
    {
       double alpha = 0.2;
       double diff_distance, angular_vel;

       if (d1 < 8 && d1 > 0 && d3 < 8 && d3 > 0) {

         msg.linear.x = 0.1;
         msg.angular.z = -0.314;

       } else {

        msg.linear.x = 0.1;

        diff_distance = (double)(d1 - d3);
        angular_vel = alpha*diff_distance;
        msg.angular.z = angular_vel;
       }

       msg.linear.y = 0;
       msg.linear.z = 0;
       msg.angular.x = 0;
       msg.angular.y = 0;

    }

    void StraightRight()
    {
       double alpha = -0.2;
       double diff_distance, angular_vel;

       if (d2 < 8 && d2 > 0 && d4 < 8 && d4 > 0) {

         msg.linear.x = 0.1;
         msg.angular.z = 0.314;

       } else {

        msg.linear.x = 0.1;

        diff_distance = (double)(d2 - d4);
        angular_vel = alpha*diff_distance;
        msg.angular.z = angular_vel;
       }

       msg.linear.y = 0;
       msg.linear.z = 0;
       msg.angular.x = 0;
       msg.angular.y = 0;

    }

    void stay() {
        msg.linear.x = 0.1;
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
    geometry_msgs::Twist msg;
};

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "maze_follower");

   MazeController mc = MazeController();

   ros::Rate loop_rate(10);
   ros::Rate rate(1/5);

   int thres_front = 30; //HERE TO CHANGE!
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
         if(d2 < 30 && d2 > 0 && d4 < 30 && d4 > 0)
            state = 3;
         else if (d1 < 30 && d1 > 0 && d3 < 30 && d3 >0)
            state = 4;
         else
            state = 0;
       }

       ROS_INFO("State: %d", state);

       switch(state){
        case 0:
           mc.stay();
           break;
        case 1:
             //turn left;
           mc.TurnLeft();
           break;
         case 2:
             //turn right;
           mc.TurnRight();
           break;
         case 3:
             //walk straight along the right wall
           mc.StraightRight();
           break;
         case 4:
             //walk straight along the left wall
           mc.StraightLeft();
        }
       mc.publishMsg();
       state = 0;
       if (state == 1 || state == 2) rate.sleep();
       else loop_rate.sleep();

   }

   return 0;
 }
