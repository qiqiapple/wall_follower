#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include <wall_follower/MakeTurn.h>
  
static int distance_sensor_left, distance_sensor_right, distance_sensor_front;
ros::Publisher twist_pub;

/*void TurnCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
 {
   distance_sensor_left = msg->ch1; 
   distance_sensor_right = msg->ch2; 
   distance_sensor_front = msg->ch5;
   ROS_INFO("left: [%d], right: [%d], front: [%d]", distance_sensor_left, distance_sensor_right, distance_sensor_front);
 }*/

bool turn(wall_follower::MakeTurn::Request &req, wall_follower::MakeTurn::Response &res) {

    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

    if (req.state == 1) {
        msg.angular.z = 1.57;
    } else {
        msg.angular.z = -1.57;
    }

    ros::Rate loop_rate(10);

     for (int i = 0; i < 30; i++) {

        if (i < 10) {
            twist_pub.publish(msg);
         } else {
            msg.angular.z = 0;
            twist_pub.publish(msg);
        }
        //ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
        loop_rate.sleep();
      }

    ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);

    return true;
}

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "turn_controller");
    ros::NodeHandle n;

    //ros::Subscriber turn_sub = n.subscribe("/ir_sensor_cm", 1, TurnCallback);
    twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::ServiceServer service = n.advertiseService("make_turn", turn);

    ROS_INFO("Ready to make a turn.");
    ros::spin();

    return 0;
 }
