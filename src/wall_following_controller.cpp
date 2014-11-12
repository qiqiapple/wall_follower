#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <wall_follower/FollowWall.h>
  
static int distance_sensor_leftfront, distance_sensor_rightfront, distance_sensor_leftback, distance_sensor_rightback;
ros::Publisher twist_pub;

void DistanceCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
 {
   distance_sensor_leftfront = msg->ch1;
   distance_sensor_rightfront = msg->ch2;
   distance_sensor_leftback = msg->ch3;
   distance_sensor_rightback = msg->ch4;
   //ROS_INFO("dlf: [%d], drf: [%d], dlb: [%d], drb: [%d]", distance_sensor_leftfront, distance_sensor_rightfront, distance_sensor_leftback, distance_sensor_rightback);
 }

bool follow(wall_follower::FollowWall::Request &req, wall_follower::FollowWall::Response &res) {

    geometry_msgs::Twist msg;
    int sensor1, sensor2, flag;

    if (req.state == 3) {
        sensor1 = distance_sensor_leftfront;
        sensor2 = distance_sensor_leftback;
        flag = -1;
    } else if (req.state == 4) {
        sensor1 = distance_sensor_rightfront;
        sensor2 = distance_sensor_rightback;
        flag = 1;
    } else return true;

    double alpha = flag*(-0.15);
    double diff_distance, angular_vel;

    if (sensor1 < 8 && sensor1 > 0 && sensor2 < 8 && sensor2 > 0) {

      msg.linear.x = 0.1;
      msg.angular.z = flag*0.314;

    } else {

     msg.linear.x = 0.1;

     diff_distance = (double)(sensor1 - sensor2);
     angular_vel = alpha*diff_distance;
     msg.angular.z = angular_vel;
    }

    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

    twist_pub.publish(msg);

    return true;

}

 int main(int argc, char **argv)
 {

    ros::init(argc, argv, "wall_following_controller");
    ros::NodeHandle n;
    ros::Subscriber distance_sub = n.subscribe("/ir_sensor_cm", 1, DistanceCallback);
    twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::ServiceServer service = n.advertiseService("/follow_wall", follow);

    ros::spin();

    return 0;
 }
