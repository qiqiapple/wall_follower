#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <wall_follower/MakeTurn.h>
#include <math.h>
  
static int delta_encoder_left, delta_encoder_right;
ros::Publisher twist_pub;

void EncoderCallback(const ras_arduino_msgs::EncodersConstPtr &msg) {
    delta_encoder_left = msg->delta_encoder2;
    delta_encoder_right = msg->delta_encoder1;
}

bool turn(wall_follower::MakeTurn::Request &req, wall_follower::MakeTurn::Response &res) {

    geometry_msgs::Twist msg;
    double base = 0.21;
    double wheel_radius = 0.05;

    double angle = req.degrees;
    double distance_per_wheel = 2*M_PI*wheel_radius;
    double distance_per_tick = distance_per_wheel/360;
    double distance_circuit = M_PI*base;

    double fraction_circuit = angle/(360);

    int ticks = nearbyint(fraction_circuit*distance_circuit/distance_per_tick);

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

    int left_encoder = 0;
    int right_encoder = 0;

    while (abs(left_encoder) < ticks && abs(right_encoder) < ticks) {

        left_encoder += delta_encoder_left;
        right_encoder += delta_encoder_right;

        twist_pub.publish(msg);

        loop_rate.sleep();
    }

    msg.angular.z = 0;
    twist_pub.publish(msg);
     /*for (int i = 0; i < 30; i++) {

        if (i < 10) {
            twist_pub.publish(msg);
         } else {
            msg.angular.z = 0;
            twist_pub.publish(msg);
        }
        //ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);
        loop_rate.sleep();
      }*/

    //ROS_INFO("v: %f, w: %f", msg.linear.x, msg.angular.z);

    return true;
}

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "turn_controller");
    ros::NodeHandle n;

    ros::Subscriber enc_sub = n.subscribe("/arduino/encoders", 1, EncoderCallback);
    twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::ServiceServer service = n.advertiseService("make_turn", turn);

    ROS_INFO("Ready to make a turn.");
    ros::spin();

    return 0;
 }
