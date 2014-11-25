#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <wall_follower/MakeTurn.h>
#include <wall_follower/ResetPWM.h>
#include <std_msgs/Bool.h>
#include <math.h>
  
static int delta_encoder_left, delta_encoder_right;
ros::Publisher twist_pub;
ros::ServiceClient reset_client;
ros::Publisher turn_pub;

void EncoderCallback(const ras_arduino_msgs::EncodersConstPtr &msg) {
    delta_encoder_left = msg->delta_encoder2;
    delta_encoder_right = msg->delta_encoder1;
    //ROS_INFO("Delta left: %d Delta right: %d", delta_encoder_left, delta_encoder_right);
}

bool turn(wall_follower::MakeTurn::Request &req, wall_follower::MakeTurn::Response &res) {

    geometry_msgs::Twist msg;
    std_msgs::Bool turn;
    turn.data = true;
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
        msg.angular.z = 1.57/2;
    } else if (req.state == 2){
        msg.angular.z = -1.57/2;
    } else return true;

    ros::Rate loop_rate(20);

    int left_encoder = 0;
    int right_encoder = 0;

    wall_follower::ResetPWM srv;
    srv.request.reset = 1;

    if (reset_client.call(srv)) {
        ROS_INFO("Succesfully called a service");

        while (abs(left_encoder) < ticks && abs(right_encoder) < ticks) {//(abs(left_encoder) < ticks && abs(right_encoder) < ticks) {

            ROS_INFO("Ticks to rotate: %d", ticks);
            ros::spinOnce();

            left_encoder += delta_encoder_left;
            right_encoder += delta_encoder_right;

            twist_pub.publish(msg);
            turn_pub.publish(turn);

            ROS_INFO("w = %f", msg.angular.z);
            ROS_INFO("Left encoder: %d Right encoder: %d", left_encoder, right_encoder);

            loop_rate.sleep();
        }
    } else {
    ROS_ERROR("Failed to call service. No turn performed.");
    }

    if (reset_client.call(srv)) {
        ROS_INFO("Succesfully called a service");
    } else {
        ROS_ERROR("Failed to call service. No turn performed.");
    }

    msg.angular.z = 0;
    twist_pub.publish(msg);

    turn.data = false;
    turn_pub.publish(turn);

    return true;
}

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "turn_controller");
    ros::NodeHandle n;

    ros::Subscriber enc_sub = n.subscribe("/arduino/encoders", 1, EncoderCallback);
    twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    ros::ServiceServer service = n.advertiseService("/make_turn", turn);
    reset_client = n.serviceClient<wall_follower::ResetPWM>("/reset_pwm");
    turn_pub = n.advertise<std_msgs::Bool>("/robot_turn", 1);

    ROS_INFO("Ready to make a turn.");
    ros::spin();

    return 0;
 }
