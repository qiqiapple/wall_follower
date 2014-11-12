#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <wall_follower/MakeTurn.h>
#include <wall_follower/FollowWall.h>
#include <wall_follower/ResetPWM.h>
#include <math.h>
  
int front_left, front_right, back_left, back_right, front;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;
    int previous_state;

    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<wall_follower::MakeTurn>("/make_turn");
        follow_client = n.serviceClient<wall_follower::FollowWall>("/follow_wall");
        reset_client = n.serviceClient<wall_follower::ResetPWM>("/reset_pwm");
        previous_state = 0;

    }

    void MazeCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        front_left = msg->ch1;
        front_right = msg->ch2;
        back_left = msg->ch3;
        back_right = msg->ch4;
        front = msg->ch5;
    }

    void forward() {

        wall_follower::ResetPWM srv;
        srv.request.reset = 2;

        if (previous_state != 0) {
            if (reset_client.call(srv)) {
                ROS_INFO("Succesfully called a service");
            } else
            {
                ROS_ERROR("Failed to call service. No turn performed.");
            }
        }

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

    void setClientCall(int state) {
        srv_turn.request.state = state;
        srv_follow.request.state = state;
        if (state == 1 || state == 2 || state == 5) {
            srv_turn.request.degrees = (state == 1 || state == 2) ? 90 : 180;
            if (turn_client.call(srv_turn)) {
                ROS_INFO("Succesfully called a service");
              }
              else
              {
                ROS_ERROR("Failed to call turn service in maze_follower");
              }
        }
        else if (state == 3 || state == 4) {
            if (follow_client.call(srv_follow)) {
                ROS_INFO("Succesfully called a service");
              }
              else
              {
                ROS_ERROR("Failed to call follow service in maze_follower");
              }
        }
    }

private:
    ros::ServiceClient turn_client;
    ros::ServiceClient follow_client;
    ros::ServiceClient reset_client;
    wall_follower::MakeTurn srv_turn;
    wall_follower::FollowWall srv_follow;

    geometry_msgs::Twist msg;
};

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "maze_follower");

    MazeController mc = MazeController();

    ros::Rate loop_rate(10);

    int thres_front = 20; //HERE TO CHANGE!
    int state = 0;

   while (ros::ok())
   {
       ros::spinOnce();

       if (front < thres_front && front > 0){
            if (front_left > 0 &&
                    front_left < 20 &&
                    front_right > 0 &&
                    front_right < 20)
               state = 5;
            else if (front_left > front_right) {// ||
                     //(front_right == 0 && front_left > 0))
                state = 2;
            else
                state = 1;
       }
       else{
           if ((front_left > 0 && (front_left < front_right || front_right == 0)) && (front_left < 30 && front_left > 0 && back_left < 30 && back_left > 0)) {
                state = 3;
           } else if ((front_right > 0 &&
                       (front_right < front_left || front_left == 0)) &&
                      (front_right < 30 && front_right > 0 && back_right < 30 && back_right >0)) {
                state = 4;
           }
            else
                state = 0;
       }

       ROS_INFO("State: %d", state);

       if (state == 0) {
        mc.forward();
        mc.publishMsg();
       } else {
        mc.setClientCall(state);
       }

       mc.previous_state = state;
       state = 0;

       loop_rate.sleep();

   }

   return 0;
 }
