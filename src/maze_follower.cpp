#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <wall_follower/MakeTurn.h>
#include <wall_follower/FollowWall.h>
#include <wall_follower/ResetPWM.h>
#include <math.h>

#define INVALID 1000

enum {FORWARD = 0, LEFT_TURN = 1, RIGHT_TURN = 2, FOLLOW_LEFT = 3, FOLLOW_RIGHT = 4};
  
int front_left, front_right, back_left, back_right, forward_left, forward_right, state;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;
    int previous_state;
    int previous_sensor_reading[2];

    //Constructor
    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        turn_client = n.serviceClient<wall_follower::MakeTurn>("/make_turn");
        follow_client = n.serviceClient<wall_follower::FollowWall>("/follow_wall");
        reset_client = n.serviceClient<wall_follower::ResetPWM>("/reset_pwm");
        previous_state = 0;
        previous_sensor_reading[0] = 0;
        previous_sensor_reading[1] = 0;

    }

    //Destructor
    ~MazeController() {}

    //Callback for using IR sensor values
    void MazeCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        front_left = msg->ch1;
        front_right = msg->ch2;
        back_left = msg->ch3;
        back_right = msg->ch4;
        forward_right = msg->ch5;
        forward_left = msg->ch6;
    }

    //Method to make the robot drive forward
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

    //Publishes a Twist message
    void publishMsg() {
        twist_pub.publish(msg);
    }

    //Sends service message through chosen client to perform desired task
    void setClientCall(int state) {
        if (state == LEFT_TURN || state == RIGHT_TURN) {
            srv_turn.request.state = state;
            srv_turn.request.degrees = 90;
            if (turn_client.call(srv_turn)) {
                ROS_INFO("Succesfully called a service");
              }
              else
              {
                ROS_ERROR("Failed to call turn service in maze_follower");
              }
        }
        else if (state == FOLLOW_LEFT || state == FOLLOW_RIGHT) {
            srv_follow.request.state = state;
            if (follow_client.call(srv_follow)) {
                ROS_INFO("Succesfully called a service");
              }
              else
              {
                ROS_ERROR("Failed to call follow service in maze_follower");
              }
        }
    }

    //Outputs if the state changes
    void changeState(int s) {
        std::cout << "want to change the state " << s << std::endl;
        std::cin.ignore();
        state = s;
    }

    //Check if there is a rapid change in distance measured by ir sensors on the left side
    bool checkSensorsDistanceLeft() {
        if (abs(front_left - previous_sensor_reading[0]) > 10) {
            return false;
        } else if (abs(front_left - back_left) < 10 || state != previous_state) {
            previous_sensor_reading[0] = front_left;
            previous_sensor_reading[1] = back_left;
        }

        return true;
    }

    //Check if there is a rapid change in distance measured by ir sensors on the right side
    bool checkSensorsDistanceRight() {
        if (abs(front_right - previous_sensor_reading[0]) > 10) {
            return false;
        } else if (abs(front_right - back_right) < 10 || state != previous_state) {
            previous_sensor_reading[0] = front_right;
            previous_sensor_reading[1] = back_right;
        }

        return true;
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

    //Treshold for when the robot should react on obstacles in front of it
    int tresh_front = 15;
    state = FORWARD;

   while (ros::ok())
   {
       ros::spinOnce();

       if (forward_left < tresh_front &&
               forward_left > 0 &&
               forward_right < tresh_front &&
               forward_right > 0){
           if (front_left > front_right ||
                   back_left > back_right) {
                state = LEFT_TURN;
            }
            else
                state = RIGHT_TURN;
       }
       else{
           if (front_left < front_right &&
                   back_left < back_right &&
                   front_left < 30 &&
                   back_left < 30) {
                state = FOLLOW_LEFT;
           } else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 30 &&
                      back_right < 30) {
                state = FOLLOW_RIGHT;
           }
            else
                state = FORWARD;
       }

       ROS_INFO("State: %d", state);

        switch (state) {
        case FORWARD:
            mc.forward();
            mc.publishMsg();
            break;

        case LEFT_TURN:
            mc.setClientCall(state);
            break;

        case RIGHT_TURN:
            mc.setClientCall(state);
            break;

        case FOLLOW_LEFT:
            if (state != mc.previous_state) {
                mc.previous_sensor_reading[0] = front_left;
                mc.previous_sensor_reading[1] = back_left;
            }
            if (mc.checkSensorsDistanceLeft())
                mc.setClientCall(state);
            else {
                mc.forward();
                mc.publishMsg();
            }
            break;

        case FOLLOW_RIGHT:
            if (state != mc.previous_state) {
                mc.previous_sensor_reading[0] = front_right;
                mc.previous_sensor_reading[1] = back_right;
            }
            if (mc.checkSensorsDistanceRight()) mc.setClientCall(state);
            else {
                mc.forward();
                mc.publishMsg();
            }
            break;
        }

/*
       if (state == FORWARD) {
        mc.forward();
        mc.publishMsg();
       } else {
        mc.setClientCall(state);
       }
*/
       mc.previous_state = state;

       loop_rate.sleep();

   }

   return 0;
 }
