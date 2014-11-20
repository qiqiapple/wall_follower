#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_arduino_msgs/Encoders.h>
#include <wall_follower/MakeTurn.h>
#include <wall_follower/FollowWall.h>
#include <wall_follower/ResetPWM.h>
#include <math.h>

#define INVALID 1000

enum {FORWARD = 0, LEFT_TURN = 1, RIGHT_TURN = 2, FOLLOW_LEFT = 3, FOLLOW_RIGHT = 4, TWO_LEFT = 5};
  
int front_left, front_right, back_left, back_right, forward_left, forward_right, state;

class MazeController {

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber distance_sub;
    ros::Subscriber encoder_sub;
    int previous_state;
    int previous_sensor_reading[2];

    //Constructor
    MazeController() {
        n = ros::NodeHandle();
        distance_sub = n.subscribe("/ir_sensor_cm", 1, &MazeController::MazeCallback, this);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
        encoder_sub = n.subscribe("/arduino/encoders", 1, &MazeController::EncoderCallback, this);
        turn_client = n.serviceClient<wall_follower::MakeTurn>("/make_turn");
        follow_client = n.serviceClient<wall_follower::FollowWall>("/follow_wall");
        reset_client = n.serviceClient<wall_follower::ResetPWM>("/reset_pwm");
        previous_state = FORWARD;
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

    //Callback for the encoders
    void EncoderCallback(const ras_arduino_msgs::EncodersConstPtr &msg) {
        delta_encoder_left = msg->delta_encoder2;
        delta_encoder_right = msg->delta_encoder1;
        //ROS_INFO("Delta left: %d Delta right: %d", delta_encoder_left, delta_encoder_right);
    }

    //Method to make the robot drive forward
    void forward() {

        msg.linear.x = 0.1;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        publishMsg();
    }

    //Method for driving the robot a specific distance
    void forward(double distance) {

        double wheel_radius = 5.0;

        double distance_per_wheel = 2*M_PI*wheel_radius;
        double distance_per_tick = distance_per_wheel/360;

        int ticks = nearbyint(distance/distance_per_tick);

        msg.linear.x = 0.1;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        int left_encoder = 0;
        int right_encoder = 0;

        ros::Rate loop_rate(20);
        while (abs(left_encoder) < ticks && abs(right_encoder) < ticks) {

            ROS_INFO("Ticks to rotate: %d", ticks);
            ros::spinOnce();

            left_encoder += delta_encoder_left;
            right_encoder += delta_encoder_right;

            twist_pub.publish(msg);

            ROS_INFO("w = %f", msg.angular.z);
            ROS_INFO("Left encoder: %d Right encoder: %d", left_encoder, right_encoder);

            loop_rate.sleep();
        }
    }

    //Publishes a Twist message
    void publishMsg() {
        twist_pub.publish(msg);
    }

    //Sends service message through chosen client to perform desired task
    void setClientCall(int state) {
        if (state == LEFT_TURN || state == RIGHT_TURN) {
            srv_turn.request.state = state;
            srv_turn.request.degrees = 85;
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
        if (abs(front_left - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

   //Check if there is a rapid change in distance measured by ir sensors on the right side
    bool checkSensorsDistanceRight() {
        if (abs(front_right - previous_sensor_reading[0]) > 8) {
            return false;
        }

        return true;
    }

    //Checks whenever the two sensors have registered an edge of wall
    void checkSensorsTurn() {
        previous_sensor_reading[0] = front_left;
        previous_sensor_reading[1] = back_left;
        bool back = false;
        bool front = false;
	ros::Rate loop_rate(10);

        while (!back || !front) {
	    ros::spinOnce();
            if (front_left < 15) front = true;
            if (back_left < 15) back = true;
	    ROS_INFO("front: %d back: %d", front, back);
            forward();
	    loop_rate.sleep();
        }

	//forward(10.0);
	ros::spinOnce();
	//if (front_left > 25 && back_left > 25) setClientCall(LEFT_TURN);
	if (front_left > 25) {
		forward(10.0);
		setClientCall(LEFT_TURN);
	}

    }

    //Checks the ir sensors which decides what state to use
    int checkState() {
        int tresh_front = 17;
	int s;

       if ((forward_left < tresh_front &&
            forward_left > 0) ||
               (forward_right < tresh_front &&
               forward_right > 0)){
           if (front_left > front_right ||
                   back_left > back_right) {
                s = LEFT_TURN;
            }
            else
                s = RIGHT_TURN;
       }
       else{
           if (front_left < front_right &&
                   back_left < back_right &&
                   front_left < 30 &&
                   back_left < 30) {
                s = FOLLOW_LEFT;
           } else if (front_right < front_left &&
                      back_right < back_left &&
                      front_right < 30 &&
                      back_right < 30) {
                s = FOLLOW_RIGHT;
           }
            else
                s = FORWARD;
       }
       return s;
    }

private:
    ros::ServiceClient turn_client;
    ros::ServiceClient follow_client;
    ros::ServiceClient reset_client;
    wall_follower::MakeTurn srv_turn;
    wall_follower::FollowWall srv_follow;

    geometry_msgs::Twist msg;
    int delta_encoder_left;
    int delta_encoder_right;
};

 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "maze_follower");

    MazeController mc = MazeController();

    ros::Rate loop_rate(10);

    //Initial state
    state = FORWARD;

   while (ros::ok())
   {
       ros::spinOnce();

       //Returns the state the robot is in depending on the ir sensor readings
       state = mc.checkState();

       //Decides the state depending on the state transition
       if (mc.previous_state == FOLLOW_LEFT && state == FORWARD) {
            state = TWO_LEFT;
       }

       ROS_INFO("State: %d", state);

        switch (state) {
        case FORWARD:
            mc.forward();
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
                mc.forward(22.0);
            }
            break;

        case FOLLOW_RIGHT:
            if (state != mc.previous_state) {
                mc.previous_sensor_reading[0] = front_right;
                mc.previous_sensor_reading[1] = back_right;
            }
            if (mc.checkSensorsDistanceRight()) mc.setClientCall(state);
            else {
                mc.forward(22.0);
            }
            break;

        case TWO_LEFT:
            mc.forward(22.0);
            mc.setClientCall(LEFT_TURN);
            mc.checkSensorsTurn();
        }

       mc.previous_state = state;

       loop_rate.sleep();

   }

   return 0;
 }
