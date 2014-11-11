#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <wall_follower/MakeTurn.h>
#include <wall_follower/FollowWall.h>
#include <math.h>
  
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
        turn_client = n.serviceClient<wall_follower::MakeTurn>("make_turn");
        follow_client = n.serviceClient<wall_follower::FollowWall>("follow_wall");
    }

    void MazeCallback(const ras_arduino_msgs::ADConverterConstPtr &msg) {
        d1 = msg->ch1;
        d2 = msg->ch2;
        d3 = msg->ch3;
        d4 = msg->ch4;
        d5 = msg->ch5;
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

    void setClient(int state) {
        srv.request.state = state;
        if (state == 1 || state == 2) {
            client = turn_client;
            srv.request.degrees = (state == 1) ? 90 : -90;
        }
        else if (state == 3 || state == 4)
            client = follow_client;

    }

    void callService() {
        if (client.call(srv)) {
            ROS_INFO("Succesfully called a service");
          }
          else
          {
            ROS_ERROR("Failed to call service add_two_ints");
          }
    }

private:
    ros::ServiceClient turn_client;
    ros::ServiceClient follow_client;
    ros::ServiceClient client;
    wall_follower::MakeTurn srv;

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
         if(d1 < 30 && d1 > 0 && d3 < 30 && d3 > 0)
            state = 3;
         else if (d2 < 30 && d2 > 0 && d4 < 30 && d4 >0)
            state = 4;
         else
            state = 0;
       }

       ROS_INFO("State: %d", state);

       if (state == 0) {
        mc.stay();
        mc.publishMsg();
       } else {
        mc.setClient(state);
        mc.callService();
       }

       state = 0;
       if (state == 1 || state == 2) rate.sleep();
       else loop_rate.sleep();

   }

   return 0;
 }
