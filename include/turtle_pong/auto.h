#include <ros/ros.h>
#include <ros/subscriber.h>
#include <turtlesim/Pose.h>

// Service includes
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>

// topic type includes
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

#define deg2rad(angle) (angle * M_PI / 180.0)

enum eDir { 
    STOP = 0, 
    LEFT = 1, 
    UP_LEFT = 2,
    DOWN_LEFT = 3,
    RIGHT = 4,
    UP_RIGHT = 5,
    DOWN_RIGHT = 6
};

class Auto
{
public:
    Auto(ros::NodeHandle &nh);

    void pose0Callback(const turtlesim::PoseConstPtr& pose);
    void pose1Callback(const turtlesim::PoseConstPtr& pose);
    void pose2Callback(const turtlesim::PoseConstPtr& pose);
    void pose3Callback(const turtlesim::PoseConstPtr& pose);
    void pose4Callback(const turtlesim::PoseConstPtr& pose);
    void poseLeftCallback(const turtlesim::PoseConstPtr& pose);
    void poseRightCallback(const turtlesim::PoseConstPtr& pose);
    void move();
    void reset();
    void updateDirection();

    turtlesim::PoseConstPtr pose_ball_;
    turtlesim::PoseConstPtr pose_left_;
    turtlesim::PoseConstPtr pose_right_;

    eDir direction_;
    int num;

    // std::vector<double> section_x;
    // std::vector<double> section_y;
    double section_x[5];
    double section_y[5];



private:
    ros::NodeHandle& nh_;
    ros::Subscriber pose0_sub_;
    ros::Subscriber pose1_sub_;
    ros::Subscriber pose2_sub_;
    ros::Subscriber pose3_sub_;
    ros::Subscriber pose4_sub_;
    ros::Subscriber pose_left_sub_;
    ros::Subscriber pose_right_sub_;
    ros::Publisher cmd_vel_left_pub_;
    ros::Publisher cmd_vel_right_pub_;
    ros::ServiceClient teleport_abs_client_;




};