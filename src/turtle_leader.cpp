#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

// Service includes
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/SetPen.h>



#include <math.h>
#include <iostream>
#include <cstdlib>



void colorSensorCallback(const turtlesim::ColorConstPtr& color)
{
    //ROS_INFO_THROTTLE(0.1, "Color received (r,g,b) = (%i,%i,%i)", color->r, color->g, color->b);
}


void spawnPlayerTurtle(std::string name, double x, double y, double theta)
{
    ROS_INFO("Spawn %s", name.c_str());
    turtlesim::Spawn spawn;
    spawn.request.name = name.c_str();
    spawn.request.x = x;
    spawn.request.y = y;
    spawn.request.theta = theta;
    ros::service::call<turtlesim::Spawn>("/spawn", spawn);

    turtlesim::SetPen set_pen;
    set_pen.request.off = true;
    std::string service_name = "/" + name + "/set_pen";
    ros::service::call(service_name, set_pen);
}

void Timecallback1(const ros::TimerEvent& event){  //回调函数
     turtlesim::Kill kill;
    kill.request.name = "turtle_leader";
    ros::service::call<turtlesim::Kill>("/kill", kill);
    double x = rand()%10;
    double y = rand()%10;
    spawnPlayerTurtle("turtle_leader", x, y, M_PI_2);

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pong");
    ros::NodeHandle nh;

    ROS_INFO("Reset and remove default turtle1 from the game");
    std_srvs::Empty empty;
    ros::service::call<std_srvs::Empty>("/reset", empty);
    turtlesim::Kill kill;
    kill.request.name = "turtle1";
    ros::service::call<turtlesim::Kill>("/kill", kill);


    double x = rand()%5+2.5;
    double y = rand()%5+2.5;
    spawnPlayerTurtle("turtle_leader", x, y, M_PI_2);

    ros::Timer timer= nh.createTimer(ros::Duration(5), Timecallback1);//定时器

    ros::spin();//回旋函数
    return 0;


    
}