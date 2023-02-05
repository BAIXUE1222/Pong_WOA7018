#include <turtle_pong/auto.h>

Auto::Auto(ros::NodeHandle &nh)
    : nh_(nh)
{

    ROS_INFO("Initialize Turtle Ball");

    pose0_sub_ = nh_.subscribe<turtlesim::Pose>("/ball0/pose", 1, &Auto::pose0Callback, this);
    pose1_sub_ = nh_.subscribe<turtlesim::Pose>("/ball1/pose", 1, &Auto::pose1Callback, this);
    pose2_sub_ = nh_.subscribe<turtlesim::Pose>("/ball2/pose", 1, &Auto::pose2Callback, this);
    pose3_sub_ = nh_.subscribe<turtlesim::Pose>("/ball3/pose", 1, &Auto::pose3Callback, this);
    pose4_sub_ = nh_.subscribe<turtlesim::Pose>("/ball4/pose", 1, &Auto::pose4Callback, this);
    pose_left_sub_ = nh_.subscribe<turtlesim::Pose>("/turtle_left/pose", 1, &Auto::poseLeftCallback, this);
    pose_right_sub_ = nh_.subscribe<turtlesim::Pose>("/turtle_right/pose", 1, &Auto::poseRightCallback, this);
    cmd_vel_left_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle_left/cmd_vel", 1);
    cmd_vel_right_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle_right/cmd_vel", 1);


    direction_ = STOP;
    for (int i = 0; i < 5; i++)
    {
        section_x[i] = 0;
        section_y[i] = 0;
    }


}


void Auto::updateDirection()
{
    double theta = pose_ball_->theta;
    if (0.0 == theta)
    {
        direction_ = RIGHT;
    }
    else if (0.0 < theta && theta < M_PI_2)
    {
        direction_ = UP_RIGHT;
    }
    else if (M_PI_2 < theta && theta < M_PI)
    {
        direction_ = UP_LEFT;
    }
    else if (M_PI == theta)
    {
        direction_ = LEFT;
    }
    else if (-M_PI < theta && theta < -M_PI_2)
    {
        direction_ = DOWN_LEFT;
    }
    else if (-M_PI_2 < theta && theta < 0.0)
    {
        direction_ = DOWN_RIGHT;
    }
}


void Auto::pose0Callback(const turtlesim::PoseConstPtr& pose)
{
    section_x[0] = pose->x;
    section_y[0] = pose->y;
    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};
void Auto::pose1Callback(const turtlesim::PoseConstPtr& pose)
{
    section_x[1] = pose->x;
    section_y[1] = pose->y;
    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};

void Auto::pose2Callback(const turtlesim::PoseConstPtr& pose)
{
    section_x[2] = pose->x;
    section_y[2] = pose->y;
    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};

void Auto::pose3Callback(const turtlesim::PoseConstPtr& pose)
{
    section_x[3] = pose->x;
    section_y[3] = pose->y;
    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};

void Auto::pose4Callback(const turtlesim::PoseConstPtr& pose)
{
    section_x[4] = pose->x;
    section_y[4] = pose->y;
    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
        pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};






void Auto::poseLeftCallback(const turtlesim::PoseConstPtr& pose)
{
    pose_left_ = pose;
    int minPosition = min_element(section_x,section_x+5) - section_x;
    double dy;
    double speed_limit = 20;
    double k = 20;
    dy = section_y[minPosition] - pose_left_->y;
    geometry_msgs::Twist twist;
    twist.linear.x = k * dy;

    if (abs(twist.linear.x) > speed_limit)
    {
        twist.linear.x = abs(twist.linear.x) /twist.linear.x* speed_limit;

    }

    // if ((M_PI_2 < theta && theta < M_PI )||(-M_PI < theta && theta < -M_PI_2))
    // {
        cmd_vel_left_pub_.publish<geometry_msgs::Twist>(twist);
    // }

    ROS_DEBUG_THROTTLE(1, "Left: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
            pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);

};

void Auto::poseRightCallback(const turtlesim::PoseConstPtr& pose)
{
    pose_right_ = pose;
    int maxPosition = max_element(section_x,section_x+5) - section_x;
    double dy;
    double speed_limit = 20;
    double k = 20;
    dy = section_y[maxPosition] - pose_right_->y;
    geometry_msgs::Twist twist;
    twist.linear.x = k * dy;

    if (abs(twist.linear.x) > speed_limit)
    {
        twist.linear.x = abs(twist.linear.x) /twist.linear.x* speed_limit;

    }

    // if ((0.0 < theta && theta < M_PI_2)||(-M_PI_2 < theta && theta < 0.0))
    // {
        cmd_vel_right_pub_.publish<geometry_msgs::Twist>(twist);
    // }



    // std::cout << pose_right_->y << std::endl;
    ROS_DEBUG_THROTTLE(1, "right: x: %f, y: %f, theta: %f, linear_vel: %f, angular_vel: %f", 
            pose->x, pose->y, pose->theta, pose->linear_velocity, pose->angular_velocity);
    
};


void Auto::move()
{


}




int main(int argc, char** argv)
 {
    ros::init(argc, argv, "turtle_pong_auto");

    ros::NodeHandle nh;


    Auto Auto(nh);

    ros::Rate loop_rate(100);
    
    while(ros::ok())
    {
        Auto.move();
        ros::spinOnce();
        loop_rate.sleep();
        
    }
}