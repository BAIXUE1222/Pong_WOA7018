#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <iostream>
 
std::string turtle_name;
std::string turtle_name1;
double dx = 0;
double dy = 0;
 
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion quaternion;
    
    transform.setOrigin(tf::Vector3(msg->x+dx, msg->y+dy, 0.0));
    quaternion.setRPY(0, 0, msg->theta);
    transform.setRotation(quaternion);
    // std::cout << turtle_name <<std::endl;
 
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name1));
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_broadcaster");
    if (argc != 2)
    {
        ROS_ERROR("need turtle name as argument"); 
        return -1;
    }

    std::cout << "ok" <<std::endl;
    std::cout <<argv[1] <<std::endl;

    turtle_name1 = argv[1]; // = argv[1]
    turtle_name = argv[1];

    if(strcmp(argv[1],"turtle_leaderA") ==0  )
        {
            dx = 1;
            dy = 0;
            turtle_name = "turtle_leader";
            // std::cout <<"ok11111111" <<std::endl;
        }
    if(strcmp(argv[1],"turtle_leaderB")  ==0)
        {
            dx = 0.4;
            dy = 0.4;
            turtle_name = "turtle_leader";
            // std::cout << argv[1] <<std::endl;
            
        }
    if(strcmp(argv[1],"turtle_leaderC")  ==0 )
        {
            dx = -0.4;
            dy = 0.4;
            turtle_name = "turtle_leader";
            // std::cout << argv[1] <<std::endl;
            std::cout <<"ok11111111" <<std::endl;
            
        }
     if(strcmp(argv[1],"turtle_leaderD")==0  )
        {
            dx = -0.4;
            dy = -0.4;
            turtle_name = "turtle_leader";
            // std::cout << argv[1] <<std::endl;
            
        }
    if(strcmp(argv[1],"turtle_leaderE")==0  )
        {
            dx = 0.4;
            dy = -0.4;
            turtle_name = "turtle_leader";
            // std::cout << argv[1] <<std::endl;
            
        }


    
    
    // std::cout << turtle_name <<std::endl;
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 1, &poseCallback);
 
    ros::spin();
    
    return 0;
};

