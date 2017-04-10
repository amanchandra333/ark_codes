#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    ros::Publisher go2goal = n.advertise<std_msgs::String>("/tum_ardrone/com", 10);
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    return 0;
}
