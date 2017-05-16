#include "ark_llp/go2goal.h"
#include "ardrone_autonomy/Navdata.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <sstream>
#include <math.h>
#define Default 2.5
nav_msgs::Odometry obspose;
nav_msgs::Odometry quadpose;
void obsCallback(const nav_msgs::Odometry::ConstPtr& msg);
void quadCallback(const nav_msgs::Odometry::ConstPtr& msg);
float GetErrorLin(nav_msgs::Odometry obstpose,nav_msgs::Odometry quadpose);
/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    Go2Goal random;
    double x, y, z, theta;
    float func;
    x = std::atof(argv[1]);
    y = std::atof(argv[2]);
    z = std::atof(argv[3]);
    theta = std::atof(argv[4]);
    func = std::atof(argv[5]);
    //double x=1,y=2,z=3,theta=0;
    //double x1=0.1,y1=0.2,z1=3,theta1=0;
    if(func==1)
    	random.set_dest(x, y, z, theta);
    else
    	random.false_dest(x, y, z, theta);
    return 0;

}*/
int main(int argc, char **argv)
{
  ros::init(argc,argv,"quad_p2p");
  Go2Goal dest;
  double x, y, z, theta,kp;
  float ErrorLin,ErrorQuad,ErrorObs ;
  float obs_theta;
  float set_theta;
  x = std::atof(argv[1]);
  y = std::atof(argv[2]);
  z = std::atof(argv[3]);
  kp=std::atof(argv[4]);
  theta = 0;
  ros::NodeHandle n;
  ros::Subscriber quadpose_sub = n.subscribe("/ground_truth/state", 100, quadCallback); // subscriber to get MAV position
  ros::Subscriber obspose_sub = n.subscribe("/robot3/odom", 100, obsCallback); // subscriber to get ground bot position
  ros::Rate loop_rate(10);



  while(ros::ok())
  {
    obs_theta= atan2(((obspose.pose.pose.position.x)-(quadpose.pose.pose.position.x)),((obspose.pose.pose.position.y)*(-1)- (quadpose.pose.pose.position.y)*(-1)));
    set_theta= atan2((y-(quadpose.pose.pose.position.x)),(x-(quadpose.pose.pose.position.y)*(-1)));

    ErrorLin = GetErrorLin(obspose,quadpose);
    ErrorQuad = sqrt(pow((y - quadpose.pose.pose.position.x),2) + pow((x - (quadpose.pose.pose.position.y)*(-1)),2));
    ErrorObs = sqrt(pow((y - obspose.pose.pose.position.x),2) + pow((x - (obspose.pose.pose.position.y)*(-1)),2));
    if(ErrorLin < 2 && (ErrorObs<ErrorQuad))
    {
        z=kp*ErrorLin;
        if((set_theta) <= (obs_theta))
         {
          set_theta-=z;
         }
        else if((set_theta) > (obs_theta))
         {
          set_theta+=z;
         }
         dest.false_dest(((quadpose.pose.pose.position.y)*(-1)+2*cos(set_theta)),(quadpose.pose.pose.position.x+2*sin(set_theta)),Default,0);
    }
    else
    {
     dest.set_dest(x,y,z,theta);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void obsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  obspose.pose.pose.position.x = msg->pose.pose.position.x;
  obspose.pose.pose.position.y = msg->pose.pose.position.y;
  obspose.pose.pose.position.z = msg->pose.pose.position.z;
  return;
}
void quadCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  quadpose.pose.pose.position.x = msg->pose.pose.position.x;
  quadpose.pose.pose.position.y = msg->pose.pose.position.y;
  quadpose.pose.pose.position.z = msg->pose.pose.position.z;
  return;
}

float GetErrorLin(nav_msgs::Odometry obstpose,nav_msgs::Odometry quadpose)
{
  float El;
  El = sqrt(pow((quadpose.pose.pose.position.x - obspose.pose.pose.position.x),2) + pow((quadpose.pose.pose.position.y - obspose.pose.pose.position.y),2));
  return(El);
}
