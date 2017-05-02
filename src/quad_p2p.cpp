#include "ark_llp/go2goal.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    Go2Goal random;
    float x, y, z, theta, func;
    x = std::atof(argv[1]);
    y = std::atof(argv[2]);
    z = std::atof(argv[3]);
    theta = std::atof(argv[4]);
    func = std::atof(argv[5]);
    if(func==1)
    	random.set_dest(x, y, z, theta);
    else
    	random.add_dest(x, y, z, theta);

    return 0;
}
