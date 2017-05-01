#include "ark_llp/go2goal.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_p2p");
    Go2Goal random;
    random.set_dest();

    return 0;
}
