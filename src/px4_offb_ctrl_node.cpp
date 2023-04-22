#include "px4_offb_ctrl/px4_offb_ctrl.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_offb_node");

    ros::NodeHandle nh("~");

    OffboardNode offb_node(nh);

    ros::spin();

    return 0;
}