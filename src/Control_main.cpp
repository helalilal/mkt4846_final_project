#include <ros/ros.h>
#include "Control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle node_handle("~");

    pid_controller::Control Control(node_handle);

    ros::spin();
    return 0;
}