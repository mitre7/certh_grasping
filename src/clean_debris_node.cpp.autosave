#include "ros/ros.h"
#include <certh_grasping/certh_grasping.hpp>
#include <push_debris/PushDebris.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "clean_debris");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    CerthGrasping cg;
    if (!cg.removeDebris())
        cout << "Error when trying to remove debris" << endl;

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}
