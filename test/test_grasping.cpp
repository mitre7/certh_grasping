#include "ros/ros.h"

#include <certh_grasping/certh_grasping.hpp>
#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>

using namespace std;
using namespace robot_helpers;
using namespace Eigen;

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "test_grasping");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    string arm_name = "r2" ;
    RobotArm arm(arm_name) ;
    arm.moveHome() ;

    CerthGrasping cg;

    for (uint i=0; i</*cg.spring_list.size()*/ 1; i++)
    {
        cg.detectSprings();
        cv::Point centroid = cg.calculateSpringCenter(cg.spring_list[i]);
        cout << "x = " << centroid.x << ", y = " << centroid.y << endl;
        Vector3f spring_position_world = cg.calculateWorldCoordinates(centroid);
        cout << "World Coordinates: " << spring_position_world << endl;

        Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), 0);

        RobotArm::Plan plan ;
        if ( !arm.planTipIK(Eigen::Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "moving to: " << Eigen::Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset).adjoint() << endl  ;
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        if (!arm.openGripper()) continue;

        ros::Duration(0.5).sleep();



    }

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}
