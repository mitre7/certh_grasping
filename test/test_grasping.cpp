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

//    arm.moveTipIK(Eigen::Vector3d(0.5, -1, 0.7), robot_helpers::lookAt(Vector3d(0, 0, -1)));
//    ros::waitForShutdown();

    arm.moveHome() ;

    arm.closeGripper();

    ros::Duration(2).sleep();

    CerthGrasping cg;
    cg.matrixToEulerAngles();
    cg.detectSprings();

    cout << "Springs at: " << endl;
    for (uint j=0; j<cg.spring_list.size(); j++)
    {
        cv::Point centroid = cg.calculateSpringCenter(cg.spring_list[j]);
        cout << "x = " << centroid.x << ", y = " << centroid.y << ", rotY = " << cg.rotX[j] << ", rotZ = " << cg.rotY[j] <<endl;
    }

    for (uint i=0; i<cg.spring_list.size(); i++)
    {
        cv::Point centroid = cg.calculateSpringCenter(cg.spring_list[i]);
        Vector3f spring_position_world = cg.calculateWorldCoordinates(centroid);
        cout << endl << "World Coordinates: " << endl << spring_position_world << endl;

        float gripper_angle = -cg.rotY[i] + M_PI/2;
        cout << "Gripper angle without tranformations: " << gripper_angle << endl;

        gripper_angle = cg.calculateGraspingAngle(i);
        cout << "Gripper angle with tranformations: " << gripper_angle << endl;

        Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), gripper_angle);

        Vector3f gripper_position;
        cg.calculateGripperPosition(gripper_position, spring_position_world, gripper_angle);

        RobotArm::Plan plan ;
        if ( !arm.planTipIK(Eigen::Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + cg.height_offset).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {

                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        ros::Duration(1).sleep();

        cout << "Move robot down?" << endl;
        cin.get();

        if (!arm.openGripper()) continue;

        ros::Duration(1).sleep();

        if ( !arm.planTipIK(Eigen::Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.spring_radius), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.spring_radius).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        cout << "Move robot up?" << endl;
        cin.get();

        if ( !arm.planTipIK(Eigen::Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(spring_position_world(0), spring_position_world(1), spring_position_world(2) + cg.height_offset).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

    }

    arm.setServoPowerOff();

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}
