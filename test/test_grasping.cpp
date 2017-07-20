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

    ros::Duration(1).sleep();

    arm.moveHome() ;

    ros::Duration(1).sleep();

    CerthGrasping cg;
    cg.detectSprings();

    Vector3f drop_position = cg.calculateWorldCoordinates(cv::Point(4720, 1390));
    drop_position.z() += 0.10;
    cout << "Drop position: " << drop_position << endl;

    cout << "Springs at: " << endl;
    for (uint j=0; j<cg.spring_list.size(); j++)
    {
        cv::Point centroid = cg.calculateSpringCenter(cg.spring_list[j]);
        cout << "x = " << centroid.x << ", y = " << centroid.y << ", rotX = " << cg.rotX[j] << ", rotY = " << cg.rotY[j] <<endl;
    }

    for (uint i=0; i<cg.spring_list.size(); i++)
    {
        cv::Point centroid = cg.calculateSpringCenter(cg.spring_list[i]);
        Vector3f spring_position_world = cg.calculateWorldCoordinates(centroid);
        cout << endl << "World Coordinates: " << endl << spring_position_world << endl;

        float gripper_angle_1 = -cg.rotY[i];
        cout << "Gripper angle without tranformations: " << gripper_angle_1 * 180 / M_PI << endl;

        float gripper_angle_2 = cg.calculateGraspingAngle(i);
        cout << "Gripper angle with tranformations: " << gripper_angle_2 * 180 / M_PI << endl;
        gripper_angle_2 = cg.angleCorrection(gripper_angle_2);
        cout << "Gripper angle with tranformations and correction: " << gripper_angle_2 * 180 / M_PI << endl;

        Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), gripper_angle_2);

        Vector3f gripper_position;
        gripper_position = cg.calculateGripperPosition(spring_position_world, gripper_angle_2);

        if (!arm.closeGripper()) continue;

        RobotArm::Plan plan ;
        if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + cg.height_offset), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + cg.height_offset).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {

                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        ros::Duration(1).sleep();

//        float dx = gripper_position(0) - spring_position_world(0);
//        float dy = gripper_position(1) - spring_position_world(1);
//        float dz = gripper_position(2) - spring_position_world(2);

//        cout << "dx = " << dx << ", dy = " << dy << ", dz = " << dz << endl;

        cout << "Move robot down?" << endl;
        cin.get();

        if (!arm.openGripper()) continue;

        ros::Duration(1).sleep();

        if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2)), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2)).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        if (!arm.closeGripper()) continue;

        cout << "Move robot up?" << endl;
        cin.get();

        if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + cg.height_offset), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + cg.height_offset).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        ros::Duration(1).sleep();

        if ( !arm.planTipIK(Eigen::Vector3d(drop_position(0), drop_position(1), drop_position(2)), q, plan) ) {
            cerr << "can't plan to location:" << Vector3d(drop_position(0), drop_position(1), drop_position(2)).adjoint() << endl ;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }

        ros::Duration(1).sleep();

        if (!arm.openGripper()) continue;

        ros::Duration(1).sleep();

    }

    arm.setServoPowerOff();

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}


