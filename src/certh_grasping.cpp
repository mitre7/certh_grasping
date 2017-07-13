#include <certh_grasping/certh_grasping.hpp>

#include <ros/node_handle.h>

#include <spring_detector/springDetect.h>
#include "spring_detector/hull.h"
#include "spring_detector/hullArray.h"
#include "spring_detector/point.h"


void CerthGrasping::detectSprings()
{
    ros::NodeHandle n;
    detect_spring_client = n.serviceClient<spring_detector::springDetect>("detect_spring");

    spring_detector::springDetect srv;

    if (detect_spring_client.call(srv))
    {
        for (uint m=0; m<srv.response.spring_msg.springs.size(); m++)
        {
            for (uint n=0; n<srv.response.spring_msg.springs[m].points.size(); n++)
            {
                cv::Point pt;
                pt.x = srv.response.spring_msg.springs[m].points[n].x / resize_ratio;
                pt.y = srv.response.spring_msg.springs[m].points[n].y / resize_ratio;
                spring_points.push_back(pt);
            }
            rotX.push_back(srv.response.spring_msg.springs[m].phi);
            rotY.push_back(srv.response.spring_msg.springs[m].theta);
            spring_list.push_back(spring_points);
            spring_points.clear();
        }
    }
}

cv::Point CerthGrasping::calculateSpringCenter(std::vector<cv::Point> &spring_points)
{
    cv::Point centroid;
    int temp_x = 0;
    int temp_y = 0;
    for (uint i=0; i<spring_points.size(); i++)
    {
        temp_x += spring_points[i].x;
        temp_y += spring_points[i].y;
    }
    centroid.x = temp_x / spring_points.size();
    centroid.y = temp_y / spring_points.size();
    return centroid;
}

Vector3f CerthGrasping::calculateWorldCoordinates(cv::Point centroid)
{
    cvx::util::PinholeCamera cam(fx, fy, cx, cy, cv::Size(size_x, size_y));
    Vector3f object_position_temp = cam.backProject(centroid.x, centroid.y, camera_matrix(2,3) - tray_height);
    Vector4f object_position_camera(object_position_temp.x(), object_position_temp.y(), object_position_temp.z(), 1);
    Vector4f object_position_world = camera_matrix * object_position_camera;

    return Vector3f(object_position_world(0), object_position_world(1), object_position_world(2));
}

Vector3f CerthGrasping::calculateGripperPosition(Vector3f spring_position, float gripper_angle)
{
    Vector3f gripper_position;
    gripper_position.x() = spring_position.x() + grasp_offset * cos(gripper_angle);
    gripper_position.y() = spring_position.y() - grasp_offset * sin(gripper_angle);
    gripper_position.z() = spring_position.z() + spring_radius;
    return gripper_position;
}

Vector3f CerthGrasping::matrixToEulerAngles()
{
    Matrix3f temp;

    temp << camera_matrix(0, 0), camera_matrix(0, 1), camera_matrix(0, 2),
            camera_matrix(1, 0), camera_matrix(1, 1), camera_matrix(1, 2),
            camera_matrix(2, 0), camera_matrix(2, 1), camera_matrix(2, 2);

    Vector3f ea = temp.eulerAngles(2, 1, 0);
    std::cout << ea << std::endl;
    return ea;
}

float CerthGrasping::calculateGraspingAngle(uint i)
{
    Matrix3f spring_to_camera = (AngleAxisf(-rotX[i], Eigen::Vector3f::UnitX())
                               * AngleAxisf(rotY[i], Eigen::Vector3f::UnitY())
                               * AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY())).matrix();

    Matrix3f camera_to_base = camera_matrix.block(0, 0, 2, 2);

    Matrix3f spring_to_base = (spring_to_camera * camera_to_base);

    Vector3f ea = spring_to_base.eulerAngles(2, 0, 1);   //ZXY rotation
    std::cout << ea * 180 / M_PI << std::endl;
    return ea[0] + M_PI/2;   //gripper perpendicular to the spring direction
}

float CerthGrasping::angleCorrection(float angle)
{
    if (angle > M_PI/2 && angle < 3*M_PI/2)
        return angle - M_PI;
    else if (angle < -M_PI/2 && angle > -3*M_PI/2)
        return angle + M_PI;
    else
        return angle;
}



