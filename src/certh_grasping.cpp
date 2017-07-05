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

    spring_list.clear();
    spring_points.clear();
    rotY.clear();
    rotZ.clear();

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
            rotY.push_back(srv.response.spring_msg.springs[m].phi);
            rotZ.push_back(srv.response.spring_msg.springs[m].theta);
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
    cvx::util::PinholeCamera cam(9908.44, 9887.03, 2464, 1632, cv::Size(4928, 3264));
    Vector3f object_position_temp = cam.backProject(centroid.x, centroid.y, camera_matrix(2,3) - tray_height);
    Vector4f object_position_camera(object_position_temp.x(), object_position_temp.y(), object_position_temp.z(), 1);
    Vector4f object_position_world = camera_matrix * object_position_camera;

    return Vector3f(object_position_world(0), object_position_world(1), object_position_world(2));
}




