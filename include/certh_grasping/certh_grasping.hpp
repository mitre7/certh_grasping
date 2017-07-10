#ifndef CERTH_GRASPING_HPP
#define CERTH_GRASPING_HPP

#include <ros/ros.h>
#include <spring_detector/springDetect.h>

#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cvx/util/camera/camera.hpp>

using namespace Eigen;

class CerthGrasping
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient detect_spring_client;

    Matrix4f camera_matrix;

    float resize_ratio;
    float grasp_offset;

public:
    CerthGrasping()
    : resize_ratio(0.75)
    , tray_height(0.53)
    , height_offset(0.01)
    , grasp_offset(0.012)
    {

        spring_list.clear();
        spring_points.clear();
        rotY.clear();
        rotZ.clear();

        camera_matrix <<  -0.0873767,    0.996173,  -0.0022065,    0.102225,
                            0.996172,   0.0873817,  0.00228325,    -1.19153,
                          0.00246732, -0.00199855,   -0.999995,     2.26495,
                                   0,           0,           0,           1;

    }

    std::vector<std::vector<cv::Point> > spring_list;
    std::vector<cv::Point> spring_points;
    std::vector<float> rotY, rotZ;

    float tray_height;
    float height_offset;

    void detectSprings();
    cv::Point calculateSpringCenter(std::vector<cv::Point> &spring_points);
    Vector3f calculateWorldCoordinates(cv::Point centroid);
    void calculateGripperPosition(Vector3f &gripper_position, Vector3f spring_position, float gripper_angle);
};

#endif // CERTH_GRASPING_HPP
