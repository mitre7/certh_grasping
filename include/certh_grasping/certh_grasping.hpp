#ifndef CERTH_GRASPING_HPP
#define CERTH_GRASPING_HPP

#include <ros/ros.h>
#include <spring_detector/springDetect.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
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
    double fx, fy, cx, cy, size_x, size_y;

    float resize_ratio;
    float grasp_offset;

public:
    CerthGrasping()
    : resize_ratio(0.75)
    , tray_height(0.52)
    , height_offset(0.05)
    , spring_radius(0.015)
    , grasp_offset(0.012)
    , fx(10179.87), fy(10154.06), cx(2464), cy(1632), size_x(4928), size_y(3264)
    {

        spring_list.clear();
        spring_points.clear();
        rotX.clear();
        rotY.clear();

        camera_matrix <<  -0.0873767,    0.996173,  -0.0022065,    0.102225,
                            0.996172,   0.0873817,  0.00228325,    -1.19153,
                          0.00246732, -0.00199855,   -0.999995,     2.26495,
                                   0,           0,           0,           1;

    }

    std::vector<std::vector<cv::Point> > spring_list;
    std::vector<cv::Point> spring_points;
    std::vector<float> rotX, rotY;

    float tray_height;
    float height_offset;
    float spring_radius;

    Vector3f matrixToEulerAngles();
    float calculateGraspingAngle(uint i);

    void detectSprings();
    cv::Point calculateSpringCenter(std::vector<cv::Point> &spring_points);
    Vector3f calculateWorldCoordinates(cv::Point centroid);
    void calculateGripperPosition(Vector3f &gripper_position, Vector3f spring_position, float gripper_angle);
};

#endif // CERTH_GRASPING_HPP
