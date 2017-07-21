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
    float gripper_height_offset;

public:
    CerthGrasping()
    : resize_ratio(0.75)
    , tray_height(0.752)
    , height_offset(0.05)
    , spring_radius(0.008)
    , grasp_offset(0.019)
    , gripper_height_offset(0.005)
    , fx(8460.70), fy(8452.94), cx(2464), cy(1632), size_x(4928), size_y(3264)
    {

        spring_list.clear();
        spring_points.clear();
        rotX.clear();
        rotY.clear();

        camera_matrix <<    0.999205,   0.0395858,  -0.0046684,     0.24137,
                           0.0395596,   -0.999202, -0.00558069,   -0.985534,
                         -0.00488559,  0.00539157,   -0.999974,     2.27777,
                                   0,           0,           0,           1;

    }

    std::vector<std::vector<cv::Point> > spring_list;
    std::vector<cv::Point> spring_points;
    std::vector<float> rotX, rotY;

    float tray_height;
    float height_offset;
    float spring_radius;

    Vector3f cameraMatrixToEulerAngles();
    float calculateGraspingAngle(uint i);
    float angleCorrection(float angle);

    void detectSprings();
    cv::Point calculateSpringCenter(std::vector<cv::Point> &spring_points);
    Vector3f calculateWorldCoordinates(cv::Point centroid);
    Vector3f calculateGripperPosition(Vector3f spring_position, float gripper_angle);
};

#endif // CERTH_GRASPING_HPP



