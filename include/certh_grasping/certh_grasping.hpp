#ifndef CERTH_GRASPING_HPP
#define CERTH_GRASPING_HPP

#include <ros/ros.h>
#include <spring_detector/springDetect.h>

#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>
#include <cvx/util/camera/camera.hpp>

using namespace Eigen;

class CerthGrasping
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient detect_spring_client;

    Matrix4f camera_matrix;

    float resize_ratio;

public:
    CerthGrasping()
    : resize_ratio(0.75)
    , tray_height(0.53)
    , height_offset(0.00)
    {
        camera_matrix << -0.0912497 ,    0.995828 , -0.000231745 ,   0.0991083,
                           0.995505 ,   0.0912141 ,   -0.0254834 ,    -1.15709,
                          -0.025356 , -0.00255606 ,    -0.999675 ,      2.2305,
                                  0 ,          0  ,           0  ,           1;
    }

    std::vector<std::vector<cv::Point> > spring_list;
    std::vector<cv::Point> spring_points;
    std::vector<float> rotY, rotZ;

    float tray_height;
    float height_offset;

    void detectSprings();
    cv::Point calculateSpringCenter(std::vector<cv::Point> &spring_points);
    Vector3f calculateWorldCoordinates(cv::Point centroid);
};

#endif // CERTH_GRASPING_HPP

