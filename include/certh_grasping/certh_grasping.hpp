#ifndef CERTH_GRASPING_HPP
#define CERTH_GRASPING_HPP

#include <ros/ros.h>
#include <spring_detector/springDetect.h>
#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <camera_helpers/gphoto2_capture.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cvx/util/camera/camera.hpp>

using namespace Eigen;
using namespace robot_helpers;
using namespace std;

class CerthGrasping
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient detect_spring_client;

    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    Matrix4f camera_matrix;
    double fx, fy, cx, cy, size_x, size_y;

    float resize_ratio;
    float grasp_offset;
    float gripper_height_offset;
    float gripper_extra_height;
    int patch_offset; //width or height of image patch in pixels (patch containing a spring)
    float push_offset; //how far should the brush move to clean the neighboor

public:
    CerthGrasping()
    : it(nh_)
    , resize_ratio(0.75)
    , tray_height(0.752)
    , pre_grasp_height_offset(0.05)
    , spring_radius(0.008)
    , grasp_offset(0.019)
    , gripper_height_offset(0.005)
    , patch_offset(600)
    , push_offset(0.05)
    , gripper_opening_angle(0.5)
    , gripper_extra_height(0.006)
    , fx(8460.70), fy(8452.94), cx(2464), cy(1632), size_x(4928), size_y(3264)
    {

        image_pub = it.advertise("camera/Image", 1);

        spring_list.clear();
        spring_points.clear();
        rotX.clear();
        rotY.clear();
        is_cluttered.clear();

        camera_matrix <<    0.999205,   0.0395858,  -0.0046684,     0.24137,
                           0.0395596,   -0.999202, -0.00558069,   -0.985534,
                         -0.00488559,  0.00539157,   -0.999974,     2.27777,
                                   0,           0,           0,           1;

    }

    std::vector<std::vector<cv::Point> > spring_list;
    std::vector<cv::Point> spring_points;
    std::vector<float> rotX, rotY;
    std::vector<bool> is_cluttered;

    float tray_height;
    float pre_grasp_height_offset;
    float spring_radius;
    double gripper_opening_angle;

    Vector3f cameraMatrixToEulerAngles();
    float calculateGraspingAngle(uint i);
    float angleCorrection(float angle);

    void detectSprings();
    cv::Point calculateSpringCenter(std::vector<cv::Point> &spring_points);
    Vector3f calculateWorldCoordinates(cv::Point centroid);
    Vector3f calculateGripperPosition(Vector3f spring_position, float gripper_angle, bool is_cluttered);
    bool removeDebris();
    bool getNewImage(cv::Mat &rgb, const cv::Mat &mask);
    void findImagePatch(const cv::Mat &src, cv::Point center, int offset, cv::Mat &im_patch);
    float springToCameraOrientation(uint i);
    bool pushDebris(cv::Point start_point, float push_orientation, float push_offset, int counter = 0);
    float cameraToBaseOrientation(float angle);

    cv::Mat makeMask(const cv::Mat &src, cv::Point center, int offset);
    bool graspSpring(Vector3f gripper_position, float gripper_angle);
};

#endif // CERTH_GRASPING_HPP



