#ifndef CERTH_GRASPING_HPP
#define CERTH_GRASPING_HPP

#include <ros/ros.h>
#include <spring_detector/springDetect.h>
#include <push_debris/PushDebris.h>

#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
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
    ros::ServiceClient push_debris_client;

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

    uint push_start_point_x;
    uint push_start_point_y;
    uint push_final_point_x;
    uint push_final_point_y;
    float push_estimated_orientation;
    bool is_cleared;

    std::string camera_extrinsics, camera_intrinsics;
    cvx::util::PinholeCamera cam ;

public:
    CerthGrasping()
    : it(nh_)
    , push_start_point_x(0), push_start_point_y(0), push_final_point_x(0), push_final_point_y(0), push_estimated_orientation(0)
    , resize_ratio(0.75)
    , tray_height(0.752)
    , pre_grasp_height_offset(0.05)
    , spring_radius(0.008)
    , grasp_offset(0.019)
    , gripper_height_offset(0.005)
    , patch_offset(600)
    , push_offset(0.05)
    , gripper_opening_angle(0.3)
    , gripper_extra_height(0.006)
    //, fx(8460.70), fy(8452.94), cx(2464), cy(1632), size_x(4928), size_y(3264)
    , size_x(4928), size_y(3264)
    {

        image_pub = it.advertise("camera/Image", 1);

        spring_list.clear();
        spring_points.clear();
        rotX.clear();
        rotY.clear();
        is_cluttered.clear();
/*
        camera_matrix <<    0.999205,   0.0395858,  -0.0046684,     0.24137,
                           0.0395596,   -0.999202, -0.00558069,   -0.985534,
                         -0.00488559,  0.00539157,   -0.999974,     2.27777,
                                   0,           0,           0,           1;

*/	
	
	//camera_intrinsics = getenv("HOME") + std::string("/.ros/calibration_data_intrinsics/camera.xml");
	camera_intrinsics = getenv("HOME") + std::string("/roso_review/calib_camera_intrinsics/camera.xml");
	if  ( !cam.read(camera_intrinsics) ) {
           cerr << "can't read intrinsics from: " << camera_intrinsics << endl ;
           exit(1) ;
    	}
	else
	{
	   fx = cam.fx();
	   fy = cam.fy();
	   cx = cam.cx();
	   cy = cam.cy();
	}

	//camera_extrinsics = getenv("HOME") + std::string("/.ros/calibration_data/pose.txt");
	camera_extrinsics = getenv("HOME") + std::string("/roso_review/calib_camera_extrinsics/pose.txt");	
        std::ifstream strm(camera_extrinsics.c_str());
        for(int i=0;i<4;i++)
 	   for(int j=0;j<4;j++)
	      strm >> camera_matrix(i,j);

	std::cout << "camera matrix:" << camera_matrix << std::endl;
	std::cout << fx << " " << fy << " " << cx << " " << cy << std::endl;

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
    bool pushDebris(float gripper_angle);
    bool pushDebris(cv::Point start_point, float push_orientation, float push_offset, int counter = 0); //for testing
    float cameraToBaseOrientation(float angle);

    cv::Mat makeMask(const cv::Mat &src, cv::Point center, int offset);
    bool graspSpring(Vector3f gripper_position, float gripper_angle);

    void push_debris_service_call(int x, int y, float spring_orientation, const cv::Mat &tray_image);

    float findPushDirectionInCameraFrame();
};

#endif // CERTH_GRASPING_HPP



