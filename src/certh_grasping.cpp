#include <certh_grasping/certh_grasping.hpp>

#include <ros/node_handle.h>
#include <Eigen/Core>

#include <spring_detector/springDetect.h>
#include "spring_detector/hull.h"
#include "spring_detector/hullArray.h"
#include "spring_detector/point.h"

#include <ctime>
#include <chrono>  // for high_resolution_clock


void CerthGrasping::detectSprings()
{
    ros::NodeHandle n;
    detect_spring_client = n.serviceClient<spring_detector::springDetect>("detect_spring");

    spring_list.clear();
    spring_points.clear();
    rotX.clear();
    rotY.clear();
    is_cluttered.clear();

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
            is_cluttered.push_back(srv.response.spring_msg.springs[m].is_cluttered);
            spring_list.push_back(spring_points);
            spring_points.clear();
        }
    }
}

void CerthGrasping::push_debris_service_call(int x, int y, float spring_orientation, const cv::Mat &tray_image)
{
    ros::NodeHandle n;
    push_debris_client = n.serviceClient<push_debris::PushDebris>("push_debris");

    push_debris::PushDebris srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.orientation = spring_orientation ;

    cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", tray_image);
    sensor_msgs::Image image;
    cv_img.toImageMsg(image);
    srv.request.img = image;

    cout << "Calling service..." << endl;

    if (push_debris_client.call(srv))
    {
        push_start_point_x = srv.response.x1;
        push_start_point_y = srv.response.y1;
        push_final_point_x = srv.response.x2;
        push_final_point_y = srv.response.y2;
        push_estimated_orientation = srv.response.push_orientation;
        is_cleared = srv.response.is_cleared;
        std::cout << "is cleared" << is_cleared << std::endl;
    }

}

float CerthGrasping::findPushDirectionInCameraFrame()
{
    float push_direction = atan2((push_final_point_y - push_start_point_y), (push_final_point_x - push_start_point_x));

    return push_direction + M_PI / 2;
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
    Vector3f object_position_temp = cam.backProject(centroid.x, centroid.y, camera_matrix(2,3) - tray_height - spring_radius);
    Vector4f object_position_camera(object_position_temp.x(), object_position_temp.y(), object_position_temp.z(), 1);
    Vector4f object_position_world = camera_matrix * object_position_camera;

    return Vector3f(object_position_world(0), object_position_world(1), tray_height + spring_radius);
}

Vector3f CerthGrasping::calculateGripperPosition(Vector3f spring_position, float gripper_angle, bool is_cluttered)
{
    Vector3f gripper_position;
    gripper_position.x() = spring_position.x() + grasp_offset * cos(gripper_angle);
    gripper_position.y() = spring_position.y() - grasp_offset * sin(gripper_angle);
    if (is_cluttered)
        gripper_position.z() = spring_position.z() + gripper_height_offset + gripper_extra_height;
    else
        gripper_position.z() = spring_position.z() + gripper_height_offset;

    return gripper_position;
}

Vector3f CerthGrasping::cameraMatrixToEulerAngles()
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

    Matrix3f camera_to_base; /*= camera_matrix.block(0, 0, 2, 2);*/
    camera_to_base << camera_matrix(0, 0), camera_matrix(0, 1), camera_matrix(0, 2),
                      camera_matrix(1, 0), camera_matrix(1, 1), camera_matrix(1, 2),
                      camera_matrix(2, 0), camera_matrix(2, 1), camera_matrix(2, 2);

    Matrix3f spring_to_base = (spring_to_camera * camera_to_base);

    Vector3f ea = spring_to_base.eulerAngles(2, 1, 0);   //ZYX rotation
//    std::cout << ea * 180 / M_PI << std::endl;
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




//functions related to the debris removal
bool CerthGrasping::getNewImage(cv::Mat &rgb, const cv::Mat &mask)
{
//    rgb = cv::imread("/home/echord/Pictures/test00027.png");
    if ( camera_helpers::gphoto2::capture(rgb, "nikon_overhead") )
    {
        cv::imwrite("/tmp/cap.png", rgb) ;
    }
    else
    {
      std::cerr << "Could not query photo/capture service" << std::endl ;
      return false;
    }

    cv::Mat rgb_message;
    rgb.copyTo(rgb_message, mask);

    cv_bridge::CvImage cv_img(std_msgs::Header(), "bgr8", rgb_message);
    sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
    image_pub.publish(msg);

    cout << "Image Published" << endl;

    return true;
}

void CerthGrasping::findImagePatch(const cv::Mat &src, cv::Point center, int offset, cv::Mat &im_patch)
{
    int x_min, y_min, width, height;
    x_min = center.x - offset/2;
    y_min = center.y - offset/2;
    width = offset;
    height = offset;

    if ( x_min < 0)
        x_min = 0;
    if ( y_min < 0)
        y_min = 0;
    if ((x_min + width) > (src.cols - 1))
        width = src.cols - 1 - x_min;
    if ((y_min + height) > (src.rows -1))
        height = src.rows -1 - y_min;

    im_patch = src(cv::Rect(x_min, y_min, width, height));
}

cv::Mat CerthGrasping::makeMask(const cv::Mat &src, cv::Point center, int offset)
{
    int x_min, y_min, width, height;
    x_min = center.x - offset/2;
    y_min = center.y - offset/2;
    width = offset;
    height = offset;

    if ( x_min < 0)
        x_min = 0;
    if ( y_min < 0)
        y_min = 0;
    if ((x_min + width) > (src.cols - 1))
        width = src.cols - 1 - x_min;
    if ((y_min + height) > (src.rows -1))
        height = src.rows -1 - y_min;

    cv::Rect roi(x_min, y_min, width, height);

    cv::Mat mask(src.size(), CV_8UC1, cv::Scalar::all(0));
    mask(roi).setTo(cv::Scalar::all(255));

    return mask;
}

float CerthGrasping::springToCameraOrientation(uint i)
{
    Matrix3f spring_to_camera = (AngleAxisf(-rotX[i], Eigen::Vector3f::UnitX())
                               * AngleAxisf(rotY[i], Eigen::Vector3f::UnitY())
                               * AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY())).matrix();

    Vector3f ea = spring_to_camera.eulerAngles(2, 1, 0);   //ZYX rotation

    cout << "rotZ = " << ea[0] * 180 / M_PI << endl;
//    cout << "roty = " << ea[1] * 180 / M_PI << endl;
//    cout << "rotX = " << ea[2] * 180 / M_PI << endl;
    return ea[0];
}

float CerthGrasping::cameraToBaseOrientation(float angle)
{
    Matrix3f object_to_camera = (AngleAxisf(angle, -Eigen::Vector3f::UnitZ()) ).matrix();

    Matrix4d camera_to_base = (robot_helpers::getTransform("base_link", "nikon")).matrix();
    Matrix4f cam_to_base_f = camera_to_base.cast <float> ();
    Matrix3f cam_to_base;
    cam_to_base<< cam_to_base_f(0,0), cam_to_base_f(0,1), cam_to_base_f(0,2),
                  cam_to_base_f(1,0), cam_to_base_f(1,1), cam_to_base_f(1,2),
                  cam_to_base_f(2,0), cam_to_base_f(2,1), cam_to_base_f(2,2);

    Matrix3f object_to_base = (object_to_camera * cam_to_base);

    Vector3f ea = object_to_base.eulerAngles(2, 1, 0);   // rotation
    std::cout << "object to base angle = " << ea[0] * 180 / M_PI << std::endl;
    return ea[0];   //gripper perpendicular to the object direction
}

bool CerthGrasping::pushDebris(float gripper_angle)
{
    std::cout << "Start" << std::endl;
    string arm_name = "r1";
    RobotArm arm(arm_name);
    //if (!arm.closeGripper()) return false;

    arm.setRobotSpeed(0.8);

    //TODO: move r2_arm to the position that r1_arm can move above the tray without collisions
    std::cout << "begin pushing" << std::endl;

    double z_offset = 0.0235;
    //double y_offset = 0.0135;
    //double x_offset = 0.013;
    Vector3f initial_point = calculateWorldCoordinates(cv::Point(push_start_point_x,push_start_point_y));
    initial_point.z() += z_offset;
    //initial_point.y() += y_offset;
    //initial_point.x() += x_offset;
    cout << "start point:(" << initial_point.x() << ", " << initial_point.y() << ", " << initial_point.z() << ")" << endl;

    Vector3f final_point = calculateWorldCoordinates(cv::Point(push_final_point_x,push_final_point_y));
    final_point.z() += z_offset;
    //final_point.y() += y_offset;
    //final_point.x() += x_offset;

    cout << "final point:(" << final_point.x() << ", " << final_point.y() << ", " << final_point.z() << ")" << endl;


    cout << "Look at input = " << (M_PI/2 - gripper_angle) * 180 / M_PI << endl;
    Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), /*M_PI/2 -*/ -gripper_angle+M_PI);
    cout << pre_grasp_height_offset << std::endl;

    //move to pre-push position
    RobotArm::Plan plan ;
    if ( !arm.planTipIK(Vector3d(initial_point(0), initial_point(1), initial_point(2) + pre_grasp_height_offset), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(initial_point(0), initial_point(1), initial_point(2) + pre_grasp_height_offset).adjoint() << endl ;
        arm.moveHome();
        return false;
    }
    else
    {
        if ( arm.execute(plan) )
        {
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
        }
    }

    arm.setRobotSpeed(0.4);
    ros::Duration(1).sleep();

    //move to initial position
    if ( !arm.planTipIK(Vector3d(initial_point(0), initial_point(1), initial_point(2)), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(initial_point(0), initial_point(1), initial_point(2)).adjoint() << endl;
        arm.moveHome();
        return false;
    }
    else
    {
        if ( arm.execute(plan) )
        {
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
        }
    }

    ros::Duration(1).sleep();

    //push the debris away
    if ( !arm.planTipIK(Vector3d(final_point(0), final_point(1), final_point(2)), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(final_point(0), final_point(1), final_point(2)).adjoint() << endl;
        arm.moveHome();
        return false;
    }
    else
    {
//        ros::Duration(1).sleep();
        if ( arm.execute(plan) )
        {
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
        }
    }

    arm.setRobotSpeed(0.8);

    while(!arm.moveHome())
    {
        ros::Duration(1).sleep();
        cerr << "R1 cannot move to home position" << endl;
    }

    return true;
}

bool CerthGrasping::pushDebris(cv::Point start_point, float gripper_angle, float push_offset, int counter)
{
    string arm_name = "r1";
    RobotArm arm(arm_name);
    if (!arm.closeGripper()) return false;

    arm.setRobotSpeed(0.8);

    //TODO: move r2_arm to the position that r1_arm can move above the tray without collisions

    Vector3f initial_point = calculateWorldCoordinates(start_point);
    initial_point.z() += 0.01;
    cout << "start point:(" << initial_point.x() << ", " << initial_point.y() << ", " << initial_point.z() << ")" << endl;

    float gripper_angle_corrected = gripper_angle - (M_PI / 2);

    Vector3f final_point;
    if (counter == 1)
    {
        final_point.x() = initial_point.x() - push_offset * cos(gripper_angle_corrected);
        final_point.y() = initial_point.y() - push_offset * sin(gripper_angle_corrected);
        final_point.z() = initial_point.z(); //TODO: Probably I should increase the height by a constant offset
    }
    else //for testing
    {
        final_point.x() = initial_point.x() + push_offset * cos(gripper_angle_corrected);
        final_point.y() = initial_point.y() + push_offset * sin(gripper_angle_corrected);
        final_point.z() = initial_point.z();
    }

    Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), M_PI/2 - gripper_angle);

    //move to pre-push position
    RobotArm::Plan plan ;
    if ( !arm.planTipIK(Vector3d(initial_point(0), initial_point(1), initial_point(2) + pre_grasp_height_offset), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(initial_point(0), initial_point(1), initial_point(2) + pre_grasp_height_offset).adjoint() << endl ;
        arm.moveHome();
        return false;
    }
    else
    {   auto start = std::chrono::high_resolution_clock::now();
        if ( arm.execute(plan) )
        {
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
        }

    }

    arm.setRobotSpeed(0.4);
    ros::Duration(1).sleep();

    //move to initial position
    if ( !arm.planTipIK(Vector3d(initial_point(0), initial_point(1), initial_point(2)), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(initial_point(0), initial_point(1), initial_point(2)).adjoint() << endl;
        arm.moveHome();
        return false;
    }
    else
    {
        if ( arm.execute(plan) )
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
    }

    ros::Duration(1).sleep();

    //push the debris away
    if ( !arm.planTipIK(Vector3d(final_point(0), final_point(1), final_point(2)), q, plan) )
    {
        cerr << "can't plan to location:" << Vector3d(final_point(0), final_point(1), final_point(2)).adjoint() << endl;
        arm.moveHome();
        return false;
    }
    else
    {
//        ros::Duration(1).sleep();
        if ( arm.execute(plan) )
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
    }

    arm.setRobotSpeed(0.8);

    while(!arm.moveHome())
    {
        ros::Duration(1).sleep();
        cerr << "R1 cannot move to home position" << endl;
    }
    return true;
}

bool CerthGrasping::graspSpring(Vector3f gripper_position, float gripper_angle)
{
    string arm_name = "r2" ;
    RobotArm arm(arm_name) ;
    RobotGripper gripper(arm_name);

    Quaterniond q = robot_helpers::lookAt(Vector3d(0, 0, -1), gripper_angle);

    RobotArm::Plan plan ;
    if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + pre_grasp_height_offset), q, plan) ) {
        cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + pre_grasp_height_offset).adjoint() << endl ;
        return false;
    }
    else
    {
        auto start = std::chrono::high_resolution_clock::now();
        if ( arm.execute(plan) )
        {
            cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
        }
    }

    ros::Duration(1).sleep();

    if (!gripper.changeGripperState(gripper_opening_angle)) return false;

    cout << "Move robot down?" << endl;
    cin.get();

    ros::Duration(1).sleep();

    auto start = std::chrono::high_resolution_clock::now();
    if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + gripper_height_offset), q, plan) ) {
        cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + gripper_height_offset).adjoint() << endl ;
        return false;
    }
    else {
        if ( arm.execute(plan) )
        {
           cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl;
        }
    }

    if (!arm.closeGripper()) return false;
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time for robot move: " << elapsed.count() << " s\n";

    cout << "Move robot up?" << endl;
    cin.get();

    if ( !arm.planTipIK(Eigen::Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + pre_grasp_height_offset), q, plan) ) {
        cerr << "can't plan to location:" << Vector3d(gripper_position(0), gripper_position(1), gripper_position(2) + pre_grasp_height_offset).adjoint() << endl ;
    }
    else {
        while (!arm.execute(plan) ) {
            cout << "Waiting to remove debris..." <<endl  ;
        }
    }

    ros::Duration(1).sleep();

    arm.moveHome();

    return true;
}

bool CerthGrasping::removeDebris()
{
    //Capture an image and publish it to /camera/Image topic
    cv::Mat rgb, mask;
    if (!getNewImage(rgb, mask))
        return false;
    ros::Duration(2).sleep();

    //Call spring detection
    detectSprings();

    //Printing out the results
    cout << "Springs at: " << endl;
    for (uint j=0; j<spring_list.size(); j++)
    {
        cv::Point centroid = calculateSpringCenter(spring_list[j]);
        cout << "x = " << centroid.x << ", y = " << centroid.y << ", rotX = " << rotX[j] << ", rotY = " << rotY[j] <<endl;
    }


    bool run_detection = false;
    int number_of_detected_springs = spring_list.size(); //spring_list changes through the iterations

    //Spring Grasping
    for (uint i=0; i<number_of_detected_springs; i++)
    {

        cout << "----------------Spring n." << i << "----------------" << endl;

        //those two variables are only for testing
        int counter = 0; //counts the number of pushes performed

        cv::Mat im_patch; //input for the agent

        cout << "run detection:"  << run_detection << endl;
        if (run_detection)
        {

            if (!getNewImage(rgb, mask))
                return false;
            ros::Duration(1).sleep();

            auto start = std::chrono::high_resolution_clock::now();
            //Call spring detection
            detectSprings();
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            std::cout << "Detection for the remaining springs: " << elapsed.count() << " s\n";

            run_detection = false;
        }
        cout << rgb.size() << endl;
        while (true)
        {
            cout << "Size:" <<spring_list.size() << endl;
            cv::Point centroid = calculateSpringCenter(spring_list[0]); //it's important to take the first element of the vector, because the iteration might start after a push and the vector will contain only one point

            findImagePatch(rgb, centroid, patch_offset, im_patch);
            cv::imwrite("/tmp/spring_patch.png", im_patch);
            float spring_orientation = springToCameraOrientation(0); //spring orientation in the camera cs, the input parameter defines the i-th spring detected

            // call the push estimation function push(centroid, spring_orientation, rgb, mask!!!!!
            push_debris_service_call(centroid.x, centroid.y, spring_orientation, rgb);
            cout << "Debris is cleared:" <<  is_cleared << endl;
            if(spring_list.size() != 0)
            {
                if (is_cleared == true)
                {



                    cv::Point new_centroid = calculateSpringCenter(spring_list[0]);
                    Vector3f new_spring_position_world = calculateWorldCoordinates(new_centroid);

                    float grasp_gripper_angle = calculateGraspingAngle(0);
                    grasp_gripper_angle = angleCorrection(grasp_gripper_angle);

                    std::cout << "Press enter to remove debris for next spring" << std::endl;
                    cin.get();
                    run_detection = true;
                    mask.release();
                    std::cout << "Cleaning debris..." << std::endl;
                    break;

    //                    Vector3f gripper_position;
    //                    gripper_position = calculateGripperPosition(new_spring_position_world, grasp_gripper_angle, false);
/*
                    if (graspSpring(new_spring_position_world, grasp_gripper_angle))
                    {
                        //cout << "Pushes = "<< counter << endl;
                        run_detection = true;
                        mask.release();
                        break;
                    }
                    else
                    {
                        cout << "can't grasp spring, move to the next one" << std::endl;
                        run_detection = true;
                        mask.release();
                        break;

                        //cout << "Spring clean of debris" << endl;
                        //continue;
    //                    cout << "Spring clean of debris" << endl;
    //                    break;
                    }
                    */
                }
                else
                {
                    cout << "Debris around spring! Move to next push" << endl;
                }

            }





            //-------------------this code block is only for testing----------------
            cv::Point temp_point;
/*
            float spring_orientation_corrected = spring_orientation - (M_PI / 2); // perpendicular to spring orientation

            if (counter == 0)
            {
                temp_point.x = centroid.x + 80 * cos(spring_orientation_corrected);
                temp_point.y = centroid.y + 80 * sin(spring_orientation_corrected);
                cv::circle(rgb, temp_point, 4, cv::Scalar(255, 255, 255), 4);
            }
            else
            {
                temp_point.x = centroid.x - 80 * cos(spring_orientation_corrected);
                temp_point.y = centroid.y - 80 * sin(spring_orientation_corrected);
                cv::circle(rgb, temp_point, 4, cv::Scalar(0, 0, 0), 4);
            }

*/
//            float push_orientation = spring_orientation; //just for testing, push orientatino would be different with the spring orientation

            cout << "Push estimated orientation = " << push_estimated_orientation * 180 / M_PI << endl;
            float push_orientation = push_estimated_orientation;

//            float push_direction_in_cam_frame = findPushDirectionInCameraFrame();
//            cout << "Push orientation in camera frame = " << push_direction_in_cam_frame * 180 / M_PI << endl;
//            float push_orientation = cameraToBaseOrientation(push_direction_in_cam_frame);

            temp_point.x = push_start_point_x;
            temp_point.y = push_start_point_y;
            cv::circle(rgb, temp_point, 4, cv::Scalar(0, 0, 0), 4);

            temp_point.x = push_final_point_x;
            temp_point.y = push_final_point_y;
            cv::circle(rgb, temp_point, 4, cv::Scalar(255, 0, 0), 4);

            cv::namedWindow("rgb", CV_WINDOW_NORMAL );
            //cv::imshow( "rgb", rgb );
            //cv::waitKey(0);
            //---------------------------end of testing block-----------------------

            float gripper_angle = cameraToBaseOrientation(push_orientation); //tranform the rotation angle from camera cs to world reference

            if (pushDebris(gripper_angle))
            {
                counter++;
                cout << "Push was done correctly" << endl;
            }
//            if (pushDebris(temp_point, gripper_angle, push_offset, counter))
//                counter = (counter) ? 0 : 1; //counter is used only for testing
            else //in case that pushDebris failed and the brush moved the spring, we re-detect the springs
            {
                if (!getNewImage(rgb, mask))
                    return false;
                ros::Duration(1).sleep();

                detectSprings();

                continue;
            }


            //re-detect the springs in case the examined spring was moved
            mask = makeMask(rgb, centroid, patch_offset); //search only around a small image region

            if (!getNewImage(rgb, mask))
                return false;
            ros::Duration(1).sleep();

            auto start = std::chrono::high_resolution_clock::now();
            //Call spring detection
            detectSprings();
            auto finish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = finish - start;
            std::cout << "Re-detection time " << elapsed.count() << " s\n";

            cout << spring_list.size() << " spring found after push!" << endl;

/*
            if(spring_list.size() != 0)
            {
                if (is_cluttered == false)
                {
                    cout << "Spring around debris! Move to next push" << endl << endl;
                    continue;
                }
                else
                {
                    cv::Point new_centroid = calculateSpringCenter(spring_list[0]);
                    Vector3f new_spring_position_world = calculateWorldCoordinates(new_centroid);

                    float grasp_gripper_angle = calculateGraspingAngle(0);
                    grasp_gripper_angle = angleCorrection(grasp_gripper_angle);

//                    Vector3f gripper_position;
//                    gripper_position = calculateGripperPosition(new_spring_position_world, grasp_gripper_angle, false);

                    if (graspSpring(new_spring_position_world, grasp_gripper_angle))
                    {
                        run_detection = true;
                        break;
                    }
                    else
                        continue;
//                    cout << "Spring clean of debris" << endl;
//                    break;
                }
            }
*/
        }
    }
    return true;
}





