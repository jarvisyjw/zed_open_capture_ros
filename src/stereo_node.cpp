#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <thread>
// #include <mutex>
#include "videocapture.hpp"
// #include "videocapture_def.hpp"
// #include "sensorcapture.hpp"
#include <opencv2/opencv.hpp>

// // Global variables
// std::mutex imuMutex;
// std::string imuTsStr;
// std::string imuAccelStr;
// std::string imuGyroStr;

// bool sensThreadStop = false;
// uint64_t mcu_sync_ts = 0;

// IMU publisher
// ros::Publisher imu_pub;

// Function to publish IMU data
// void publishIMUData(sl_oc::sensors::SensorCapture* sensCap) {
//     sensThreadStop = false;
//     uint64_t last_imu_ts = 0;

//     while (!sensThreadStop) {
//         const sl_oc::sensors::data::Imu imuData = sensCap->getLastIMUData(2000);

//         if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
//             sensor_msgs::Imu imu_msg;

//             imu_msg.header.stamp = ros::Time::now();
//             imu_msg.header.frame_id = "imu_link";

//             imu_msg.linear_acceleration.x = imuData.aX;
//             imu_msg.linear_acceleration.y = imuData.aY;
//             imu_msg.linear_acceleration.z = imuData.aZ;

//             imu_msg.angular_velocity.x = imuData.gX;
//             imu_msg.angular_velocity.y = imuData.gY;
//             imu_msg.angular_velocity.z = imuData.gZ;

//             imu_pub.publish(imu_msg);

//             imuMutex.lock();
//             if (imuData.sync) {
//                 mcu_sync_ts = imuData.timestamp;
//             }
//             imuMutex.unlock();
//         }
//     }
// }

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "stereo_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Publisher camera_pub = it.advertise("stereo_camera/image_raw", 10);

    // Publishers
    image_transport::Publisher left_camera_pub = it.advertise("stereo_camera/frame_left/image_raw", 10);
    image_transport::Publisher right_camera_pub = it.advertise("stereo_camera/frame_right/image_raw", 10);
    // imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);

    // Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = sl_oc::VERBOSITY::ERROR;

    // Create Video Capture object
    sl_oc::video::VideoCapture videoCap(params);
    if (!videoCap.initializeVideo(-1)) {
        ROS_ERROR("Cannot open camera video capture");
        return EXIT_FAILURE;
    }

    int camSn = videoCap.getSerialNumber();
    ROS_INFO("Video Capture connected to camera sn: %d", camSn);

    // Create Sensors Capture object
    // sl_oc::sensors::SensorCapture sensCap(sl_oc::VERBOSITY::ERROR);
    // if (!sensCap.initializeSensors(camSn)) {
        // ROS_ERROR("Cannot open sensors capture");
        // return EXIT_FAILURE;
    // }
    // ROS_INFO("Sensors Capture connected to camera sn: %d", sensCap.getSerialNumber());

    // Start the sensor capture thread
    // std::thread sensThread(publishIMUData, &sensCap);

    // Enable video/sensors synchronization
    // videoCap.enableSensorSync(&sensCap);

    int w, h;
    videoCap.getFrameSize(w, h);

    cv::Mat frameBGR(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    uint64_t last_timestamp = 0;

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        // Get the last video frame
        const sl_oc::video::Frame frame = videoCap.getLastFrame(1);

        if (frame.data != nullptr && frame.timestamp != last_timestamp) {
            last_timestamp = frame.timestamp;

            // Convert YUV to BGR
            cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            // Publish left camera image
            cv::Mat left_raw = frameBGR(cv::Rect(0, 0, w / 2, h));
            cv::Mat right_raw = frameBGR(cv::Rect(w / 2, 0, w / 2, h));

            sensor_msgs::ImagePtr left_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_raw).toImageMsg();
            sensor_msgs::ImagePtr right_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_raw).toImageMsg();

            ros::Time now = ros::Time::now();
            left_img_msg->header.stamp = now;
            right_img_msg->header.stamp = now;
            right_img_msg->header.frame_id = "zed_right";
            left_img_msg->header.frame_id = "zed_left";

            left_camera_pub.publish(left_img_msg);
            right_camera_pub.publish(right_img_msg);


            // std_msgs::Header header;
            // header.stamp = ros::Time::now();
            // header.frame_id = "camera_link";
            // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frameLeft).toImageMsg();
            // left_camera_pub.publish(img_msg);

            // // // Publish right camera image
            // cv::Mat frameRight = frameBGR(cv::Rect(w / 2, 0, w / 2, h));
            // header.stamp = ros::Time::now();
            // header.frame_id = "camera_link";
            // img_msg = cv_bridge::CvImage(header, "bgr8", frameRight).toImageMsg();
            // right_camera_pub.publish(img_msg);
        
            // Convert OpenCV image to ROS Image message
            // std_msgs::Header header;
            // header.stamp = ros::Time::now();
            // header.frame_id = "camera_link";

            // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frameBGR).toImageMsg();
            // camera_pub.publish(img_msg);
        }

        // Check for shutdown signal
        // if (!ros::ok()) {
        //     sensThreadStop = true;
        //     sensThread.join();
        //     break;
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
