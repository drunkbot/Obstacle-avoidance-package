#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


ros::Publisher scan_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");  // Assuming 8-bit monochrome image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat& depth_img = cv_ptr->image;

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "lidar_frame";
    scan.angle_min = -M_PI/3;  // Assuming a 180-degree horizontal field of view
    scan.angle_max = M_PI/3;
    scan.angle_increment = M_PI/90;  // 1 degree per scan
    scan.range_min = 0.00005;
    scan.range_max = 425.0f / 80.0f;  // Max distance scaled by 1/40

    int num_scans = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_scans, scan.range_max);

    for (int x = 0; x < depth_img.cols; ++x)
    {
        float min_distance = scan.range_max;
        for (int y = 0; y < depth_img.rows; ++y)
        {
            if (depth_img.at<uchar>(y, x) > 127)  // Threshold set at mid-brightness
            {
                float distance = (425.0f - static_cast<float>(y)) / 40.0f;
                min_distance = std::min(min_distance, distance);
            }
        }

        int index = static_cast<int>((x - 127) * (num_scans / depth_img.cols));
        if (index >= 0 && index < num_scans)
        {
            scan.ranges[index] = min_distance;
        }
    }

    scan_pub.publish(scan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_scan_node");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "bluerov/";
    transformStamped.child_frame_id = "lidar_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;


    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/lidar_scan", 10);
    ros::Subscriber image_sub = nh.subscribe("/bluerov/oculus_m1200d/sonar_image", 10, imageCallback);
    ros::Rate r(30);
    while(ros::ok())
    {
        // Continuously broadcast the static transform
        transformStamped.header.stamp = ros::Time::now();
        tf_broadcaster.sendTransform(transformStamped);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
