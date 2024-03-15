#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher point_cloud_pub;
ros::Publisher laser_scan_pub;

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

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Initialize the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "sonar_frame";
    scan.angle_min = -M_PI/3;
    scan.angle_max = M_PI/3;
    scan.angle_increment = M_PI/90;
    scan.range_min = 0.005;
    scan.range_max = 1.2;
    scan.ranges.resize(180, scan.range_max);

    // Scan the image for the closest object
    for (int y = 0; y < depth_img.rows; ++y)
    {
        for (int x = 0; x < depth_img.cols; ++x)
        {
            if (depth_img.at<uchar>(y, x) > 127)  // Threshold set at mid-brightness
            {
                pcl::PointXYZ point;
                point.x = (x - 127) / 354.2f;
                point.y = (425.0f - static_cast<float>(y)) / 354.2f;
                point.z = 0;
                cloud.push_back(point);

                // Convert to polar coordinates
                double angle = atan2(point.y, point.x);
                double range = sqrt(point.x*point.x + point.y*point.y);
                int index = (angle - scan.angle_min) / scan.angle_increment;
                if (index >= 0 && index < scan.ranges.size())
                {
                    if (range < scan.ranges[index])
                    {
                        scan.ranges[index] = range;
                    }
                }
            }
        }
    }

    // Convert PCL point cloud to PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "sonar_frame";
    output.header.stamp = ros::Time::now();

    point_cloud_pub.publish(output);
    laser_scan_pub.publish(scan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_pointcloud_node");
    ros::NodeHandle nh;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link_b";  // replace with your parent frame
    transformStamped.child_frame_id = "sonar_frame";
    transformStamped.transform.translation.x = 0.13;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = -0.3;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = -0.7071;
    transformStamped.transform.rotation.w = 0.7071;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(transformStamped);

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_pointcloud", 10);
    laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_sonar", 10);  // Added this line
    ros::Subscriber image_sub = nh.subscribe("/bluerov/oculus_m1200d/sonar_image", 10, imageCallback);

    ros::spin();
    return 0;
}
