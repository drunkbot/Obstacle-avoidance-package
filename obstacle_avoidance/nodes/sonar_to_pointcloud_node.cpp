#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <marine_acoustic_msgs/ProjectedSonarImage.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher point_cloud_pub;

void sonarCallback(const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert beam_directions and ranges to point cloud
    for(size_t i = 0; i < msg->beam_directions.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = msg->ranges[i] * msg->beam_directions[i].x;
        point.y = msg->ranges[i] * msg->beam_directions[i].y;
        point.z = msg->ranges[i] * msg->beam_directions[i].z;
        cloud.push_back(point);
    }

    // Convert PCL point cloud to PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "sonar_frame";
    output.header.stamp = ros::Time::now();

    point_cloud_pub.publish(output);
    ROS_INFO("Processing new sonar data...");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_to_pointcloud_node");
    ros::NodeHandle nh;

    ros::Subscriber sonar_sub = nh.subscribe<marine_acoustic_msgs::ProjectedSonarImage>("/bluerov/oculus_m1200d/sonar_image_raw", 10, sonarCallback);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sonar_pointcloud", 10);

    ros::spin();

    return 0;
}
