#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>


double degreesToRadians(double degrees);

ros::Publisher pub1;
ros::Publisher pub2;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam1Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("+");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *left_cloud);

    // Perform translation and rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -0.35,-0.2, 0.0; // No translation

    double zDegrees = -90.0;
    double zRadians = degreesToRadians(zDegrees);
    transform.rotate(Eigen::AngleAxisf(zRadians, Eigen::Vector3f::UnitZ())); // No rotation around z-axis

    double xDegrees = -61.0/2;
    double xRadians = degreesToRadians(xDegrees);
    transform.rotate(Eigen::AngleAxisf(xRadians, Eigen::Vector3f::UnitX())); // Rotation around x-axis

    double yDegrees = 0.0;
    double yRadians = degreesToRadians(yDegrees);
    transform.rotate(Eigen::AngleAxisf(yRadians, Eigen::Vector3f::UnitY())); // Rotation around y-axis

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*left_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    transformed_cloud_msg.header = cloud_msg->header;
    transformed_cloud_msg.header.frame_id = "cam_link_1"; // Set parent frame

    // Publish transformed point cloud with child frame
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "cam_link_1"; // Set parent frame
    transformStamped.child_frame_id = "cam_link_left"; // Set child frame
    transformStamped.transform.translation.x = transform.translation().x();
    transformStamped.transform.translation.y = transform.translation().y();
    transformStamped.transform.translation.z = transform.translation().z();
    Eigen::Quaternionf quat(transform.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    br.sendTransform(transformStamped);

    pub1.publish(transformed_cloud_msg);
    return transformed_cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cam2Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("-");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *right_cloud);

    // Perform translation and rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.35,-0.2, 0.0; // No translation

    double zDegrees = -90.0;
    double zRadians = degreesToRadians(zDegrees);
    transform.rotate(Eigen::AngleAxisf(zRadians, Eigen::Vector3f::UnitZ())); // No rotation around z-axis

    double xDegrees = 61.0/2;
    double xRadians = degreesToRadians(xDegrees);
    transform.rotate(Eigen::AngleAxisf(xRadians, Eigen::Vector3f::UnitX())); // Rotation around x-axis

    double yDegrees = 0.0;
    double yRadians = degreesToRadians(yDegrees);
    transform.rotate(Eigen::AngleAxisf(yRadians, Eigen::Vector3f::UnitY())); // Rotation around y-axis

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*right_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    transformed_cloud_msg.header = cloud_msg->header;
    transformed_cloud_msg.header.frame_id = "cam_link_1"; // Set parent frame

    // Publish transformed point cloud with child frame
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "cam_link_1"; // Set parent frame
    transformStamped.child_frame_id = "cam_link_right"; // Set child frame
    transformStamped.transform.translation.x = transform.translation().x();
    transformStamped.transform.translation.y = transform.translation().y();
    transformStamped.transform.translation.z = transform.translation().z();
    Eigen::Quaternionf quat(transform.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    br.sendTransform(transformStamped);

    pub2.publish(transformed_cloud_msg);
    return transformed_cloud;
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    // Subscribe to cam_1 point cloud topic
    ros::Subscriber cam1_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", 1, cam1Callback);

    // Subscribe to cam_2 point cloud topic
    ros::Subscriber cam2_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", 1, cam2Callback);

    pub1 = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud1", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud2", 1);

    ros::spin();

    return 0;
}
