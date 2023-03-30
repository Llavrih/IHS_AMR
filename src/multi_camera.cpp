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
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>


double degreesToRadians(double degrees);
void planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB> combined_cloud);

ros::Publisher pub_left;
ros::Publisher pub_right;
ros::Publisher pub;
ros::Publisher pub_plane;
ros::Publisher pub_bounding_box;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);

void cam1Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("+");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *left_cloud);

    // Randomly downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    srand(time(NULL)); // Seed the random number generator
    for (size_t i = 0; i < left_cloud->size(); ++i)
    {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (r < 0.1) // Keep 10% of the points
        {
            downsampled_cloud->push_back(left_cloud->at(i));
        }
    }


    // Perform translation and rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -0.34,-0.2, 0.0; // No translation

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
    pcl::transformPointCloud(*downsampled_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    pcl::copyPointCloud(*transformed_cloud, *transformed_cloud_left);
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

    //pub_left.publish(transformed_cloud_msg);
}


void cam2Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_INFO("-");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *right_cloud);

    // Randomly downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    srand(time(NULL)); // Seed the random number generator
    for (size_t i = 0; i < right_cloud->size(); ++i)
    {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (r < 0.1) // Keep 10% of the points
        {
            downsampled_cloud->push_back(right_cloud->at(i));
        }
    }

    // Perform translation and rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.34,-0.2, 0.0; // No translation

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
    pcl::transformPointCloud(*downsampled_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
    pcl::copyPointCloud(*transformed_cloud, *transformed_cloud_right);
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

    //pub_right.publish(transformed_cloud_msg);
}


double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

void combineAndPublishPointClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_left,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_right,
                                   ros::Publisher& pub)
{
    if (transformed_cloud_left != nullptr && transformed_cloud_right != nullptr)
    {
        // Combine both point clouds
        ROS_INFO("message");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *combined_cloud = *transformed_cloud_left + *transformed_cloud_right;
        ROS_INFO("Combined point cloud size: %lu", combined_cloud->size());

        // Publish the combined point cloud
        sensor_msgs::PointCloud2 combined_cloud_msg;
        pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
        combined_cloud_msg.header.frame_id = "cam_link_1"; // Set frame
        if (combined_cloud->size() != 0)
            planeSegmentation(*combined_cloud);
        pub.publish(combined_cloud_msg);
    }
}

void planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB> combined_cloud)
{
    std::cout << "Input point cloud size: " << combined_cloud.size() << std::endl;

    // Downsample the point cloud using a VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(combined_cloud.makeShared());
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(downsampled_cloud);
    ne.setKSearch(5);
    ne.setViewPoint(0, 0, 0); // Set view point to improve plane segmentation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    for (int i = 0; i < 50; i++)
    {
        // Set input to segmentation object
        seg.setInputCloud(downsampled_cloud);
        seg.setInputNormals(cloud_normals);

        // Set segmentation parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.5);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(50);
        seg.setDistanceThreshold(0.01);

        // Segment plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) // No more planes found
            break;

        // Extract the plane inliers from the point cloud
        extract.setInputCloud(downsampled_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*plane_cloud);

        // Add the plane inliers to the planes point cloud
        *planes_cloud += *plane_cloud;

        // Remove the plane inliers from the point cloud
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*remaining_cloud);
        downsampled_cloud = remaining_cloud;

        // Compute normals for the remaining cloud
        ne.setInputCloud(downsampled_cloud);
        ne.compute(*cloud_normals);
    }

    // Publish the planes point cloud
    sensor_msgs::PointCloud2 planes_cloud_msg;
    pcl::toROSMsg(*planes_cloud, planes_cloud_msg);
    planes_cloud_msg.header.frame_id = "cam_link_1"; // Set frame
    pub_plane.publish(planes_cloud_msg);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    // Subscribe to cam_1 point cloud topic
    ros::Subscriber cam1_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", 1, cam1Callback);

    // Subscribe to cam_2 point cloud topic
    ros::Subscriber cam2_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", 1, cam2Callback);
    
    pub_left = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud1", 1);
    pub_right = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud2", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_clouds", 1);
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);
    pub_bounding_box = nh.advertise<visualization_msgs::Marker>("plane", 1);
    while (ros::ok())
    {
        combineAndPublishPointClouds(transformed_cloud_left, transformed_cloud_right, pub);
        ros::spinOnce();
    }
    

    ros::spin();

    return 0;
}
