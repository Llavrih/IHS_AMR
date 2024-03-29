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
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h> 
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h> // Include for OpenMP parallelization
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h> // Include for FLANN search
#include <pcl/filters/passthrough.h>
#include <omp.h>
#include <future>
#include <utility> // for std::move()
#include <chrono>
#include <ctime>
#include <iostream>


double degreesToRadians(double degrees);
void planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB> combined_cloud);

ros::Publisher pub_left;
ros::Publisher pub_right;
ros::Publisher pub;
ros::Publisher pub_plane;
ros::Publisher pub_bounding_box;
ros::Publisher pub_outliers;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void cam1Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *left_cloud);

    // Randomly downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    srand(time(NULL)); // Seed the random number generator
    for (size_t i = 0; i < left_cloud->size(); ++i)
    {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (r < 0.05) // Keep 10% of the points
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *right_cloud);

    // Randomly downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    srand(time(NULL)); // Seed the random number generator
    for (size_t i = 0; i < right_cloud->size(); ++i)
    {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        if (r < 0.05) // Keep 10% of the points
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
    if (!transformed_cloud_left->empty() && !transformed_cloud_right->empty())
    {
        // Combine both point clouds

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *combined_cloud = *transformed_cloud_left + *transformed_cloud_right;


        // Apply MLS smoothing
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
        mls.setInputCloud(combined_cloud);
        mls.setSearchRadius(0.03);
        mls.setPolynomialOrder(2);
        mls.process(*cloud_smoothed);

        // Apply transform to the smoothed combined point cloud
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (-75.0f * M_PI / 180.0f, Eigen::Vector3f::UnitX()));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud (*cloud_smoothed, *transformed_cloud, transform);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = passThroughFilter(transformed_cloud);
        // Publish the transformed point cloud
        sensor_msgs::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.frame_id = "cam_link_1"; // Set frame
        if (filtered_cloud->size() != 0)
            
            planeSegmentation(*filtered_cloud);
        pub.publish(transformed_cloud_msg);
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Filter in z direction
    pcl::PassThrough<pcl::PointXYZRGB> passZ;
    passZ.setInputCloud(cloud);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits(-std::numeric_limits<float>::infinity(), 0.10);
    passZ.filter(*filtered_cloud);
    
    // Filter in y direction
    pcl::PassThrough<pcl::PointXYZRGB> passY;
    passY.setInputCloud(filtered_cloud);
    passY.setFilterFieldName("y");
    passY.setFilterLimits(-3.0, 3.0);
    passY.filter(*filtered_cloud);
    
    return filtered_cloud;
}



void planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB> combined_cloud)
{

    // Downsample the point cloud using a VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(combined_cloud.makeShared());
    voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planes_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Change the search method to pcl::search::flann::KdTree with epsilon for approximate search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setEpsilon(0.01); // Set epsilon for approximate search

    // Change normal estimation to use OpenMP parallelization and pcl::search::flann::KdTree search method
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(downsampled_cloud);
    ne.setKSearch(50);
    ne.setViewPoint(0, 0, 0); // Set view point to improve plane segmentation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<Eigen::Vector4f> min_points, max_points;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Preallocate memory
    planes_cloud->reserve(50 * 1000); // Assuming 1000 points per plane on average
    outlier_cloud->reserve(combined_cloud.size());
    remaining_cloud->reserve(combined_cloud.size());

    #pragma omp parallel for
    for (int i = 0; i < 10; i++)
    {
        // Set input to segmentation object
        seg.setInputCloud(downsampled_cloud);
        seg.setInputNormals(cloud_normals);

        // Set segmentation parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.3);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(10);
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

        // Compute the bounding box for the segmented plane
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);

        // Define a CropBox filter to remove points inside the bounding box
        pcl::CropBox<pcl::PointXYZRGB> crop_box;
        crop_box.setInputCloud(downsampled_cloud);
        crop_box.setMin(Eigen::Vector4f(min_pt.x, min_pt.y, min_pt.z, 1.0));
        crop_box.setMax(Eigen::Vector4f(max_pt.x, max_pt.y, max_pt.z, 1.0));
        float box_size_x = max_pt.x - min_pt.x;
        float box_size_y = max_pt.y - min_pt.y;
        float box_size_z = max_pt.z - min_pt.z;

        //std::cout << "Box size: x=" << box_size_x << " y=" << box_size_y << " z=" << box_size_z << std::endl;

        // Save the bounding box for the segmented plane
        Eigen::Vector4f min_pt_eigen(min_pt.x, min_pt.y, min_pt.z, 1.0);
        Eigen::Vector4f max_pt_eigen(max_pt.x, max_pt.y, max_pt.z, 1.0);
        min_points.push_back(min_pt_eigen);
        max_points.push_back(max_pt_eigen);


        // Add the plane inliers to the planes point cloud
        *planes_cloud += *plane_cloud;

        // Remove the plane inliers from the point cloud
        extract.setNegative(true);
        extract.filter(*remaining_cloud);

        // Crop out inliers
        crop_box.setNegative(true);
        crop_box.filter(*remaining_cloud);

        // Swap pointers to reuse memory
        std::swap(downsampled_cloud, remaining_cloud);

        // Compute normals for the remaining cloud
        ne.setInputCloud(downsampled_cloud);
        ne.compute(*cloud_normals);
    }

    // Publish the planes point cloud
    sensor_msgs::PointCloud2 planes_cloud_msg;
    pcl::toROSMsg(*planes_cloud, planes_cloud_msg);
    planes_cloud_msg.header.frame_id = "cam_link_1"; // Set frame
    pub_plane.publish(planes_cloud_msg);

    // Crop out all inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud = downsampled_cloud;
    for (size_t i = 0; i < min_points.size(); i++)
    {
        pcl::CropBox<pcl::PointXYZRGB> crop_box;
        crop_box.setInputCloud(cropped_cloud);
        crop_box.setMin(min_points[i]);
        crop_box.setMax(max_points[i]);
        crop_box.setNegative(true); // Set to true to keep points outside the box
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        crop_box.filter(*remaining_cloud);
        cropped_cloud = remaining_cloud;
    }

    // Assign the cropped_cloud to the outlier_cloud
    outlier_cloud = cropped_cloud;

    // Apply Radius Outlier Removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
    outlier_removal.setInputCloud(outlier_cloud);
    outlier_removal.setRadiusSearch(0.02);
    outlier_removal.setMinNeighborsInRadius(10);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_outlier_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    outlier_removal.filter(*filtered_outlier_cloud);

    // Assign the filtered_outlier_cloud to the outlier_cloud
    outlier_cloud = filtered_outlier_cloud;

    // Publish the outlier point cloud
    sensor_msgs::PointCloud2 outlier_cloud_msg;
    pcl::toROSMsg(*outlier_cloud, outlier_cloud_msg);
    outlier_cloud_msg.header.frame_id = "cam_link_1"; // Set frame
    pub_outliers.publish(outlier_cloud_msg);
    auto now = std::chrono::system_clock::now();
    auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    std::cout << "Unix time: " << milliseconds_since_epoch.count() << " milliseconds since epoch" << std::endl;

} 

void cam1CallbackWrapper(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cam1Callback(cloud_msg);
}

void cam2CallbackWrapper(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cam2Callback(cloud_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber cam1_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", 1, cam1CallbackWrapper);
    ros::Subscriber cam2_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", 1, cam2CallbackWrapper);

    
    pub_left = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud1", 1);
    pub_right = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud2", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_clouds", 1);
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);
    pub_outliers = nh.advertise<sensor_msgs::PointCloud2>("outlier_points", 1);
    

    while (ros::ok())
    {
        std::future<void> cam1_future = std::async(std::launch::async, [&]() {
            combineAndPublishPointClouds(transformed_cloud_left, transformed_cloud_right, pub);
        });
        std::future<void> cam2_future = std::async(std::launch::async, [&]() {
            combineAndPublishPointClouds(transformed_cloud_left, transformed_cloud_right, pub);
        });

        cam1_future.wait();
        cam2_future.wait();

        ros::spinOnce();
    }
    

    ros::spin();

    return 0;
}
