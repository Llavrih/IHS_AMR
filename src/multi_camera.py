#!/usr/bin/env python
import rospy
import open3d as o3d
#from open3d.geometry import crop_point_cloud
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import math
import message_filters
from ctypes import *
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
import random

pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
pub_objects = rospy.Publisher('/pointcloud_objects', PointCloud2, queue_size=10)
pub_clusters = rospy.Publisher('/pointcloud_clusters', PointCloud2, queue_size=10)


#pub_traffic_light = rospy.Publisher('/traffic_light', Float64, queue_size=10)

def callback(data_1,data_2):
    tic() 
    
    # points_PCD = points_PCD.remove_duplicated_points(0.02)
    #points = PCDToNumpy(points_PCD)

    points_PCD = combinePCD(data_1,data_2)

    original_box = DrawBoxAtPoint(0.5,1,lenght=4, r=0, g=1 , b=0.3)
    original_box_PCD = NumpyToPCD(np.array((original_box.points), dtype=np.float64)).get_oriented_bounding_box()
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    #point_original = NumpyToPCD(points)
    point_original = o3d.geometry.PointCloud.crop(points_PCD,original_box_PCD)
    
    #Escape room for increasing the speed.
    
    point_original= o3d.geometry.PointCloud.random_down_sample(point_original,0.40)
    point_original= o3d.geometry.PointCloud.uniform_down_sample(point_original,20)
    point_original.remove_statistical_outlier(nb_neighbors=500,std_ratio=0.1)
    point_original.remove_radius_outlier(nb_points=100, radius=0.1)
    downsampled_original_np = (DownSample(PCDToNumpy(point_original),0.01))
    downsampled_original = NumpyToPCD(downsampled_original_np)
   
    """ Detect multiple planes from given point clouds
    Args:
        points (np.ndarray):
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.
    """
    downsampled_original.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=200))
    downsampled_original_np = PCDToNumpy(downsampled_original)
    plane_list, index_arr = DetectMultiPlanes((downsampled_original_np), min_ratio=0.5, threshold=0.05, init_n=3, iterations=100)
    planes = []
    boxes = []
    """find boxes for planes"""
    for _, plane in plane_list:
        box = NumpyToPCD(plane).get_oriented_bounding_box()
        planes.append(plane)
        boxes.append(box)
    #print('Planes detected: ',len(boxes))
    planes_np = (np.concatenate(planes, axis=0))
    # planes = NumpyToPCD(np.concatenate(planes, axis=0))
    
    index_arr_new  =[]
    for i in range(len(index_arr)):
        index_arr_new = index_arr_new + index_arr[i]
    #print("index arr new",(index_arr_new))    
    outlier =  o3d.geometry.PointCloud.select_by_index(downsampled_original,index_arr_new,invert=True)


    outlier = PCDToNumpy(outlier)
    #print(outlier)
    mask = np.isin(outlier[:,:],(planes_np)[:,:], invert=True)
    objects = outlier[mask[:,2]]
    rows_to_delete = np.where(objects[:, 2] < 0.03)[0]
    # delete the rows from the matrix
    objects = np.delete(objects, rows_to_delete, axis=0)
    if np.size(objects) != 0:
        print(np.size(objects) )
        centers_pcd, boxes_obsticles = clusteringObjects(objects)
        if (boxes_obsticles and centers_pcd) != None:
            visualize_bounding_boxes(boxes_obsticles)
            Talker_Clusters(centers_pcd)

    objects = NumpyToPCD(objects)
    objects.paint_uniform_color([0.7, 0.8, 0.2])
    objects.remove_statistical_outlier(nb_neighbors=20,std_ratio=0.01)
    Talker_Objects(objects)
    

    Talker_PCD(downsampled_original)
    
    toc()
    

def combinePCD(data_1,data_2):
    pc1 = ros_numpy.numpify(data_1)
    pc2 = ros_numpy.numpify(data_2)
    points=np.zeros((pc1.shape[0],3))

    points1=np.zeros((pc1.shape[0],3))
    translation = [-0.15, -0.35,0]
    points1[:,0]=np.subtract(pc1['x'],translation[0])
    points1[:,1]=np.subtract(pc1['y'],translation[1])
    points1[:,2]=np.subtract(pc1['z'],translation[2])
    #Translation between cameras
    translation = [-0.15,0.35, 0]
    
    points2=np.zeros((pc2.shape[0],3))
    points2[:,0]=np.subtract(pc2['x'],translation[0])
    points2[:,1]=np.subtract(pc2['y'],translation[1])
    points2[:,2]=np.subtract(pc2['z'],translation[2])
    
    
    points1 = (np.array(points1, dtype=np.float64))
    points2 = (np.array(points2, dtype=np.float64))
    

    #Rotation between cameras
    rotation1 = [math.radians(30), math.radians(0), math.radians(0)]
    
    points_PCD1 = NumpyToPCD(points1)
    #R1 = points_PCD1.get_rotation_matrix_from_xyz((rotation1))
    R1 = [[1.0, 0.0, 0.0],[0.0, 0.8660254, -0.5],[0.0, 0.5, 0.8660254]]
    points_PCD1.rotate(R1, center=(0, 0, 0))
    points1 = PCDToNumpy(points_PCD1)

    rotation2 = [math.radians(-30), math.radians(0), math.radians(0)]
    
    points_PCD2 = NumpyToPCD(points2)
    #R2 = points_PCD2.get_rotation_matrix_from_xyz((rotation2))
    R2 = [[1.0, 0.0, 0.0],[0.0, 0.8660254, 0.5],[0.0, -0.5, 0.8660254]]
    points_PCD2.rotate(R2, center=(0, 0, 0))
    points2 = PCDToNumpy(points_PCD2)

    #Stack both pointclouds
    
    points = np.concatenate((points1,points2))
    #points = points1
 
    #Rotation of poitncloud in world cordinate system
    points_PCD = NumpyToPCD(points)
    R = points_PCD.get_rotation_matrix_from_xyz((math.radians(0), math.radians(-75), math.radians(0)))
    R = [[ 0.25881905, 0.0, -0.96592583],[ 0.0, 1.0, 0.0],[0.96592583, 0.0, 0.25881905]]

    points_PCD.rotate(R, center=(0, 0, 0))

    return points_PCD

def Talker_PCD(pointcloud):
    pointcloud = convertCloudFromOpen3dToRos((pointcloud),"cam_1_link")
    pointcloud.is_dense = True   
    pc = pointcloud
    pub.publish(pc)  

def Talker_Objects(pointcloud):
    pointcloud = convertCloudFromOpen3dToRos((pointcloud),"cam_1_link")
    pointcloud.is_dense = True
    pc = pointcloud
    pub_objects.publish(pc)  

def Talker_Clusters(pointcloud):
    pointcloud = convertCloudFromOpen3dToRos((pointcloud),"cam_1_link")
    pointcloud.is_dense = True
    pc = pointcloud
    pub_clusters.publish(pc) 

def TalkerTrafficLight(traffic_light):
    if traffic_light > 1 & traffic_light < 5:
        traffic_light = 2
    if traffic_light >= 5:
        traffic_light = 3
    pub_traffic_light.publish(traffic_light)   


def visualize_bounding_boxes(boxes):
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
                
    for i, box in enumerate(boxes):
        x = box.center[0]
        y = box.center[1]
        z = box.center[2]
        width = box.extent[2]
        height = box.extent[1]
        depth = box.extent[0]

        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "cam_1_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.scale.y = height
        marker.scale.z = depth
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0.3
        marker.color.a = 0.7
        # Publish the marker message
        marker_pub.publish(marker)

    for j in range(100):
            if j > len(boxes):
                marker = Marker()
                marker.header.frame_id = "cam_1_link"
                marker.id = j  # ID of the marker to delete
                marker.action = Marker.DELETE
                marker_pub.publish(marker)
    
    

def clusteringObjects(objects):
    # Perform DBSCAN clustering on the objects
    clustering = DBSCAN(eps=0.2, min_samples=30).fit(objects)
    # Extract the labels for each point indicating the cluster it belongs to
    labels = clustering.labels_
    # Identify the number of clusters and the points belonging to each cluster
    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    if num_clusters == 0:
        return
    else:
        clusters = [objects[labels == i] for i in range(num_clusters)]
        # Compute the centers and bounding boxes of the clusters
        centers = [np.mean(cluster, axis=0) for cluster in clusters]
        boxes = []
        for cluster in clusters:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(cluster)
            if len(pcd.points) < 4:
                return None, None
            bbox = pcd.get_oriented_bounding_box()
            boxes.append(bbox)
        # Convert the centers and bounding boxes to point cloud objects
        centers_pcd = o3d.geometry.PointCloud()
        centers_pcd.points = o3d.utility.Vector3dVector(np.stack(centers, axis=0))
        boxes_pcd = boxes
        return centers_pcd, boxes_pcd
   
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    """ converts PCD to PointCloud2
    Args:
        points (ndarray): N x 3 point clouds
    Returns:
        PointCloud2
    """ 
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),]
    points=np.asarray(open3d_cloud.points)
    cloud_data=points

   
    # create ros_cloud
    #print(header)
    #print(cloud_data)
    return pc2.create_cloud(header, fields, cloud_data)

def DistanceCalculator(arr):
    """ calculate distance from oroigin to points
    Args:
        points (ndarray): N x 3 point clouds
    Returns:
        [ndarray]: distances
    """ 
    distance_arr = []
    for i in range(len(arr)):
        distance = np.sqrt(np.power(arr[i,0],2)+np.power(arr[i,1],2)+np.power(arr[i,2],2))
        distance_arr.append(distance)
    return distance_arr

       
def DrawBoxAtPoint(center, edgeLength, lenght, r, g, b):
    #points = np.array([[0, -0.05, 0], [-0.05, -0.05, 0], [1, -1, 1], [-1, -1, 1], [0.05, 0.05, 0], [-0.05, 0.05, 0],[1, 1, 1], [-1, 1, 1]], dtype=np.float64)
    x = lenght * np.sin(43.5/180*math.pi)
    y = lenght * np.sin(29.0/180*math.pi)
    #print(x,y)
    points = np.array([[0, 0, 0], [0, 0, 0], [x, -y, lenght], [-x, -y, lenght], [0, 0, 0], [0, 0, 0],[x, y, lenght], [-x, y, lenght]], dtype=np.float64)
   
    for i in range(len(points)):
        point = points[i]*edgeLength
        points[i] = np.add(point, center-edgeLength/2)
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[r, g, b] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def DrawAMR(center, edgeLength, lenght, r, g, b):
    #points = np.array([[0, -0.05, 0], [-0.05, -0.05, 0], [1, -1, 1], [-1, -1, 1], [0.05, 0.05, 0], [-0.05, 0.05, 0],[1, 1, 1], [-1, 1, 1]], dtype=np.float64)
    lenght =1.63
    x = 0.55
    y = 0.25
    points = np.array([[-x, -y, -lenght], [x, -y, -lenght], [-x, -y, 0],[x, -y, 0], [-x, y, -lenght],[x, y, -lenght],[-x, y, 0], [x, y, 0]], dtype=np.float64)
   
    for i in range(len(points)):
        point = points[i]*edgeLength
        points[i] = np.add(point, center-edgeLength/2)
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[r, g, b] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def DrawBoxForward(center, edgeLength,v,v_max,direction, lenght, r, g, b):
    #points = np.array([[0, -0.05, 0], [-0.05, -0.05, 0], [1, -1, 1], [-1, -1, 1], [0.05, 0.05, 0], [-0.05, 0.05, 0],[1, 1, 1], [-1, 1, 1]], dtype=np.float64)
    x = (lenght * np.sin(43.5/180*math.pi))
    y = (lenght * np.sin(29.0/180*math.pi))
    extension = 0
    direction = direction * lenght/5
   
    if (x > 2.1):
        extension = lenght - 2.1 / np.sin(43.5/180*math.pi)
        if direction == 0:
            lenght = 2.1 / np.sin(43.5/180*math.pi)
            x = 2.1
            y = (lenght * np.sin(29.0/180*math.pi))
   
    if (x + abs(direction) > x):
        x = (lenght * np.sin(43.5/180*math.pi)) -abs(direction)
        if x < 1:
            x = (lenght * np.sin(43.5/180*math.pi))
            direction = 0
       
    #print('x: ',x,'y: ',y,'lenght', lenght, 'extension', extension)
    points = np.array([[0, 0, 0], [0, 0, 0], [x + direction, -y, lenght], [-x+ direction, -y, lenght], [0, 0, 0], [0, 0, 0],[x+ direction, y, lenght], [-x+ direction, y, lenght], [-x+ direction*2, -y, lenght + extension], [x + direction*2, -y, lenght + extension],[-x+ direction*2, y, lenght + extension], [x + direction*2, y, lenght + extension]], dtype=np.float64)
   
    for i in range(len(points)):
        point = points[i]*edgeLength
        points[i] = np.add(point, center-edgeLength/2)
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                [0, 4], [1, 5], [2, 6], [3, 7],[7,10],[6,11],[10,11],[3,8],[8,10],[2,9],[9,11],[9,8]]
    colors = [[r, g, b] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def TicTocGenerator():
    # Generator that returns time differences
    ti = 0           # initial time
    tf = time.time() # final time
    while True:
        ti = tf
        tf = time.time()
        diff = tf-ti
        if diff == 0:
            diff = 0.001
        yield ((1/diff)) # returns the time difference

TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# This will be the main function through which we define both tic() and toc()
def toc(tempBool=True):
    # Prints the time difference yielded by generator instance TicToc
    tempTimeInterval = next(TicToc)
    if tempBool:
        #print( "Elapsed time: %f seconds.\n" %tempTimeInterval )
        print( "Elapsed frequency: %f Hz.\n" %tempTimeInterval )

def tic():
    # Records a time in TicToc, marks the beginning of a time interval
    toc(False)

def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray
    Args:
        pcd (open3d.geometry.PointCloud):
    Returns:
        [ndarray]:
    """

    return np.asarray(pcd.points)


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud
    Args:
        xyz (ndarray):
    Returns:
        [open3d.geometry.PointCloud]:
    """

    pcd = o3d.geometry.PointCloud()
    xyz = np.array(xyz, dtype = np.float64)
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd

def RemoveNan(points):
    """ remove nan value of point clouds
    Args:
        points (ndarray): N x 3 point clouds
    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]


def DownSample(pts, voxel_size):
    """ down sample the point clouds
    Args:
        pts (ndarray): N x 3 input point clouds
        voxel_size (float, optional): voxel size. Defaults to 0.003.
    Returns:
        [ndarray]:
    """

    p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

    return PCDToNumpy(p)

def PlaneRegression(points, threshold, init_n, iter):
    """ planepub = rospy.Publisher('/pc', PointCloud2, queue_size=10) regression using ransac
    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.
    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    pcd.points = o3d.utility.Vector3dVector(points)

    _, inliers = pcd.segment_plane(distance_threshold=threshold,
                                   ransac_n=init_n,
                                   num_iterations=iter)

    inlier_points = np.asarray(pcd.select_by_index(inliers).points)

    A = np.c_[inlier_points[:, 0], inlier_points[:, 1], np.ones_like(inlier_points[:, 0])]
    w = np.linalg.lstsq(A, inlier_points[:, 2], rcond=None)[0]
    
    return w, inliers

def DetectMultiPlanes(points, min_ratio, threshold, init_n, iterations):
    """ Detect multiple planes from given point clouds
    Args:
        points (np.ndarray):
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.
    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    N = len(points)
    points_copy = points.copy()
    index_arr = []

    while len(points_copy) > min_ratio * N:
        w, index = PlaneRegression(
            points_copy, threshold=threshold, init_n=init_n, iter=iterations)

        plane_list.append((w, points_copy[index]))
        points_copy = np.delete(points_copy, index, axis=0)
        index_arr.append(index)

    return plane_list, index_arr


def DrawResult(points,colors):
    pcd = o3d.geometry.PointCloud(points)
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

rospy.init_node('listener', anonymous=True)
data_1 = message_filters.Subscriber("/cam_1/depth/color/points", PointCloud2)
data_2 = message_filters.Subscriber("/cam_2/depth/color/points", PointCloud2)

#drive_mode = message_filters.Subscriber("/lizard/drive_supervisor/mode", UInt8)
#vel = message_filters.Subscriber("/lizard/velocity_controller/cmd_vel", Twist)
ts = message_filters.ApproximateTimeSynchronizer([data_1, data_2], 1, 1,True)
ts.registerCallback(callback)
rospy.spin()
