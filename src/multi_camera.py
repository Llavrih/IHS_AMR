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
import math as m
import message_filters
from ctypes import *
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
import random
import threading
import matplotlib.path as mplPath  
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import tf_conversions



def DetectObjects(data_1,data_2,drive_mode):
    global load
    load = 0.5
    point_original = combinePCD(data_1,data_2)
    global traffic_light_up
    global traffic_light_floor
    
    tic()
    def DetectObjectsOnFloor(point_original, drive_mode):
        global load
        global traffic_light_floor

        # Escape room for increasing the speed.
        point_cloud = PCDToNumpy(point_original)
        rows_to_delete = np.where(point_cloud[:, 2] > 0.1)[0]
        point_cloud_floor = np.delete(point_cloud, rows_to_delete, axis=0)
        point_cloud_floor_pcd = NumpyToPCD(point_cloud_floor)

        plane_list, index_arr = DetectMultiPlanes((point_cloud_floor), min_ratio=0.2, threshold=0.01, init_n=3, iterations=50)

        planes = []
        boxes = []

        """find boxes for planes"""
        for _, plane in plane_list:
            box = NumpyToPCD(plane).get_oriented_bounding_box()
            planes.append(plane)
            boxes.append(box)
        planes_np = (np.concatenate(planes, axis=0))

        index_arr_new = np.concatenate(index_arr, axis=0)
        outlier = o3d.geometry.PointCloud.select_by_index(point_cloud_floor_pcd, index_arr_new, invert=True)
        outlier_np = PCDToNumpy(outlier)

        planes_mask = ~np.isin(outlier_np, planes_np).any(axis=1)
        objects = outlier_np[planes_mask]

        cl_arr = o3d.geometry.PointCloud()
        radii = np.array([0.002 + 0.002 * i for i in range(0, 4, 1)])
        nb_points = np.array([20 + 1 * i for i in range(0, 4, 1)])
        distance_cut = np.array([i * 1 for i in range(0, 4, 1)])

        for i in range(4):
            distance_mask = abs(objects[:, 0]) <= distance_cut[i]
            objects_cut = objects[distance_mask]
            if len(objects_cut) > 0:
                cl = o3d.geometry.PointCloud()
                cl.points = o3d.utility.Vector3dVector(objects_cut)
                cl, _ = cl.remove_radius_outlier(nb_points=nb_points[i], radius=radii[i])
                cl_arr += cl

        objects_viz = cl_arr
        objects_viz_np = PCDToNumpy(objects_viz)

        if np.size(objects_viz_np) != 0:
            centers_pcd = clusteringObjects(objects_viz_np)
            if centers_pcd is not None:
                traffic_light_floor = DetectTraffic(PCDToNumpy(objects_viz), load, drive_mode)
                Talker_PCD(centers_pcd, 4)
                Talker_PCD(objects_viz, 1)
            else:
                traffic_light_floor = DetectTraffic([], load, drive_mode)

        Talker_PCD(point_cloud_floor_pcd, 0)
        
        

    def DetectObjectsInTheAir(point_original,drive_mode):

        global load
        global traffic_light_up
        traffic_light_up = []
        point_cloud = PCDToNumpy(point_original)
        rows_to_delete = np.where(point_cloud[:, 2] < 0.05)[0]
        point_cloud_up = np.delete(point_cloud, rows_to_delete, axis=0)
        point_cloud_up = (DownSample((point_cloud_up),0.01))
        point_cloud_up_pcd = NumpyToPCD(point_cloud_up)
        point_cloud_up_pcd= o3d.geometry.PointCloud.random_down_sample(point_cloud_up_pcd,0.5)
        point_cloud_up_pcd= o3d.geometry.PointCloud.uniform_down_sample(point_cloud_up_pcd,10)

        downsampled_original_np = PCDToNumpy(point_cloud_up_pcd)
        plane_list, index_arr = DetectMultiPlanes((downsampled_original_np), min_ratio=0.55, threshold=0.1, init_n=3, iterations=50)
        planes = []
        boxes = []
        """find boxes for planes"""
        for _, plane in plane_list:
            box = NumpyToPCD(plane).get_oriented_bounding_box()
            planes.append(plane)
            boxes.append(box)

        planes_np = (np.concatenate(planes, axis=0))


        index_arr_new  =[]
        for i in range(len(index_arr)):
            index_arr_new = index_arr_new + index_arr[i]
        #print("index arr new",(index_arr_new))    
        outlier =  o3d.geometry.PointCloud.select_by_index(point_cloud_up_pcd,index_arr_new,invert=True)


        outlier = PCDToNumpy(outlier)
        #print(outlier)
        mask = np.isin(outlier[:,:],(planes_np)[:,:], invert=True)
        objects = outlier[mask[:,2]]
        
        objects_viz = NumpyToPCD(objects)

        traffic_light_up = DetectTraffic(PCDToNumpy(objects_viz),load,drive_mode)
        Talker_PCD(point_cloud_up_pcd,2)
        Talker_PCD(objects_viz,3)

    
    try:
        
        thread1 = threading.Thread(target=DetectObjectsOnFloor, args = [point_original,drive_mode])
        thread2 = threading.Thread(target=DetectObjectsInTheAir, args = [point_original,drive_mode])

        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        TalkerTrafficLight(min(min(traffic_light_floor),min(traffic_light_up)))
        #TalkerTrafficLight(min(traffic_light_floor))
        


    except:
        print("Error: unable to start thread")

    toc()

def DetectTraffic(objects,load,drive_mode):
    drive_mode = np.array(drive_mode.data)
    print(drive_mode)
    if len(objects) == 0:
        return [6]
    else: 
        # {
        # UNDEFINED = 0,
        # NORTH = 1,
        # SOUTH = 2,
        # NORTH_WEST_MILD = 3,
        # SOUTH_WEST_MILD = 4,
        # NORTH_WEST_SHARP = 5,
        # SOUTH_WEST_SHARP = 6,
        # NORTH_EAST_MILD = 7,
        # SOUTH_EAST_MILD = 8,
        # NORTH_EAST_SHARP = 9,
        # SOUTH_EAST_SHARP = 10,
        # ROTATE = 11,
        # DOCKING = 12,
        # STANDSTILL = 13,
        # MANUAL = 14,
        # SUPER_MANUAL = 15,
        # }
        vertices_arr_left_post = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]]
        vertices_arr_right_post = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]]
        vertices_arr_forward = [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]]
        drive_zones = [vertices_arr_forward,
        vertices_arr_forward,
        vertices_arr_forward,
        vertices_arr_right_post,
        vertices_arr_left_post,
        vertices_arr_right_post,
        vertices_arr_left_post,
        vertices_arr_left_post,
        vertices_arr_right_post,
        vertices_arr_left_post,
        vertices_arr_right_post,
        vertices_arr_forward,
        vertices_arr_forward,
        vertices_arr_forward,
        vertices_arr_forward,
        vertices_arr_forward] 
        object_in_zone = []
        
        for obj in objects:

            for i in range(6):
                point = (obj[1],abs(obj[0]))
                #vertices = vertices_arr_forward[i]
                vertices = drive_zones[drive_mode][i]

                poly_path = mplPath.Path(np.array([vertices[0],
                                                    vertices[1],
                                                    vertices[2],
                                                    vertices[3],
                                                    vertices[4],
                                                    vertices[5],
                                                    vertices[0]]))
                if poly_path.contains_point(point) == True:
                    object_in_zone.append(i)
                    #print('point is in', point, 'zone',i)
                    break
        #print(object_in_zone)
        return object_in_zone
        

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

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(np.deg2rad(theta)),-np.sin(np.deg2rad(theta))],
                   [ 0, np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta))]])
  
def Ry(theta):
  return np.matrix([[ np.cos(np.deg2rad(theta)), 0, np.sin(np.deg2rad(theta))],
                   [ 0           , 1, 0           ],
                   [-np.sin(np.deg2rad(theta)), 0, np.cos(np.deg2rad(theta))]])
  
def Rz(theta):
  return np.matrix([[ np.cos(np.deg2rad(theta)), -np.sin(np.deg2rad(theta)), 0 ],
                   [ np.sin(np.deg2rad(theta)), np.cos(np.deg2rad(theta)) , 0 ],
                   [ 0           , 0            , 1 ]])

def combinePCD(data_1, data_2):
    # Define a function that will run in a separate thread to process data_1

    # at_init = 25.4647972
    # bt_init = -555.8204027
    # ct_init = 386.0848065
    
    x_translation = 0.34
    y_translation = -0.2
    z_translation = 0.0

    x_rot = 61.587353023478485/2 
    y_rot = 0
    z_rot = -90

    br_left = tf2_ros.TransformBroadcaster()
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "cam_1_link"
    transform_stamped.child_frame_id = "cam_left_link"
    transform_stamped.transform.translation.x = -x_translation
    transform_stamped.transform.translation.y = z_translation
    transform_stamped.transform.translation.z = -y_translation

    # Assuming you have roll, pitch, and yaw angles in radians
    roll = np.deg2rad(-x_rot)
    pitch = np.deg2rad(y_rot)
    yaw = np.deg2rad(z_rot)
    quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    br_left.sendTransform(transform_stamped)

    br_right = tf2_ros.TransformBroadcaster()
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "cam_1_link"
    transform_stamped.child_frame_id = "cam_right_link"
    transform_stamped.transform.translation.x = x_translation 
    transform_stamped.transform.translation.y = z_translation
    transform_stamped.transform.translation.z = -y_translation

    # Assuming you have roll, pitch, and yaw angles in radians
    roll = np.deg2rad(x_rot)
    pitch = np.deg2rad(y_rot)
    yaw = np.deg2rad(z_rot)
    quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    br_right.sendTransform(transform_stamped)
    def process_data_1(data_1):
        global points_PCD1
        pc1 = ros_numpy.numpify(data_1)

        points1=np.zeros((pc1.shape[0],3))
        translation = [-x_translation,y_translation,z_translation]
        
        #translation = [-x_translation+x_translation_offset, -y_translation, z_translation]
        points1[:,0]=np.add(pc1['x'],0)
        points1[:,1]=np.add(pc1['y'],0)
        points1[:,2]=np.add(pc1['z'],0)
        
        points1 = (np.array(points1, dtype=np.float64))

        
        points_PCD1 = NumpyToPCD(points1)
        points_PCD1.translate(translation)
        R1 = np.array(Rz(z_rot) * Rx(-x_rot) * Ry(y_rot))
        points_PCD1.rotate(R1, center=(translation))

        return points_PCD1


    # Define a function that will run in a separate thread to process data_2
    def process_data_2(data_2):
        global points_PCD2
        pc2 = ros_numpy.numpify(data_2)
        translation = [x_translation,y_translation,z_translation]

        points2=np.zeros((pc2.shape[0],3))
        points2[:,0]=np.add(pc2['x'],0)
        points2[:,1]=np.add(pc2['y'],0)
        points2[:,2]=np.add(pc2['z'],0)
        

        points2 = (np.array(points2, dtype=np.float64))
        
        points_PCD2 = NumpyToPCD(points2)
        points_PCD2.translate(translation)

        R2 = np.array(Rz(z_rot) * Rx(x_rot) * Ry(y_rot))
        points_PCD2.rotate(R2, center=(translation))
        return points_PCD2

    # Start two threads to process data_1 and data_2 simultaneously
    thread_1 = threading.Thread(target=process_data_1, args=[data_1])
    thread_2 = threading.Thread(target=process_data_2, args=[data_2])
    thread_1.start()
    thread_2.start()

    # Wait for both threads to finish and get the results
    thread_1.join()
    thread_2.join()

    points_PCD = points_PCD1 + points_PCD2
    points_PCD= o3d.geometry.PointCloud.random_down_sample(points_PCD,0.9)
    points_PCD= o3d.geometry.PointCloud.uniform_down_sample(points_PCD,5)
    R = points_PCD.get_rotation_matrix_from_xyz((np.deg2rad(-75), np.deg2rad(0), np.deg2rad(0)))
    points_PCD.rotate(R, center=(0,0,0))

    original_box = DrawBoxAtPoint(0.5,1,lenght=4, r=0, g=1 , b=0.3)
    original_box_PCD = NumpyToPCD(np.array((original_box.points), dtype=np.float64)).get_oriented_bounding_box()
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    point_original = o3d.geometry.PointCloud.crop(points_PCD,original_box_PCD)

    return point_original

def Talker_PCD(pointcloud,num):
    pointcloud = convertCloudFromOpen3dToRos((pointcloud),"cam_1_link")
    pointcloud.is_dense = True   
    pc = pointcloud
    if num == 0:
        pc.header.frame_id = "cam_1_link" 
        pub.publish(pc)  
    if num == 1:
        pc.header.frame_id = "cam_1_link" 
        pub_objects.publish(pc) 
    if num == 2:
        pc.header.frame_id = "cam_1_link" 
        pub_air.publish(pc)  
    if num == 3:
        pc.header.frame_id = "cam_1_link" 
        pub_objects_air.publish(pc) 
    if num == 4:
        pc.header.frame_id = "cam_1_link" 
        pub_clusters.publish(pc) 

def TalkerTrafficLight(traffic_light):
    # print('pre',traffic_light)
    # if traffic_light <= 1:
    #     traffic_light = 1
    # if traffic_light > 1 and traffic_light < 3:
    #     traffic_light = 2
    # if traffic_light >= 3:
    #     traffic_light = 3
    print(traffic_light)
    pub_traffic_light.publish(traffic_light)   

def clusteringObjects(objects):
    clustering = DBSCAN(eps=0.1, min_samples=3).fit(objects)
    labels = clustering.labels_

    num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    if num_clusters == 0:
        return None #, None
    else:
        clusters = [objects[labels == i] for i in range(num_clusters)]
        centers = [np.mean(cluster, axis=0) for cluster in clusters]
        centers_pcd = o3d.geometry.PointCloud()
        centers_pcd.points = o3d.utility.Vector3dVector(np.stack(centers, axis=0))

        return centers_pcd #, boxes_pcd

   
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    """ converts PCD to PointCloud2
    Args:
        points (ndarray): N x 3 point clouds
    Returns:
        PointCloud2
    """ 
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    cloud_data = open3d_cloud.points
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
    x = lenght * np.sin(43.5/180*m.pi)
    y = lenght * np.sin(29.0/180*m.pi)
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
    x = (lenght * np.sin(43.5/180*m.pi))
    y = (lenght * np.sin(29.0/180*m.pi))
    extension = 0
    direction = direction * lenght/5
   
    if (x > 2.1):
        extension = lenght - 2.1 / np.sin(43.5/180*m.pi)
        if direction == 0:
            lenght = 2.1 / np.sin(43.5/180*m.pi)
            x = 2.1
            y = (lenght * np.sin(29.0/180*m.pi))
   
    if (x + abs(direction) > x):
        x = (lenght * np.sin(43.5/180*m.pi)) -abs(direction)
        if x < 1:
            x = (lenght * np.sin(43.5/180*m.pi))
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
    if not isinstance(xyz, np.ndarray) or xyz.dtype != np.float64:
        xyz = np.array(xyz, dtype=np.float64)
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
    pcd= pcd.voxel_down_sample(voxel_size=0.05)
    pcd= o3d.geometry.PointCloud.random_down_sample(pcd,0.9)
    pcd= o3d.geometry.PointCloud.uniform_down_sample(pcd,100)

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
        w, index = PlaneRegression(points_copy, threshold=threshold, init_n=init_n, iter=iterations)
        plane_list.append((w, points_copy[index]))
        points_copy = np.delete(points_copy, index, axis=0)
        index_arr.append(index)

    return plane_list, index_arr

def DrawResult(points,colors):
    pcd = o3d.geometry.PointCloud(points)
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)
        drive_mode = message_filters.Subscriber("/drive_mode", UInt8)
        #vel = message_filters.Subscriber("/lizard/velocity_controller/cmd_vel", Twist)
        #ts = message_filters.ApproximateTimeSynchronizer([data_1, data_2], 1, 1,True)
        #ts.registerCallback(DetectObjectsOnFloor)
        data_1 = message_filters.Subscriber("/cam_1/depth/color/points", PointCloud2)
        data_2 = message_filters.Subscriber("/cam_2/depth/color/points", PointCloud2)
        print('here')
        ts = message_filters.ApproximateTimeSynchronizer([data_1, data_2,drive_mode], 1, 1,True)

        ts.registerCallback(DetectObjects)
        #ts.registerCallback(DetectObjectsOnFloor)
        #ts.registerCallback(DetectObjectsInTheAir)
            
        pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        pub_objects = rospy.Publisher('/pointcloud_objects', PointCloud2, queue_size=10)
        pub_air = rospy.Publisher('/pointcloud_air', PointCloud2, queue_size=10)
        pub_objects_air = rospy.Publisher('/pointcloud_objects_air', PointCloud2, queue_size=10)
        pub_clusters = rospy.Publisher('/pointcloud_clusters', PointCloud2, queue_size=10)
        pub_traffic_light = rospy.Publisher('/traffic_light', Float64, queue_size=10)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
