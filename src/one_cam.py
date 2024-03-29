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
import concurrent.futures
from geometry_msgs.msg import Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray

global objects_detected 
object_detected = True
# from memory_profiler import profile
# @profile
def DetectObjects(data_1,drive_mode):
    global objects_detected 
    object_detected = False
    print('_______________________')
    print('Time od calling DetectObjects: ',time.time())
    start_time = time.time()
    load = 0.5
    point_original = combinePCD(data_1)
    print('PCD1 + PCD2: {}'.format(time.time()-start_time))
    
 
    def DetectObjectsOnFloor(point_original, drive_mode,load):
        start_time1 = time.time()
        #print('1: {}'.format(time_0))
        # Escape room for increasing the speed.
        original_box = DrawBoxAtPoint(0.5,1,lenght=1.5 + 0.4, r=0, g=1 , b=0.3)
        original_box_PCD = NumpyToPCD(np.array((original_box.points), dtype=np.float64)).get_oriented_bounding_box()
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        point_original = o3d.geometry.PointCloud.crop(point_original,original_box_PCD)
        point_cloud = PCDToNumpy(point_original)
        mask = point_cloud[:, 2] <= 0.1
        point_cloud_floor = point_cloud[mask]
        point_cloud_floor_pcd = NumpyToPCD(point_cloud_floor)
        #print('1: {}'.format(time.time()))
        #print('2: {}'.format(time.time()-time_0))
        plane_list, index_arr = DetectMultiPlanes((point_cloud_floor), min_ratio=0.5, threshold=0.005, init_n=3, iterations=100)

        #print('3: {}'.format(time.time()-time_0))
        planes_np = []
        boxes = []

        for _, plane in plane_list:
            box = NumpyToPCD(plane).get_oriented_bounding_box()
            boxes.append(box)
            planes_np.append(plane)

        planes_np = np.concatenate(planes_np, axis=0)
        #print('4: {}'.format(time.time()-time_0))
        index_arr_new = np.concatenate(index_arr, axis=0)
        outlier = o3d.geometry.PointCloud.select_by_index(point_cloud_floor_pcd, index_arr_new, invert=True)
        outlier_np = PCDToNumpy(outlier)

        planes_mask = ~np.isin(outlier_np, planes_np).any(axis=1)
        objects = outlier_np[planes_mask]

        cl_arr = o3d.geometry.PointCloud()
        radii = np.array([0.002 + 0.002 * i for i in range(0, 4, 1)])
        nb_points = np.array([8 + 1 * i for i in range(0, 4, 1)])
        distance_cut = np.array([i * 1 for i in range(0, 4, 1)])
        #print('5: {}'.format(time.time()-time_0))
        for i in range(4):
            distance_mask = abs(objects[:, 0]) <= distance_cut[i]
            objects_cut = objects[distance_mask]
            if len(objects_cut) > 0:
                cl = o3d.geometry.PointCloud()
                cl.points = o3d.utility.Vector3dVector(objects_cut)
                cl, _ = cl.remove_radius_outlier(nb_points=nb_points[i], radius=radii[i])
                cl_arr += cl
        #print('6: {}'.format(time.time()-time_0))
        objects_viz = cl_arr
        objects_viz_np = PCDToNumpy(objects_viz)
        if np.size(objects_viz_np) > 5:
            #centers_pcd = clusteringObjects(objects_viz_np)
            
            traffic_light_floor = DetectTraffic(objects_viz_np, load, drive_mode)
            #TalkerTrafficLight(min(traffic_light_floor),1)
            
            #Talker_PCD(centers_pcd, 4)
            Talker_PCD(objects_viz, 1)
        else:
            traffic_light_floor = [6]
            #TalkerTrafficLight(min(traffic_light_floor),1)

        Talker_PCD(point_cloud_floor_pcd, 0)
        #print('7: {}'.format(time.time()-time_0))
        print('Time for detecting objects on floor: {} '.format(time.time() - start_time1))
        return traffic_light_floor    

    def DetectObjectsInTheAir(point_original,drive_mode,load):
        start_time2 = time.time()
        traffic_light_up = []
        point_cloud = PCDToNumpy(point_original)
        mask = point_cloud[:, 2] >= 0.1
        point_cloud_up = point_cloud[mask]
        point_cloud_up = (DownSample((point_cloud_up),0.01))
        point_cloud_up_pcd = NumpyToPCD(point_cloud_up)
        point_cloud_up_pcd= o3d.geometry.PointCloud.random_down_sample(point_cloud_up_pcd,0.9)
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
        traffic_light_up = DetectTraffic(objects,load,drive_mode)
        Talker_PCD(point_cloud_up_pcd,2)
        Talker_PCD(objects_viz,3)
        #TalkerTrafficLight(min(traffic_light_up),0)
        print('Time for detecting objects in the air: {} '.format(time.time() - start_time2))
        return traffic_light_up

    
    try:
        

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            
            future1 = executor.submit(DetectObjectsOnFloor, point_original, drive_mode, load)
            future2 = executor.submit(DetectObjectsInTheAir, point_original, drive_mode, load)

            traffic_light_floor = future1.result()
            traffic_light_up = future2.result()
            TalkerTrafficLight(min(min(traffic_light_floor),min(traffic_light_up)))
            object_detected = True
            print('***********************')
            print('Stop', time.time()-start_time)
            pub_stop_time.publish(1/(time.time()-start_time))
            print('***********************')
            print('_______________________')

    except Exception as e:
        object_detected = True
        print("Error: unable to start thread")
        print(e)


def IsPointInsidePoly(x, y, polygon):
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        x_intersection = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= x_intersection:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside
def CreateMarker(polygon,i,rgb):
    
    marker = Marker()
    marker.header.frame_id = "cam_1_link"
    marker.id = i
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.color.a = 1.0
    marker.color.r = rgb[0]
    marker.color.g = rgb[1]
    marker.color.b = rgb[2]
    marker.pose.orientation.w = 1.0

    for point in polygon:
        p = Point32()
        p.x = point[0]
        p.y = point[1]
        marker.points.append(p)
    
    marker.points.append(marker.points[0])  # Close the polygon
    return marker

def CallCreateMarker(drive_mode):
    drive_mode = np.array(drive_mode.data)
    for i in range(6):
        marker_array = MarkerArray()
        marker = CreateMarker(drive_zones[drive_mode][i], i, [0,0,1])
        marker_array.markers.append(marker)
        polygon_pub.publish(marker_array)

global drive_zones
drive_zones = np.array([[[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.68, 1.3], [-1.12, 0.6], [-0.63, -0.24], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.39, 2.14], [-1.45, 1.0], [-0.63, -0.38], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [0.05, 2.7], [-1.85, 1.3], [-0.63, -0.6], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.17, 3.1], [-2.45, 1.5], [-0.63, -0.9], [-0.35, 0.0]], [[-0.35, 0.0], [0.95, 0.0], [0.95, 0.64], [-0.32, 3.7], [-3.05, 2.0], [-0.63, -1.1], [-0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.68, 1.3], [1.12, 0.6], [0.63, -0.24], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.39, 2.14], [1.45, 1.0], [0.63, -0.38], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [-0.05, 2.7], [1.85, 1.3], [0.63, -0.6], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.17, 3.1], [2.45, 1.5], [0.63, -0.9], [0.35, 0.0]], [[0.35, 0.0], [-0.95, 0.0], [-0.95, 0.64], [0.32, 3.7], [3.05, 2.0], [0.63, -1.1], [0.35, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]], [[[0.0, 0.0], [0.8, 0.0], [0.8, 0.6], [-0.8, 0.6], [-0.8, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.85, 0.0], [0.85, 1.0], [-0.85, 1.0], [-0.85, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.7], [-1.0, 1.7], [-1.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.1, 0.0], [1.1, 2.5], [-1.1, 2.5], [-1.1, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.2, 0.0], [1.2, 3.2], [-1.2, 3.2], [-1.2, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.25, 0.0], [1.25, 3.8], [-1.25, 3.8], [-1.3, 0.0], [0.0, 0.0]]]],dtype=object)
        
def DetectTraffic(objects,load,drive_mode):
    global drive_zones
    drive_mode = np.array(drive_mode.data)
    object_in_zone = []  
    if len(objects) == 0:
        return [6]
    else: 
        for obj in objects:
            for i in range(6):
                if IsPointInsidePoly(obj[0],(obj[1]),drive_zones[drive_mode][i]) == True:
                    
                    #marker_array = MarkerArray()                        
                    # marker = CreateMarker(drive_zones[drive_mode][i], i, [1,0,0])
                    # marker_array.markers.append(marker)
                    # active_polygon_pub.publish(marker_array)

                    object_in_zone.append(i)
                
        if len(object_in_zone) == 0:
                    return [6]
        else:                  
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

import cProfile
import pstats
from io import StringIO
import time


def combinePCD(data_1):
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

    # br_left = tf2_ros.TransformBroadcaster()
    # transform_stamped = TransformStamped()
    # transform_stamped.header.stamp = rospy.Time.now()
    # transform_stamped.header.frame_id = "cam_1_link"
    # transform_stamped.child_frame_id = "cam_left_link"
    # transform_stamped.transform.translation.x = -x_translation
    # transform_stamped.transform.translation.y = z_translation
    # transform_stamped.transform.translation.z = -y_translation

    # # Assuming you have roll, pitch, and yaw angles in radians
    # roll = np.deg2rad(-x_rot)
    # pitch = np.deg2rad(y_rot)
    # yaw = np.deg2rad(z_rot)
    # quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

    # transform_stamped.transform.rotation.x = quaternion[0]
    # transform_stamped.transform.rotation.y = quaternion[1]
    # transform_stamped.transform.rotation.z = quaternion[2]
    # transform_stamped.transform.rotation.w = quaternion[3]

    # br_left.sendTransform(transform_stamped)

    # br_right = tf2_ros.TransformBroadcaster()
    # transform_stamped = TransformStamped()
    # transform_stamped.header.stamp = rospy.Time.now()
    # transform_stamped.header.frame_id = "cam_1_link"
    # transform_stamped.child_frame_id = "cam_right_link"
    # transform_stamped.transform.translation.x = x_translation 
    # transform_stamped.transform.translation.y = z_translation
    # transform_stamped.transform.translation.z = -y_translation

    # # Assuming you have roll, pitch, and yaw angles in radians
    # roll = np.deg2rad(x_rot)
    # pitch = np.deg2rad(y_rot)
    # yaw = np.deg2rad(z_rot)
    # quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

    # transform_stamped.transform.rotation.x = quaternion[0]
    # transform_stamped.transform.rotation.y = quaternion[1]
    # transform_stamped.transform.rotation.z = quaternion[2]
    # transform_stamped.transform.rotation.w = quaternion[3]

    # br_right.sendTransform(transform_stamped)

    def downsample_pointcloud(points, downsample_ratio):
        num_points = points.shape[0]
        num_downsampled_points = int(num_points * downsample_ratio)

        # Generate random indices
        random_indices = np.random.choice(num_points, num_downsampled_points, replace=False)

        # Extract points using the random indices
        downsampled_points = points[random_indices]

        return downsampled_points

    
    def processData1(data_1):
        def inner_process_data():
            start_time = time.time()
            start_time_all = time.time()
            pc1 = ros_numpy.numpify(data_1)
            # print('Size of data: ', np.size(pc1))
            # print('1: {}'.format(time.time() - start_time))
            # start_time = time.time()

            points1 = np.zeros((pc1.shape[0], 3), dtype=np.float64)
            # print('2: {}'.format(time.time() - start_time))
            # start_time = time.time()
            translation = [-x_translation, y_translation, z_translation]
            # print('3: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points1 = np.vstack((pc1['x'], pc1['y'], pc1['z'])).T

            # print('4: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points_PCD1 = NumpyToPCD(points1)
            # print('5: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points_PCD1 = o3d.geometry.PointCloud.random_down_sample(points_PCD1, 0.8)
            # print('6: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points_PCD1 = o3d.geometry.PointCloud.uniform_down_sample(points_PCD1, 6)
            # print('7: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points_PCD1.translate(translation)
            # print('8: {}'.format(time.time() - start_time))
            # start_time = time.time()
            R1 = np.array(Rz(z_rot) * Rx(-x_rot) * Ry(y_rot))
            # print('9: {}'.format(time.time() - start_time))
            # start_time = time.time()
            points_PCD1.rotate(R1, center=(translation))
            # print('10: {}'.format(time.time() - start_time))
            print('PCD1: {}'.format(time.time() - start_time_all))

            return points_PCD1

        # Create a profile object
        pr = cProfile.Profile()

        # Profile your function
        pr.enable()
        result = inner_process_data()
        pr.disable()

        # Store the profiling results in a StringIO object
        s = StringIO()
        sortby = 'cumulative'
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)

        # Print the profiling results
        ps.print_stats()
        print(s.getvalue())

        return result


    # Define a function that will run in a separate thread to process data_2
    def processData2(data_2):
        start_time_all = time.time()
        pc2 = ros_numpy.numpify(data_2)
        translation = [x_translation, y_translation, z_translation]
        points2 = np.vstack((pc2['x'], pc2['y'], pc2['z'])).T
        points_PCD2 = NumpyToPCD(points2)
        points_PCD2= o3d.geometry.PointCloud.random_down_sample(points_PCD2,0.8)
        points_PCD2= o3d.geometry.PointCloud.uniform_down_sample(points_PCD2,6)
        points_PCD2.translate(translation)
        R2 = np.array(Rz(z_rot) * Rx(x_rot) * Ry(y_rot))
        points_PCD2.rotate(R2, center=(translation))
        print('PCD2: {}'.format(time.time()-start_time_all))
        return points_PCD2


    # Start two threads to process data_1 and data_2 simultaneously
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        # Submit the tasks and collect the future objects
        future1 = executor.submit(processData1, data_1)
        #future2 = executor.submit(processData2, data_2)
        # Wait for the tasks to complete and get the results
        points_PCD1 = future1.result()
        #points_PCD2 = future2.result()

    points_PCD = points_PCD1# + points_PCD2
    # points_PCD= o3d.geometry.PointCloud.random_down_sample(points_PCD,0.7)
    # points_PCD= o3d.geometry.PointCloud.uniform_down_sample(points_PCD,5)
    R = points_PCD.get_rotation_matrix_from_xyz((np.deg2rad(-75), np.deg2rad(0), np.deg2rad(0)))
    points_PCD.rotate(R, center=(0,0,0))
    #points_PCD.translate((0,-0.4,0))

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
    print('Traffic light: {}'.format(traffic_light))
    print('Time for traffic: {}'.format(time.time()))
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
    pcd.points = o3d.utility.Vector3dVector(points) 

    _, inliers = pcd.segment_plane(distance_threshold=threshold,
                                   ransac_n=init_n,
                                   num_iterations=iter)

    inlier_points = np.asarray(pcd.select_by_index(inliers).points)

    A = np.vstack((inlier_points[:, :2].T, np.ones(len(inlier_points)))).T
    w = np.linalg.lstsq(A, inlier_points[:, 2], rcond=None)[0]

    return w, inliers

def DetectMultiPlanes(points, min_ratio, threshold, init_n, iterations):
    plane_list = []
    N = len(points)
    points_copy = points.copy()
    index_arr = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        while len(points_copy) > min_ratio * N:
            future = executor.submit(PlaneRegression, points_copy, threshold=threshold, init_n=init_n, iter=iterations)
            w, index = future.result()
            plane_list.append((w, points_copy[index]))
            points_copy = np.delete(points_copy, index, axis=0)
            index_arr.append(index)

    return plane_list, index_arr


def DataCheck(data_1,drive_mode):
    print('+++++++++++++++++++++++')
    print('      DATA READY')
    print('TIME: {}'.format(time.time()))
    print('+++++++++++++++++++++++')
    global object_detected
    if object_detected == True:
        DetectObjects(data_1,drive_mode)
   

if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)

        print("Waiting for /drive_mode topic...")
        drive_mode_msg = rospy.wait_for_message("/drive_mode", UInt8)

        print("Waiting for /cam_1/depth/color/points topic...")
        data_1_msg = rospy.wait_for_message("/cam_1/depth/color/points", PointCloud2)

        print("Waiting for /cam_2/depth/color/points topic...")
        data_2_msg = rospy.wait_for_message("/cam_2/depth/color/points", PointCloud2)

        drive_mode = message_filters.Subscriber("/drive_mode", UInt8)
        data_1 = message_filters.Subscriber("/cam_1/depth/color/points", PointCloud2)
        #data_2 = message_filters.Subscriber("/cam_2/depth/color/points", PointCloud2)
        print("Started the program.")
        queue_size = 10 # Adjust queue_size to control the rate of synchronization
        slop = 1  # Adjust slop to control the tolerance for the time difference between messages
        
        ts = message_filters.ApproximateTimeSynchronizer([data_1, drive_mode], queue_size, slop, allow_headerless=True)
        ts.registerCallback(DataCheck)
        
  
        #ts.registerCallback(DetectObjects)

        #ts.registerCallback(DetectObjectsOnFloor)
        #ts.registerCallback(DetectObjectsInTheAir)

        pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        pub_objects = rospy.Publisher('/pointcloud_objects', PointCloud2, queue_size=10)
        pub_air = rospy.Publisher('/pointcloud_air', PointCloud2, queue_size=10)
        pub_objects_air = rospy.Publisher('/pointcloud_objects_air', PointCloud2, queue_size=10)
        pub_clusters = rospy.Publisher('/pointcloud_clusters', PointCloud2, queue_size=10)
        pub_traffic_light = rospy.Publisher('/traffic_light', Float64, queue_size=10)
        polygon_pub = rospy.Publisher("/polygons", MarkerArray, queue_size=1)
        active_polygon_pub = rospy.Publisher("/active_polygons", MarkerArray, queue_size=1)
        pub_stop_time = rospy.Publisher('/realsense_freq', Float64, queue_size=10)

        rospy.Subscriber("/drive_mode",UInt8,CallCreateMarker)
            

   
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


