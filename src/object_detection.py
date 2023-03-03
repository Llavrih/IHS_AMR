#!/usr/bin/env python
import rospy
import open3d as o3d
#from open3d.geometry import crop_point_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import math


min_bound = np.ndarray([3, 1])
max_bound = np.ndarray([3, 1])

def callback(data):
    tic()
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points = (np.array(points, dtype=np.float64))

    original_box = DrawBoxAtPoint(0.5,1,lenght=4, r=0, g=1 , b=0.3)
    original_box_PCD = NumpyToPCD(np.array((original_box.points), dtype=np.float32)).get_oriented_bounding_box()
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
   
    point_original = NumpyToPCD(points)
    #print(point_original)
    point_original = o3d.geometry.PointCloud.crop(point_original,original_box_PCD)
    downsampled_original = NumpyToPCD(DownSample(PCDToNumpy(point_original), voxel_size=0.02))
    (downsampled_original).paint_uniform_color([0, 0.5, 1])
    #o3d.visualization.draw_geometries([downsampled_original,origin])

    """detect planes on original pointcloud"""
    plane_list, index_arr = DetectMultiPlanes(PCDToNumpy(downsampled_original), min_ratio=0.2, threshold=0.05, init_n=3, iterations=100)
    #print("Una areja", index_arr)
    planes = []
    boxes = []
    """find boxes for planes"""
    for _, plane in plane_list:
        box = NumpyToPCD(plane).get_oriented_bounding_box()
        #.get_axis_aligned_bounding_box()
        planes.append(plane)
        boxes.append(box)
    print('Planes detected: ',len(boxes))
   
    planes = NumpyToPCD(np.concatenate(planes, axis=0))
    #planes = (NumpyToPCD(DownSample((np.array(planes, dtype=np.float32)), voxel_size=0.01)))
    planes.paint_uniform_color([1,0.8,0.8])
   
   # print(len(index_arr[1]))
    index_arr_new  =[]
    for i in range(len(index_arr)):
        index_arr_new = index_arr_new + index_arr[i]
    #print("index arr new",(index_arr_new))    
    outlier =  o3d.geometry.PointCloud.select_by_index(downsampled_original,index_arr_new,invert=True)
       
   

    """safety zones"""
    load = 0
    h = 0.60 + load
    v_max = 0.3
    v = 0.3

    zone_size = 1
    direction = 1
   
    original_box_1 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 1, r=1, g=0 , b=0)
    original_box_2 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 2, r=1, g=0.2 , b=0.1)
    original_box_3 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 3, r=1, g=0.4 , b=0.1)
    original_box_4 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 4, r=1, g=0.6 , b=0.1)
    original_box_5 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 5, r=1, g=0.8 , b=0.1)
    original_box_6 =  DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 6, r=1, g=1 , b=0.1)
    AMR_box = DrawAMR(0.5,1,1,r=0, g=0 , b=0)
    """extract objects from plane in specific zone"""
    original_box_PCD = NumpyToPCD(np.array((original_box_6.points), dtype=np.float32)).get_oriented_bounding_box()
    cropped_box_original = o3d.geometry.PointCloud.crop(downsampled_original,original_box_PCD)
   

    """joint outliers and planes"""

    outlier = PCDToNumpy(outlier)
    #print(outlier)
    mask = np.isin(outlier[:,:],PCDToNumpy(planes)[:,:], invert=True)
    objects = outlier[mask[:,2]]
    objects = NumpyToPCD(objects)
    objects.paint_uniform_color([0.7, 0.8, 0.2])
    #print(objects)

    toc()
    o3d.visualization.draw_geometries([objects,planes,
    origin,original_box,original_box_1,original_box_2,original_box_3,original_box_4,
    original_box_5,original_box_6,AMR_box], width=1000, height=1000, left=50, top=50)

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
    lenght = lenght 
    if x > 2.1:
        x = 2.1
    print('x: ',x,'y: ',y)
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

def TicTocGenerator():
    # Generator that returns time differences
    ti = 0           # initial time
    tf = time.time() # final time
    while True:
        ti = tf
        tf = time.time()
        yield tf-ti # returns the time difference

TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# This will be the main function through which we define both tic() and toc()
def toc(tempBool=True):
    # Prints the time difference yielded by generator instance TicToc
    tempTimeInterval = next(TicToc)
    if tempBool:
        print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

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

def RemoveNoiseStatistical(pc, nb_neighbors, std_ratio):
    """ remove point clouds noise using statitical noise removal method
    Args:
        pc (ndarray): N x 3 point clouds
        nb_neighbors (int, optional): Defaults to 20.
        std_ratio (float, optional): Defaults to 2.0.
    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = NumpyToPCD(pc)
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return PCDToNumpy(cl)

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
    """ plane regression using ransac
    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.
    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    w, index = pcd.segment_plane(threshold, init_n, iter)
    #outlier = pcd.select_by_index(index,invert=True)
    return w, index

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
    target = points.copy()
    count = 0
    index_arr = []


    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold, init_n, iterations)
   
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)
        index_arr.append(index)

    return plane_list, index_arr


def DrawResult(points,colors):
    pcd = o3d.geometry.PointCloud(points)
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
rospy.spin()
