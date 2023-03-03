#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import math

upper_limit = 2.5

########desired inputs##########
distance_of_camera_from_center_x = 0.35
distance_of_camera_from_center_y = 0.20
rotation_x = 15
rotation_y = 30
###############################

########desired inputs##########
# distance_of_camera_from_center_x = 0.4
# distance_of_camera_from_center_y = 0.22
# rotation_x = 8
# rotation_y = 40
###############################

#############OTTO##############
# distance_of_camera_from_center_x = 0.3
# distance_of_camera_from_center_y = 0.2
# rotation_x = 10
# rotation_y = 10
################################

#############WIDE##############
# distance_of_camera_from_center_x = 0.35
# distance_of_camera_from_center_y = 0.2
# rotation_x = 10
# rotation_y = 35
################################

########centerd cameras#########
# distance_of_camera_from_center_x = 0.05
# distance_of_camera_from_center_y = 0.22
# rotation_x = 5
# rotation_y = 30
###############################

def DrawAMR(center, edgeLength, lenght, r, g, b):
    #points = np.array([[0, -0.05, 0], [-0.05, -0.05, 0], [1, -1, 1], [-1, -1, 1], [0.05, 0.05, 0], [-0.05, 0.05, 0],[1, 1, 1], [-1, 1, 1]], dtype=np.float64)
    lenght =1.63
    x = 0.55
    y = 0.25
    points = np.array([[-x+10, -y-0.25, -lenght], [x+10, -y-0.25, -lenght], [-x+10, -y-0.25, 0],[x+10, -y-0.25, 0], [-x+10, y-0.25, -lenght],[x+10, y-0.25, -lenght],[-x+10, y-0.25, 0], [x+10, y-0.25, 0]], dtype=np.float64)
   
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


def DrawBoxForward(camera_offset_x, camera_offset_y, pitch, yawn, lenght, r, g, b):
    pitch = pitch/180*math.pi
    yawn = yawn/180*math.pi
    y = (lenght * np.tan(43.5/180*math.pi))
    x = (lenght * np.tan(29.0/180*math.pi))

    points = np.array([[0, 0, 0], [0, 0, 0], [x , -y, lenght], [-x, -y, lenght],
                        [0, 0, 0], [0, 0, 0],[x, y, lenght], [-x, y, lenght],
                        [-x, -y, lenght], [x , -y, lenght],[-x, y, lenght], [x , y, lenght ]], dtype=np.float64)
   
    #Euler rotations
    Rx = [[1, 0, 0], [0, np.cos(pitch), np.sin(pitch)], [0, -np.sin(pitch), np.cos(pitch)]]
    Ry = [[np.cos(yawn),0, -np.sin(yawn)], [0, 1, 0], [np.sin(yawn), 0, np.cos(yawn)]]
    for i in range(len(points)):
        R = (np.array(Ry) @ np.array(Rx))
        points[i] = np.dot(R, points[i,:])

    camera_offset_x = camera_offset_x+10
    camera_offset_y = -camera_offset_y
   
    for i in range(len(points)):
        points[i][0] = points[i][0] + camera_offset_x
        points[i][1] = points[i][1] + camera_offset_y

    for i in range(len(points)):
        point = points[i]
        points[i] = np.add(point,0)
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[r, g, b] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    print('Lower end - floor: ',distance_of_camera_from_center_y/np.tan((43.5/180*math.pi)+pitch))
    print('Upper end - 2.5 m: ',upper_limit/np.tan((43.5/180*math.pi)-pitch))
    origin_low = distance_of_camera_from_center_y/np.tan((43.5/180*math.pi)+pitch)
    origin_up = (upper_limit-distance_of_camera_from_center_y)/np.tan((43.5/180*math.pi)-pitch)

    pcd = NumpyToPCD(points)
    return line_set, origin_low, origin_up

AMR_box = DrawAMR(0.5,1,1,r=0, g=0 , b=0)
mesh_box = o3d.geometry.TriangleMesh.create_box(width=30.0, height=0.010, depth=30)
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[10, 0, 3.36])

[right_cam,origin_r_low,origin_r_up] =  DrawBoxForward(-distance_of_camera_from_center_x, distance_of_camera_from_center_y,-rotation_x,-rotation_y,lenght= 6, r=1, g=0 , b=0)
[left_cam,origin_l_low,origin_l_up] =  DrawBoxForward(distance_of_camera_from_center_x,distance_of_camera_from_center_y,-rotation_x,rotation_y,lenght= 6, r=0, g=0 , b=1)
origin_right = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[10, 0, origin_r_low])
origin_left = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[10, 0, origin_l_low])
origin_right_up = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[10, -upper_limit,origin_r_up])
origin_left_up = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[10, -upper_limit, origin_l_up])
o3d.visualization.draw_geometries([AMR_box,left_cam,right_cam,mesh_box,origin_right,origin_left,origin_right_up,origin_left_up])