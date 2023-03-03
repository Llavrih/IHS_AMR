#!/usr/bin/env python
from symbol import tfpdef
import rospy
import open3d as o3d
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
from ctypes import *
import csv
from smartgv_realsense.msg import _CameraTrafficLight as CTL


min_bound = np.ndarray([3, 1])
max_bound = np.ndarray([3, 1])

class TicTocGenerator:
    def __init__(self):
        self.stopwatch = self.generator()

    def generator(self):
        self.ti = 0
        self.tf = time.time()
        while True:
            self.ti = self.tf
            self.tf = time.time()
            yield (1/(self.tf-self.ti))

    def toc(self,tempBool=True):
        tti = next(self.stopwatch)
        if tempBool:
            pass
            # rospy.log("Elapsed frequency: %f Hz.\n" % tti)
        
        return tti

    def tic(self):
        self.toc(False)


class TrafficSupervisor:
    vel = 0
    mode = 0
    # TRAFFIC_GREEN = 3
    TRAFFIC_YELLOW = 2
    TRAFFIC_RED = 1

    def __init__(self):
        rospy.init_node('traffic_supervisor', anonymous=True)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        rospy.Subscriber("/lizard/drive_supervisor/mode", UInt8, self.cb_mode)
        rospy.Subscriber("/lizard/velocity_controller/cmd_vel", Twist, self.cb_vel)
        self.TicToc = TicTocGenerator()
        self.pub_pc = rospy.Publisher('pointcloud', PointCloud2, queue_size=10)
        self.pub_traffic_light = rospy.Publisher('traffic_light', CTL.CameraTrafficLight, queue_size=10)
        # Start the node
        rospy.spin()

    def cb_mode(self, data):
        self.mode = data.data

    def cb_vel(self, data):
        self.vel = data.linear.x


    def callback(self, data):
        self.TicToc.tic()
        pc = ros_numpy.numpify(data)
        points=np.zeros((pc.shape[0],3))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        
        points = (np.array(points, dtype=np.float64))
        '''
        #Rotate camera for 7 degrees
        points_PCD = self.NumpyToPCD(points)
        R = points_PCD.get_rotation_matrix_from_xyz((7 * np.pi / 180, 0, 0))
        points_PCD.rotate(R, center=(0, 0, 0))
        points = self.PCDToNumpy(points_PCD)

        # FILTER L = 6m in Y < 0.2m (kar odreže tla)
        pts2 = []
        for p in points:
            if p[2] < 6 and p[1] < 0.2:
                pts2.append(p)
        points = pts2

        # GET CLOSEST POINT IN XY plane
        distance = 100
        ds = 100
        for p in points:
            ds = np.sqrt(p[0]**2 + p[2]**2)
            if ds < distance:
                distance = ds
        '''

        #Rotate camera for 7 degrees
        points_PCD = self.NumpyToPCD(points)
        R = points_PCD.get_rotation_matrix_from_xyz((7 * np.pi / 180, 0, 0))
        points_PCD.rotate(R, center=(0, 0, 0))
        points =  self.PCDToNumpy(points_PCD)

        original_box = self.DrawBoxAtPoint(0.5,1,lenght=5, r=0, g=1 , b=0.3)
        original_box_PCD =  self.NumpyToPCD(np.array((original_box.points), dtype=np.float64)).get_oriented_bounding_box()
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        point_original =  self.NumpyToPCD(points)
        point_original = o3d.geometry.PointCloud.crop(point_original,original_box_PCD)

        #Escape room for increasing the speed.
        point_original= o3d.geometry.PointCloud.uniform_down_sample(point_original,20)

        downsampled_original_np = ( self.DownSample( self.PCDToNumpy(point_original), voxel_size=0.02))
        downsampled_original =  self.NumpyToPCD(downsampled_original_np)
        (downsampled_original).paint_uniform_color([0, 0.5, 1])
        
        #o3d.visualization.draw_geometries([downsampled_original,origin])
        
        """detect planes on original pointcloud"""
        plane_list, index_arr =  self.DetectMultiPlanes((downsampled_original_np), min_ratio=0.6, threshold=0.08, init_n=3, iterations=50)
        planes = []
        boxes = []
        """find boxes for planes"""
        for _, plane in plane_list:
            box =  self.NumpyToPCD(plane).get_oriented_bounding_box()
            #.get_axis_aligned_bounding_box()
            planes.append(plane)
            #print(box)
            boxes.append(box)
        #print('Planes detected: ',len(boxes))
        planes_np = (np.concatenate(planes, axis=0))
        planes =  self.NumpyToPCD(np.concatenate(planes, axis=0))
        #planes = (NumpyToPCD(DownSample((np.array(planes, dtype=np.float32)), voxel_size=0.01)))
        planes.paint_uniform_color([1,0.8,0.8])
        
        # print(len(index_arr[1]))
        index_arr_new  =[]
        for i in range(len(index_arr)):
            index_arr_new = index_arr_new + index_arr[i]
        #print("index arr new",(index_arr_new))    
        outlier =  o3d.geometry.PointCloud.select_by_index(downsampled_original,index_arr_new,invert=True)
        
    
        """safety zones"""
        load = 0 #2 #0.6
        h = 0.30 + load
        v_max = 2.0
        #v = abs(vel.linear.x)
        v = np.abs(self.vel)
        kf = 0.25
        #print('velocity: ',v)
        direction = self.mode
        direction_translate = [0,0,0,1,-1,1.2,-1.2,-1,1,-1.2,1.2,0,0,0,0,0]
        direction = direction_translate[direction]
        #print('drive_mode: ',direction)
        zone_size = 0.5 + (v/v_max) * kf

        original_box_1 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 1, r=1, g=0 , b=0)
        original_box_2 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 2, r=1, g=0.2 , b=0.1)
        original_box_3 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 3, r=1, g=0.4 , b=0.1)
        original_box_4 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 4, r=1, g=0.6 , b=0.1)
        original_box_5 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 5, r=1, g=0.8 , b=0.1)
        original_box_6 =   self.DrawBoxForward(0.5,1,v,v_max,direction,lenght=zone_size * 6, r=1, g=1 , b=0.1)
        AMR_box =  self.DrawAMR(0.5,1,1,r=0, g=0 , b=0)

        
        outlier =  self.PCDToNumpy(outlier)
        #print(outlier)
        mask = np.isin(outlier[:,:],(planes_np)[:,:], invert=True)
        objects = outlier[mask[:,2]]
        objects =  self.NumpyToPCD(objects)
        objects.paint_uniform_color([0.7, 0.8, 0.2])
        objects.remove_statistical_outlier(nb_neighbors=20,std_ratio=0.01)

        """extract objects from plane in specific zone"""
        distance_arr = [1]
        traffic_light = 6
        original_boxes = [original_box_1,original_box_2,original_box_3,original_box_4,original_box_5,original_box_6]
        for i in range(len(original_boxes)):
            original_box_PCD =  self.NumpyToPCD(np.array((original_boxes[i].points), dtype=np.float64)).get_oriented_bounding_box()
            cropped_box_original = o3d.geometry.PointCloud.crop(objects,original_box_PCD)
            #print('There are: ',len( self.PCDToNumpy(cropped_box_original)),'points in zone ',i+1)
            cropped_box_original_PCD =  self.PCDToNumpy(cropped_box_original)
        
            if len(cropped_box_original_PCD) > 5:
                # print('There are: ',len(cropped_box_original_PCD),'points in zone ',i+1)
                distance_arr =  self.DistanceCalculator(cropped_box_original_PCD)
                object_height = (cropped_box_original_PCD[:,1]*(-1))
               
                for j in range(len(object_height)):
                    if (object_height[j]>-0.55) and (object_height[j] < (0.15 + load)):
                        traffic_light = i+1
                break

            #print('Zone 6. ')
        # print(traffic_light)
        frequency = self.TicToc.toc()
        distance = min(distance_arr)
        self.writeToCSV(distance,self.lenartTrafficLight(traffic_light),self.mode,self.vel,frequency)    
        # print(f"\rDISTANCE: {distance} LIGHT: {self.lenartTrafficLight(traffic_light)} {self.mode} VEL: {self.vel}", end="" )
        ctl_msg = CTL.CameraTrafficLight()
        ctl_msg.traffic_light = self.lenartTrafficLight(traffic_light, True)
        # ctl_msg.timestamp = time.time()
        self.pub_traffic_light.publish(ctl_msg)
        # o3d.visualization.draw_geometries([objects, cropped_box_original, origin,original_box,original_box_1,original_box_2,original_box_3,original_box_4,
        # original_box_5,original_box_6,AMR_box], width=500, height=500, left=50, top=50)
        

    def writeToCSV(self,distance,traffic_light,mode,velocity,frequency):
        with open('/home/ihs-systems/smartgv_realsense.csv', 'a') as file:
            # 2. step
            writer = csv.writer(file)
            # 3. step
            data = ["This", "is", "a", "Test"]
            writer.writerow((distance,traffic_light,mode,velocity,frequency,time.time()))

    def lenartTrafficLight(self,traffic_light,integer=False):
        if traffic_light < 2:
           return 1 if integer else "RED"
        if traffic_light > 1 and traffic_light < 4:
           return 2 if integer else "YELLOW"
        if traffic_light >= 4:
           return 3 if integer else "GREEN"
        

    def getTrafficLight(self, distance):
        if distance > self.TRAFFIC_YELLOW:
            return "GREEN"
        elif distance > self.TRAFFIC_RED and distance < self.TRAFFIC_YELLOW:
            return "YELLOW"
        elif distance < self.TRAFFIC_RED:
            return "RED"


    def DistanceCalculator(self, arr):
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


    def PCDToNumpy(self, pcd):
        """  convert open3D point cloud to numpy ndarray
        Args:
            pcd (open3d.geometry.PointCloud):
        Returns:
            [ndarray]:
        """

        return np.asarray(pcd.points)


    def NumpyToPCD(self, xyz):
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

    def RemoveNan(self, points):
        """ remove nan value of point clouds
        Args:
            points (ndarray): N x 3 point clouds
        Returns:
            [ndarray]: N x 3 point clouds
        """

        return points[~np.isnan(points[:, 0])]

    def RemoveNoiseStatistical(self, pc, nb_neighbors, std_ratio):
        """ remove point clouds noise using statitical noise removal method
        Args:
            pc (ndarray): N x 3 point clouds
            nb_neighbors (int, optional): Defaults to 20.
            std_ratio (float, optional): Defaults to 2.0.
        Returns:
            [ndarray]: N x 3 point clouds
        """

        pcd = self.NumpyToPCD(pc)
        cl, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors, std_ratio=std_ratio)

        return self.PCDToNumpy(cl)

    def DownSample(self, pts, voxel_size):
        """ down sample the point clouds
        Args:
            pts (ndarray): N x 3 input point clouds
            voxel_size (float, optional): voxel size. Defaults to 0.003.
        Returns:
            [ndarray]:
        """

        p = self.NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

        return self.PCDToNumpy(p)

    def PlaneRegression(self, points, threshold, init_n, iter):
        """ plane regression using ransac
        Args:
            points (ndarray): N x3 point clouds
            threshold (float, optional): distance threshold. Defaults to 0.003.
            init_n (int, optional): Number of initial points to be considered inliers in each iteration
            iter (int, optional): number of iteration. Defaults to 1000.
        Returns:
            [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
        """

        pcd = self.NumpyToPCD(points)

        w, index = pcd.segment_plane(threshold, init_n, iter)
        #outlier = pcd.select_by_index(index,invert=True)
        return w, index

    def DetectMultiPlanes(self, points, min_ratio, threshold, init_n, iterations):
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
            w, index = self.PlaneRegression(
                target, threshold, init_n, iterations)
    
            count += len(index)
            plane_list.append((w, target[index]))
            target = np.delete(target, index, axis=0)
            index_arr.append(index)

        return plane_list, index_arr


    def DrawBoxAtPoint(self, center, edgeLength, lenght, r, g, b):
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

    def DrawAMR(self, center, edgeLength, lenght, r, g, b):
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

    def DrawBoxForward(self,center, edgeLength,v,v_max,direction, lenght, r, g, b):
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









if __name__ == '__main__':
    try:
        TrafSup = TrafficSupervisor()
    except rospy.ROSInterruptException as err:
        print(err)
