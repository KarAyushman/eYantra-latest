#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf2_geometry_msgs
from tf_transformations import *
from tf2_ros import TransformException
from os import path
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
import time
##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(topl, topr, bottomr, bottoml):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.
    width = math.dist(topl, topr)
    area = abs((topl[0] * topr[1] + topr[0] * bottomr[1] + bottomr[0] * bottoml[1] + bottoml[0] * topl[1]) - (topl[1] * topr[0] + topr[1] * bottomr[0] + bottomr[1] * bottoml[0] + bottoml[1] * topl[0])) / 2.0
    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 
    arucoParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, idtemp, rejected) = detector.detectMarkers(grayImage)
    #   ->  Draw detected marker on the image frame which will be shown later
    cv2.aruco.drawDetectedMarkers(image, corners, idtemp)
    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
    if len(corners) > 0:
        idtemp = idtemp.flatten()
        for i in range(len(idtemp)):
            coordinates = corners[i][0]
            topLeft = (int(coordinates[0][0]), int(coordinates[0][1]))
            topRight = (int(coordinates[1][0]), int(coordinates[1][1]))
            bottomRight = (int(coordinates[2][0]), int(coordinates[2][1]))
            bottomLeft = (int(coordinates[3][0]), int(coordinates[3][1]))
            area, width = calculate_rectangle_area(topLeft, topRight, bottomRight, bottomLeft)
    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined
            if(area<aruco_area_threshold):
                continue
    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #   ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            center = (cX, cY)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i],size_of_aruco_m,cam_mat,dist_mat)
            transform_translation_x = tvec[0][0][0]
            transform_translation_y = tvec[0][0][1]
            transform_translation_z = tvec[0][0][2]
            trans_mat = [transform_translation_x, transform_translation_y, transform_translation_z]
            # print(trans_mat)
            rot_mat = np.eye(4)
            rot_mat[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0]))[0]
            r = R.from_matrix(rot_mat[0:3, 0:3])
            eul = r.as_euler('xyz', degrees=False)
            center_aruco_list.append(center)
            distance_from_rgb_list.append(trans_mat)
            angle_aruco_list.append(eul)
            width_aruco_list.append(width)
            ids.append(idtemp[i])
    #   ->  Draw frame axes from coordinates received using pose estimation
#       ->  HINT: You may use 'cv2.drawFrameAxes'
            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m)

    cv2.imwrite('./Detected_Aruco_Markers.png', image)
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

    ############################################

##################### CLASS DEFINITION #######################

def get_quaternion_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    # print (qx, qy, qz, qw)
    
    return [qx, qy, qz, qw]

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

class aruco_tf(Node):
    
    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')

        ############ Topic SUBSCRIPTIONS ############
        
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)
        # self.timer = self.create_timer(60, self.move())
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())



    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type
        self.depth_image =  self.bridge.imgmsg_to_cv2(data, desired_encoding= 'passthrough')

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
    

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
                node=self,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # moveit2.add_collision_mesh(filepath="~/eYantra/src/pymoveit2/examples/assets/rack1.stl", id="haha", position=[0.55, 0.06, -0.58], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id=ur5.base_link_name())
        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            
        
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids_aruco_list = detect_aruco(self.cv_image)
        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 
        boxes = list()
        for i in range(len(ids_aruco_list)):
        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)
            # angle_aruco_list[i][2] = (angle_aruco_list[i][2]*0.788) - ((angle_aruco_list[i][2]**2)/3160)
        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
            roll = 0.0#+1.57
            pitch = 0.0#-1.57
            yaw = angle_aruco_list[i][2]#+1.57
        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)
            cX, cY = center_aruco_list[i]
            cX = cX/1000
            cY = cY/1000
        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above
            x = distance_from_rgb_list[i][0]# * (sizeCamX - cX - centerCamX) / focalX
            y = distance_from_rgb_list[i][1]# * (sizeCamY - cY - centerCamY) / focalY
            z = distance_from_rgb_list[i][2]
            quat = quaternion_from_euler(roll, pitch, yaw)
            newquat = R.from_quat(quat).as_quat()
            newquat[2] = -newquat[2]
            quat = newquat
            newquat = R.from_quat(quat).as_euler('xyz')
            newquat[2] = newquat[2]+1.57
            # newquat[1] = newquat[1]+1.57
            newquat[0] = newquat[0]+1.57
            newquat = R.from_euler('xyz',newquat)
            rot_mat = np.array([[0.9659258,  0.0000000, -0.2588190],[0,1,0],[0.2588190,  0.0000000,  0.9659258]])
            quat = R.from_matrix(np.dot(rot_mat, newquat.as_matrix())).as_quat()
            # quat = R.from_euler('xyz',newquat).as_quat()
        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 
            cv2.circle(self.cv_image, center_aruco_list[i], 10, (255, 0, 0), 2)
        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID
            frame_id = 'camera_link'
            child_frame_id = 'cam_{}'.format(ids_aruco_list[i])
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = frame_id
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = z
            transform.transform.translation.y = -x
            transform.transform.translation.z = -y
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            orn = R.from_quat(quat).as_euler('xyz')
            self.br.sendTransform(transform)
            # print(child_frame_id)

            try:
                trans = self.tf_buffer.lookup_transform('cam_{}'.format(ids_aruco_list[i]), 'base_link', rclpy.time.Time())
            except TransformException as e:
                # self.get_logger().info(f'Could not transform base to obj: {e}')
                continue

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker I
            frame_id = 'base_link'
            child_frame_id = 'obj_{}'.format(ids_aruco_list[i])
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = frame_id
            transform.child_frame_id = child_frame_id
            if(orn[2]>-0.785 and orn[2]<0.785):
                transform.transform.translation.x = -trans.transform.translation.x
                transform.transform.translation.y = trans.transform.translation.z
                transform.transform.translation.z = -trans.transform.translation.y
            elif(orn[2]>=0.785 and orn[2]<2.35):
                transform.transform.translation.x = -trans.transform.translation.z
                transform.transform.translation.y = -trans.transform.translation.x
                transform.transform.translation.z = -trans.transform.translation.y
            else:
                print(ids_aruco_list[i], orn[2])    
                transform.transform.translation.x = trans.transform.translation.x
                transform.transform.translation.y = -trans.transform.translation.z
                transform.transform.translation.z = -trans.transform.translation.y
            transform.transform.rotation.x = -trans.transform.rotation.x
            transform.transform.rotation.y = -trans.transform.rotation.y
            transform.transform.rotation.z = -trans.transform.rotation.z
            transform.transform.rotation.w = trans.transform.rotation.w
            p = [-trans.transform.translation.x, trans.transform.translation.z, -trans.transform.translation.y]
            q = [-trans.transform.rotation.x, -trans.transform.rotation.y, -trans.transform.rotation.z, trans.transform.rotation.w]
            self.br.sendTransform(transform)
            # if (ids_aruco_list[i] not in x[0] for x in box):
            #     box.
            # moveit2.move_to_pose(position=p1, quat_xyzw=q1, cartesian=False)
            # moveit2.wait_until_executed()
            packet = [i, p, q]
            boxes.append(packet)
        
        # i=0
        # for i in range(3):
        #     moveit2.add_collision_mesh(
        #         filepath="~/eYantra/src/pymoveit2/examples/assets/rack1.stl", id="rack1", position=[0.55, 0.06, -0.58], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id=ur5.base_link_name()
        #     )
        #     i=i+1
        # i=0
        # for i in range(3):
        #     moveit2.add_collision_mesh(
        #         filepath="~/eYantra/src/pymoveit2/examples/assets/rack2.stl", id="rack2", position=[0.25, -0.64, -0.58], quat_xyzw=[0.0, 0.0, -0.707, 0.707], frame_id=ur5.base_link_name()
        #     )
        #     i=i+1 

        Dp = [-0.37, 0.12, 0.397]
        Dq = [-0.50, 0.50, 0.50, -0.50]

        # moveit2.add_collision_mesh(
        # filepath="~/eYantra/src/pymoveit2/examples/assets/rack1.stl", id="rack1", position=position1, quat_xyzw=quat_xyzw1, frame_id=ur5.base_link_name()
        #     )

        # gripperA_control = self.create_client(AttachLink, '/GripperMagnetON')
        # gripperD_control = self.create_client(AttachLink, '/GripperMagnetOFF')

        # while not gripperA_control.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('EEF service not available, waiting again...')
        # while not gripperD_control.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('EEF service not available, waiting again...')

        
        for i in boxes:
            moveit2.move_to_pose(position=i[1], quat_xyzw=i[2], cartesian=False)
            moveit2.wait_until_executed()
            print("got box")
            time.sleep(30)
            # req = AttachLink.Request()
            # req.model1_name =  'box_{}'.format(boxes[i])  
            # req.link1_name  = 'link'       
            # req.model2_name =  'ur5'       
            # req.link2_name  = 'wrist_3_link'  
            # print("Attching ", req._model1_name)
            # gripperA_control.call_async(req)
            moveit2.move_to_pose(position=Dp, quat_xyzw=Dq, cartesian=False)
            moveit2.wait_until_executed()
            print("drop box")
            # gripperD_control.call_async(req)
            boxes.remove(i)
        
        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 
            
        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################

##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information
    
    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()