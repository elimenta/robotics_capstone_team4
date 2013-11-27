#!/usr/bin/env python

import roslib
roslib.load_manifest('object_manipulation_msgs')
roslib.load_manifest('tabletop_object_detector')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('arm_navigation_msgs')
roslib.load_manifest('tabletop_collision_map_processing')


import rospy
from object_manipulation_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal, GripperTranslation
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import ManipulationResult
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from arm_navigation_msgs.msg import LinkPadding
import tf
import actionlib
import scipy
import time
import copy
import math
import pdb
import threading
import sys
import tabletop_collision_map_processing.collision_map_interface as collision_map_interface
import object_manipulator.draw_functions as draw_functions
from object_manipulator.convert_functions import *
from arm_navigation_msgs.msg import JointConstraint, PositionConstraint, OrientationConstraint, ArmNavigationErrorCodes, AllowedContactSpecification, Constraints
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from tf_conversions import posemath
from tf import TransformListener
from room_navigator import *

class PickAndPlaceManager():
    def __init__(self, tf_listener, roomNav):

        # Services used for object detection
        self.grasper_detect_name = 'object_detection'
        self.collision_map_processing_name = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        self.find_bounding_box_name = '/find_cluster_bounding_box'

        rospy.loginfo("grasp_executive: waiting for object_detection service")
        rospy.wait_for_service(self.grasper_detect_name)
        rospy.loginfo("grasp_executive: object_detection service found")

        rospy.loginfo("grasp_executive: waiting for collision_map_processing service")
        rospy.wait_for_service(self.collision_map_processing_name)
        rospy.loginfo("grasp_executive: collision_map_processing service found")

        rospy.loginfo("grasp_executive: waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(self.find_bounding_box_name)
        rospy.loginfo("grasp_executive: find_cluster_bounding_box service found")

        self.grasper_detect_srv = rospy.ServiceProxy(self.grasper_detect_name, TabletopDetection)
        self.collision_map_processing_srv = rospy.ServiceProxy(self.collision_map_processing_name, TabletopCollisionMapProcessing)
        self.bounding_box_srv = rospy.ServiceProxy(self.find_bounding_box_name, FindClusterBoundingBox)

        self.tf_listener = tf_listener
        self.roomNav = roomNav
        self.detected_clusters = None


    ##call tabletop object detection and collision_map_processing
    #(detects table/objects and adds them to collision map)
    def detect_objects(self):

        rospy.loginfo("calling tabletop detection")

        det_req = TabletopDetectionRequest()
        det_req.return_clusters = 1
        det_req.return_models = 1 
        #det_req.num_models = num_models

        #call tabletop detection, get a detection result (attempt 3 times)
        for try_num in range(3):
            try:
                det_res = self.grasper_detect_srv(det_req)
            except rospy.ServiceException, e:
                rospy.logerr("error when calling %s: %s"%(self.grasper_detect_name, e))
                self.throw_exception()
                return ([], None)
            if det_res.detection.result == det_res.detection.SUCCESS:
                rospy.loginfo("tabletop detection reports success")
                break
            else:
                rospy.logerr("tabletop detection failed with error %s, trying again"%det_res.detection.result)
        else:
            rospy.logerr("tabletop detection failed too many times. Returning.")
            return ([], None)

        self.detected_clusters = det_res.detection.clusters
        rospy.loginfo("Detected " + str(len(det_res.detection.clusters)) + " objects")
        #rospy.loginfo("Detected " + str(len(col_res.graspable_objects)) + " objects")

        # Print out the points for the first cluster
        rospy.loginfo("Coordinate frame: " + str(det_res.detection.clusters[0].header.frame_id))
        points = det_res.detection.clusters[0].points

        total_x = 0
        total_y = 0
        total_z = 0

        for point in points:
            total_x += point.x 
            total_y += point.y
            total_z += point.z

        rospy.loginfo("Avrg x: " + str(total_x / len(points)))
        rospy.loginfo("Avrg y: " + str(total_y / len(points)))
        rospy.loginfo("Avrg z: " + str(total_z / len(points)))

        camera_point_pose = Pose()
        camera_point_pose.position.x = total_x / len(points)
        camera_point_pose.position.y = total_y / len(points)
        camera_point_pose.position.z = total_z / len(points)

        map_point = self.transform(camera_point_pose, det_res.detection.clusters[0].header.frame_id, '/base_link')
        map_point.position.x -= 0.40
        map_point = self.transform(map_point, '/base_link', '/map')
        rospy.loginfo(str(map_point))
        
        self.roomNav.move_to_trash_location(map_point)

    def transform(self, pose, from_frame, to_frame):
        pose_stamped = PoseStamped()
        try:
            common_time = self.tf_listener.getLatestCommonTime(from_frame, to_frame)
            pose_stamped.header.stamp = common_time
            pose_stamped.header.frame_id = from_frame
            pose_stamped.pose = pose
            rel_pose = self.tf_listener.transformPose(to_frame, pose_stamped)
            return rel_pose.pose
        except:
            rospy.logwarn('TF exception during transform.')
            return None