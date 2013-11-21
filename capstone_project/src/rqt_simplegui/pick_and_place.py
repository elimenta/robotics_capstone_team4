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
from tf_conversions import posemath

class PickAndPlaceManager():
    def __init__(self):

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



        ####################### Extra
        '''#initialize a collision map interface
        self.collision_map_interface = collision_map_interface.CollisionMapInterface()

        #the objects held in the each arm (None if no object)
        self.held_objects = [None]*2

        #temporary height and dist until we get a table detection
        self.table_front_edge_x = rospy.get_param("~default_table_front_edge_x", .33)
        self.table_height = rospy.get_param("~default_table_height", .66)

        #saved table and object detections
        self.detected_table = None
        self.additional_tables = []
        self.detected_objects = []

        #dictionary of grasp/place error codes
        #(SUCCESS, UNFEASIBLE, FAILED, ERROR, MOVE_ARM_STUCK, LIFT_FAILED)
        self.result_code_dict = {}
        for element in dir(ManipulationResult):
            if element[0].isupper():
                self.result_code_dict[eval('ManipulationResult.'+element)] = element

        #dictionary of tabletop_object_detector error codes
        #(NO_CLOUD_RECEIVED, NO_TABLE, OTHER_ERROR, SUCCESS)
        self.tabletop_detection_result_dict = {}
        for element in dir(TabletopDetectionResult):
            if element[0].isupper():
                self.tabletop_detection_result_dict[eval('TabletopDetectionResult.'+element)] = element

        #name of the support surface's collision object
        self.collision_support_surface_name = "table"'''

    ##call tabletop object detection and collision_map_processing
    #(detects table/objects and adds them to collision map)
    def detect_objects(self):

        rospy.loginfo("calling tabletop detection")

        det_req = TabletopDetectionRequest()
        det_req.return_clusters = 1
        det_req.return_models = 0 
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
                rospy.logerr("tabletop detection failed with error %s, trying again"%\
                                 self.tabletop_detection_result_dict[det_res.detection.result])
        else:
            rospy.logerr("tabletop detection failed too many times. Returning.")
            return ([], None)

        col_req = TabletopCollisionMapProcessingRequest()
        col_req.reset_collision_models = 0
        col_req.reset_attached_models = 0
        col_req.detection_result = det_res.detection
        col_req.desired_frame = 'base_link'

        #call collision map processing to add the detected objects to the collision map
        #and get back a list of GraspableObjects
        try:
            col_res = self.collision_map_processing_srv(col_req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.collision_map_processing_name, e))
            self.throw_exception()
            return ([], None)

        table = det_res.detection.table
        self.collision_support_surface_name = col_res.collision_support_surface_name

        #save the new detected table (already in collision map)
        #self.detected_table = table
        #self.update_table_info(update_place_rectangle)

        rospy.loginfo("Detected " + str(len(col_res.graspable_objects)) + " objects")