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
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, Vector3Stamped
from tf_conversions import posemath
from tf import TransformListener
from room_navigator import *
from transformer import *
from actionlib import SimpleActionClient

class PickAndPlaceManager():
    def __init__(self, tf_listener, roomNav, animation_player):

        # Services used for object detection
        self.grasper_detect_name = 'object_detection'
        self.collision_map_processing_name = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        self.find_bounding_box_name = '/find_cluster_bounding_box'

        rospy.loginfo("grasp_executive: waiting for object_detection service")
        rospy.wait_for_service(self.grasper_detect_name)
        rospy.loginfo("grasp_executive: object_detection service found")

        self.grasper_detect_srv = rospy.ServiceProxy(self.grasper_detect_name, TabletopDetection)
        self.collision_map_processing_srv = rospy.ServiceProxy(self.collision_map_processing_name, TabletopCollisionMapProcessing)
        self.bounding_box_srv = rospy.ServiceProxy(self.find_bounding_box_name, FindClusterBoundingBox)

        self.tf_listener = tf_listener
        self.roomNav = roomNav
        self.animPlay = animation_player
        self.detected_clusters = None


    #Returns the location of the nearest object
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


        col_req = TabletopCollisionMapProcessingRequest()
        col_req.reset_collision_models = 1
        col_req.reset_attached_models = 1
        col_req.detection_result = det_res.detection
        col_req.desired_frame = 'base_link'

        #call collision map processing to add the detected objects to the collision map
        #and get back a list of GraspableObjects
        try:
            col_res = self.collision_map_processing_srv(col_req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.collision_map_processing_name, e))
            self.throw_exception()
            return (None)


        '''
        pickup_client = SimpleActionClient("/object_manipulator/object_manipulator_pickup", PickupAction)
        pickup_client.wait_for_server()

        rospy.loginfo("Calling the pickup action");
        pickup_goal = PickupGoal()

        pickup_goal.target = col_res.graspable_objects[0];
        
        #pass the name that the object has in the collision environment
        #this name was also returned by the collision map processor
        pickup_goal.collision_object_name = col_res.collision_object_names[0]
        
        #pass the collision name of the table, also returned by the collision 
        #map processor
        pickup_goal.collision_support_surface_name = col_res.collision_support_surface_name;
        
        #pick up the object with the left arm
        pickup_goal.arm_name = "left_arm";
        
        #we will be lifting the object along the "vertical" direction
        #which is along the z axis in the base_link frame
        direction = Vector3Stamped()
        direction.header.stamp = rospy.Time.now()
        direction.header.frame_id = "base_link"
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        
        #request a vertical lift of 10cm after grasping the object
        pickup_goal.lift.desired_distance = 0.1
        
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = 0
        pickup_goal.use_reactive_execution = 0
        
        
        #send the goal
        pickup_client.send_goal(pickup_goal);
        rospy.loginfo("sent pickup goal")

        
          while (!pickup_client.waitForResult(ros::Duration(10.0)))
          {
            ROS_INFO("Waiting for the pickup action...");
          }
          object_manipulation_msgs::PickupResult pickup_result = 
            *(pickup_client.getResult());
          if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_ERROR("The pickup action has failed with result code %d", 
                      pickup_result.manipulation_result.value);
            return -1;
          }
          '''

        if(len(det_res.detection.clusters) > 0):
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
            '''
            map_point = self.transform(camera_point_pose, det_res.detection.clusters[0].header.frame_id, '/base_link')
            map_point.position.x -= 0.50
            map_point = self.transform(map_point, '/base_link', '/map')
            '''

            map_point = Transformer.transform(self.tf_listener, camera_point_pose, det_res.detection.clusters[0].header.frame_id, '/map')
            rospy.loginfo(str(map_point.pose))
            
            return map_point
        else:
            return None


    def detect_and_pickup(self):
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

        col_req = TabletopCollisionMapProcessingRequest()
        col_req.reset_collision_models = 1
        col_req.reset_attached_models = 1
        col_req.detection_result = det_res.detection
        col_req.desired_frame = 'base_link'

        #call collision map processing to add the detected objects to the collision map
        #and get back a list of GraspableObjects
        try:
            col_res = self.collision_map_processing_srv(col_req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.collision_map_processing_name, e))
            self.throw_exception()
            return (None)



        pickup_client = SimpleActionClient("/object_manipulator/object_manipulator_pickup", PickupAction)
        pickup_client.wait_for_server()

        rospy.loginfo("Calling the pickup action")
        pickup_goal = PickupGoal()

        pickup_goal.target = col_res.graspable_objects[0];
        
        #pass the name that the object has in the collision environment
        #this name was also returned by the collision map processor
        pickup_goal.collision_object_name = col_res.collision_object_names[0]
        
        #pass the collision name of the table, also returned by the collision 
        #map processor
        pickup_goal.collision_support_surface_name = col_res.collision_support_surface_name
        
        #pick up the object with the left arm
        pickup_goal.arm_name = "left_arm"
        
        #we will be lifting the object along the "vertical" direction
        #which is along the z axis in the base_link frame
        direction = Vector3Stamped()
        direction.header.stamp = rospy.Time.now()
        direction.header.frame_id = "base_link"
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        
        #request a vertical lift of 10cm after grasping the object
        pickup_goal.lift.desired_distance = 0.1
        
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = 0
        pickup_goal.use_reactive_execution = 0
        
        
        #send the goal
        pickup_client.send_goal(pickup_goal)
        rospy.loginfo("sent pickup goal")
        # wait for the head movement to finish before we try to detect and pickup an object
        finished_within_time = pickup_client.wait_for_result(rospy.Duration(30))
        return finished_within_time
        # Check for success or failure
        '''if not finished_within_time:
            pickup_client.cancel_goal()
            rospy.loginfo("Timed out achieving pickup goal")
        else:
            state = pickup_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Pickup goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Pickup goal failed with error code: " + str(self.goal_states[state]))
        '''
