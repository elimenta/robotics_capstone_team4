
#!/usr/bin/env python

import roslib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from tf import TransformListener
class Transformer:

    def __init__(self):
        pass

    @staticmethod
    def transform(tf_listener, pose, from_frame, to_frame):
        pose_stamped = PoseStamped()
        try:
            common_time = tf_listener.getLatestCommonTime(from_frame, to_frame)
            pose_stamped.header.stamp = common_time
            pose_stamped.header.frame_id = from_frame
            pose_stamped.pose = pose
            rel_pose = tf_listener.transformPose(to_frame, pose_stamped)
            return rel_pose
        except:
            rospy.logwarn('TF exception during transform.')
            return None