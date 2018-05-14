#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from hanp_msgs.msg import Trajectory
from head_manager.msg import TargetWithPriority
import math
import tf.transformations
from dynamic_reconfigure.server import Server
from acting_navigation_head_behavior.cfg import NavHeadBehaviorConfig

NODE_NAME = "acting_navigation_head"

HEAD_TARGET_PUB_TOPIC_NAME = "/head_manager/head_acting_target"
DEBUG_ACTING_PUB_TOPIC_NAME = "/head_manager/debug_acting_point"
LOCAL_PLAN_SUB_TOPIC_NAME = "/local_traj"
PUBLISH_FREQUENCY = 10.0  # Hz

NAVIGATION_PRIORITY = 210


class NavHeadBehavior:
    def __init__(self, name):
        self._name = name
        self._cfg_srv = Server(NavHeadBehaviorConfig, self.reconfigure_cb)
        self._head_target_pub = rospy.Publisher(HEAD_TARGET_PUB_TOPIC_NAME, TargetWithPriority, queue_size=10)
        self._debug_acting_pub = rospy.Publisher(DEBUG_ACTING_PUB_TOPIC_NAME, PointStamped, queue_size=10)
        self._local_trajectory_sub = rospy.Subscriber(LOCAL_PLAN_SUB_TOPIC_NAME, Trajectory, self.on_new_local_path)

        self._current_path = None  # type: Trajectory

        self._local_plan_delay = rospy.Duration(2.0)
        self._look_point_elevation = 1.1
        self._end_plan_look_distance = 1.
        self._future_position_time = rospy.Duration(2.0)

    def publish_point(self, _):
        if self._current_path is None:
            return
        if rospy.Time.now() - self._current_path.header.stamp > self._local_plan_delay:
            # Last local plan is too old, considering we are not navigating any more
            rospy.loginfo_throttle(1, "We are not navigating any more, disabling navigation head behavior.")
            return

        last_local_traj_pose  = None
        for pose in self._current_path.points:
            if pose.time_from_start <= rospy.Duration(0):
                continue  # Points not taken into local plans have -1 in time_from_start
            last_local_traj_pose = pose
            #rospy.loginfo(pose.time_from_start.to_sec())
            if self._future_position_time < pose.time_from_start:
                msg = TargetWithPriority()
                p = PointStamped()
                p.header.frame_id = self._current_path.header.frame_id
                p.header.stamp = rospy.Time.now()
                p.point.x = pose.transform.translation.x
                p.point.y = pose.transform.translation.y
                p.point.z = self._look_point_elevation
                msg.target = p
                msg.priority = NAVIGATION_PRIORITY
                self._head_target_pub.publish(msg)
                self._debug_acting_pub.publish(p)
                return

        return
        # End of local plan hit, looking a point towards final desired orientation
        if last_local_traj_pose is None:
            last_local_traj_pose = self._current_path.points[-1]

        _, _, yaw = tf.transformations.euler_from_quaternion([last_local_traj_pose.transform.rotation.x, last_local_traj_pose.transform.rotation.y,
                                                              last_local_traj_pose.transform.rotation.z, last_local_traj_pose.transform.rotation.w])

        msg = TargetWithPriority()
        p = PointStamped()
        p.header.frame_id = self._current_path.header.frame_id
        p.header.stamp = rospy.Time.now()
        p.point.x = last_local_traj_pose.transform.translation.x + self._end_plan_look_distance * math.cos(yaw)
        p.point.y = last_local_traj_pose.transform.translation.y + self._end_plan_look_distance * math.sin(yaw)
        p.point.z = self._look_point_elevation
        msg.target = p
        msg.priority = NAVIGATION_PRIORITY
        self._head_target_pub.publish(msg)
        self._debug_acting_pub.publish(p)

    def on_new_local_path(self, local_path):
        self._current_path = local_path

    def reconfigure_cb(self, config, level):

        self._look_point_elevation = config.look_point_elevation
        self._local_plan_delay = rospy.Duration(config.local_plan_max_delay)
        self._end_plan_look_distance = config.end_plan_look_distance
        self._future_position_time = rospy.Duration(config.time_future_look_point)
        rospy.loginfo(NODE_NAME + " reconfigured.")
        return config


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    hb = NavHeadBehavior(NODE_NAME)
    rospy.Timer(rospy.Duration(1./PUBLISH_FREQUENCY), hb.publish_point)
    rospy.spin()

