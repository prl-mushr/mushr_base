#!/usr/bin/env python

"""
Converts FoxGlove ROS messages into a format compatible with the MuSHR stack.
Author: Schiffer
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class NavMsgConverter:
  def __init__(self) -> None:
    """
    Initialize Navigation Messages Converter.
    Attributes:
        name (string) rosnode name
    """
    self.pose = None
    self.type = None

    # Create the subscribers
    self.pose_sub = rospy.Subscriber(
      rospy.get_param("~pose_topic"), PoseStamped, self.save_pose, queue_size=100
    )
    self.type_sub = rospy.Subscriber(
      rospy.get_param("~type_topic"), String, self.save_type, queue_size=100
    )

    # Create the publishers
    self.goal_pub = rospy.Publisher(rospy.get_param("~goal_topic"), PoseStamped, queue_size=1)
    self.car_pose_pub = rospy.Publisher(rospy.get_param("~start_topic"), PoseStamped, queue_size=1)
    self.pose_estimate_pub = rospy.Publisher(rospy.get_param("~estimate_topic"), PoseStamped, queue_size=1)

  def save_pose(self, pose_msg: PoseStamped) -> None:
    """
    Take a pose stamped message and save it.
    """
    self.pose = pose_msg
  
  def save_type(self, type_msg: String) -> None:
    self.type = type_msg.data
    if self.type != 'pose' and self.type != 'goal' and self.type != 'estimate':
      raise Exception(f'Invalid type detected {self.type}')
    if self.pose is not None:
        self.publish()
  
  def publish(self) -> None:
    if self.type == 'pose':
      self.car_pose_pub.publish(self.pose)
    elif self.type == 'estimate':
      self.pose_estimate_pub.publish(self.pose)
    else:
      self.goal_pub.publish(self.pose)
    self.pose = None
    self.type = None
