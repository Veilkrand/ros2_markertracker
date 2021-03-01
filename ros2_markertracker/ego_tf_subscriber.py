#!/usr/bin/env python  

"""
The FCU publishes two TF:
frame_id: "NED", child_frame_id: "aircraft"
frame_id: "map", child_frame_id: "base_link"

We need to transform the marker pose from the camera frame to the world frame.

And publish the transform, and object.
"""

import rospy

import math
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs


br = tf2_ros.TransformBroadcaster()

def publish_tf_from_pose(pose):
    
    global br    
    
    t = geometry_msgs.msg.TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = pose.header.frame_id
    t.child_frame_id = "gates"
    t.transform.translation = pose.pose.position
#    t.transform.translation = pose.point
#    t.transform.translation = pose.point

    t.transform.rotation = pose.pose.orientation

    br.sendTransform(t)


if __name__ == '__main__':

    rospy.init_node('tf2_ego_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    r = rospy.Rate(10) #10
    
    marker = geometry_msgs.msg.PoseStamped()
    marker.header.frame_id = "/base_link"
    marker.header.stamp = rospy.Time.now()
    marker.pose.position.z = 0.0
    marker.pose.position.x = 1.0
    marker.pose.position.y = 0.0
    marker.pose.orientation.w = 1.0
    
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            r.sleep()
            continue
        
        # print transform
        
        pose_transformed = tf2_geometry_msgs.do_transform_pose(marker, transform)
        
        #print pose_transformed
        
        
        publish_tf_from_pose(pose_transformed)
        
        """
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        """


        r.sleep()
