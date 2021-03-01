#!/usr/bin/env python
"""
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Subscribe to a ROS image topic, transform to OpenCV image. And process the image.


D: [0.1915250639431566, -0.4623037094569031, 0.00130345932763652, -0.004691457734403636, 0.0]

K: [556.5250543909606, 0.0, 323.1341418711615,
    0.0, 516.0814264720717, 231.5051365870941,
    0.0, 0.0, 1.0]
"""

from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped  #, Point32
from sensor_msgs.msg import Image #, CameraInfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix
#from tf import TransformBroadcaster

import math

from markertracker_node.msg import GateMarker, GateMarkersArray


from ArucoWrapper import ArucoWrapper

class ReusableIdGenerator:

    def __init__(self, number):
        self.index = -1
        self.number = number

    def get_id(self):

        self.index += 1
        if self.index == self.number: self.index = 0

        return self.index

class PublisherSubscriberProcessFrame(object):
    """
     Subscribe to a image topic, call a process callback and publish results
    """

    def __init__(self, _node_name):

        self.node_name = _node_name
        # Publisher topics
        _result_image_topic = _node_name + '/image_result'
        _result_poses_topic = _node_name + '/poses'
        _result_markers_viz_topic = _node_name + '/visualization_markers'
        _result_marker_topic = _node_name + '/gate_markers'

        # Params
        _input_image_topic = rospy.get_param("~input_image_topic")
        _path_to_camera_file = rospy.get_param("~path_to_camera_file")
        self.marker_length = rospy.get_param("~marker_length")
        self.publish_topic_image_result = rospy.get_param("~publish_topic_image_result", False)
        _aruco_dictionary = rospy.get_param("~aruco_dictionary", 0)
        # TODO: Setup all remain params for Aruco

        self._camera_frame_id = rospy.get_param("~camera_frame_id", default="base_link")

        # Load camera file
        fs = cv2.FileStorage(_path_to_camera_file, cv2.FILE_STORAGE_READ)
        _camera_matrix = fs.getNode("camera_matrix").mat()
        _dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()

        # Classes
        self.bridge = CvBridge()
        self.detector = ArucoWrapper(self.marker_length, _camera_matrix, _dist_coeffs, aruco_dictionary=_aruco_dictionary)

        # Subscriber
        self.image_sub = rospy.Subscriber(_input_image_topic, Image, self._callback, queue_size=1)
        self.latest_msg = None  # to keep latest received message
        self.new_msg_available = False

        ## Publishers

        # TF broadcaster
        #self.tf_br = TransformBroadcaster()

        # Image Publisher
        if self.publish_topic_image_result:
            self.image_pub = rospy.Publisher(_result_image_topic, Image, queue_size=1)

        # Marker viz marker publisher
        self.marker_viz_pub = rospy.Publisher(_result_markers_viz_topic, MarkerArray, queue_size=100)

        # Marker viz pose publisher
        self.poses_pub = rospy.Publisher(_result_poses_topic, PoseArray, queue_size=100)

        # GateMarker publisher
        self.gate_marker_pub = rospy.Publisher(_result_marker_topic, GateMarkersArray, queue_size=100)

        self.id_gen = ReusableIdGenerator(500)

        ## Rospy loop
        r = rospy.Rate(30) # 10 Hz
        while not rospy.is_shutdown():
            if self.new_msg_available:

                self.new_msg_available = False

                self.process_frame(self.latest_msg)

                #rospy.loginfo(msg)
                r.sleep()



    def _publish_cv_image(self, image):
        if image is None: return
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def _callback(self, data):
        self.latest_msg = data
        self.new_msg_available = True

    def process_frame(self, image):

        if image is None: return

        ## Preprocess
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        ## Pose and corners used
        cv_image_result, poses = self.detector.get_poses_from_image(cv_image, draw_image=self.publish_topic_image_result)

        # process poses to massages
        if poses is not None:

            self._create_and_publish_markers_msgs_from_pose_results(poses, image.header.stamp, self._camera_frame_id)

        if cv_image_result is not None: image = cv_image_result

        ## Publish Image
        if self.publish_topic_image_result:
            self._publish_cv_image(image)


    def create_viz_marker_object(self, pose):

        marker = Marker()
        marker.id = self.id_gen.get_id()
        marker.ns = self.node_name
        marker.header.frame_id = self._camera_frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(2)

        # TODO: dims to params
        marker.scale.x = 0.025
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.10

        marker.pose.position = pose.position
        marker.pose.orientation = pose.orientation

        return marker

    def _create_and_publish_markers_msgs_from_pose_results(self, poses, image_timestamp, camera_frame_id):
        #print(poses)

        marker_array = MarkerArray() # For Rviz visualization

        pose_array = PoseArray() # For Rviz visualization with current time stamp
        pose_array.header.stamp = image_timestamp
        pose_array.header.frame_id = camera_frame_id

        gate_marker_array = GateMarkersArray() # For output results
        gate_marker_array.header.stamp = rospy.Time.now()
        gate_marker_array.camera_frame_stamp = image_timestamp

        _index = -1
        for e in poses:

            _index += 1

            if e['marker_id'] != 10: continue # TODO: use params

            gate_pose = Pose()
            gate_pose.position = Point(e['tvec'][2]/100, -e['tvec'][0]/100, -e['tvec'][1]/100) # z, -x, -y



            # Can't get this to work
            # _quaternion = quaternion_from_euler(-e['euler'][0], # Pitch
            #                                     e['euler'][1], # Yaw
            #                                     e['euler'][2] + math.pi, # Roll
            #                                     'ryzx'
            #                                     )


            _quaternion = quaternion_from_euler(e['ros_rpy'][0],
                                                e['ros_rpy'][1],
                                                e['ros_rpy'][2]
                                                )

            gate_pose.orientation.x = _quaternion[0]
            gate_pose.orientation.y = _quaternion[1]
            gate_pose.orientation.z = _quaternion[2]
            gate_pose.orientation.w = _quaternion[3]

            gate_viz_marker = self.create_viz_marker_object(gate_pose)

            gate_marker = self.create_gate_marker_object(gate_pose, tuple(e['corners']), camera_frame_id, image_timestamp, e['marker_id'])


            '''
            # Send transform
            self.tf_br.sendTransform((gate_pose.position.x, gate_pose.position.y, gate_pose.position.z),
                                     _quaternion,
                                     image_timestamp,
                                     'marker',
                                     camera_frame_id)

            '''


            # PoseWithCovarianceStamped
            #marker.pose_cov_stamped = self._create_pose_cov_stamped(marker, camera_frame_id, image_timestamp)

            marker_array.markers.append(gate_viz_marker)
            pose_array.poses.append(gate_pose)
            gate_marker_array.marker.append(gate_marker)


        # DEBUG


        '''
        _q = (marker_array.markers[0].pose.orientation.x,
              marker_array.markers[0].pose.orientation.y,
              marker_array.markers[0].pose.orientation.z,
              marker_array.markers[0].pose.orientation.w)

        _euler = euler_from_quaternion(_q)
        #_R = e['rot_m']
        #_euler = euler_from_matrix(_R, 'rzyx')


        _r = 180 * _euler[0] / math.pi
        _p = 180 * _euler[1] / math.pi
        _y = 180 * _euler[2] / math.pi
        rospy.loginfo("R:{:.0f} P:{:.0f} Y:{:.0f}".format(_r, _p, _y))
        '''

        # Publish Topics
        self.poses_pub.publish(pose_array)
        self.marker_viz_pub.publish(marker_array)
        self.gate_marker_pub.publish(gate_marker_array)

    def create_gate_marker_object(self, pose, corners, frame_id, image_timestamp, marker_id):

        marker = GateMarker()

        marker.id = marker_id
        marker.corners = corners
        marker.pose_cov_stamped.header.frame_id = frame_id
        marker.pose_cov_stamped.header.stamp = image_timestamp
        marker.pose_cov_stamped.pose.pose = pose

        # TODO: simple covariances... Get from latest ego pose/odometry
        _covariance = [1e-6, 0, 0, 0, 0, 0,
                       0, 1e-6, 0, 0, 0, 0,
                       0, 0, 1e-6, 0, 0, 0,
                       0, 0, 0, 1e-3, 0, 0,
                       0, 0, 0, 0, 1e-3, 0,
                       0, 0, 0, 0, 0, 1e-3]

        marker.pose_cov_stamped.pose.covariance = _covariance

        return marker




if __name__ == '__main__':

    _node_name = 'markertracker_node'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    PublisherSubscriberProcessFrame(_node_name)

    print('Ready.')

    rospy.spin()

