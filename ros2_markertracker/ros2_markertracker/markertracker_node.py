#!/usr/bin/env python
"""
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Subscribe to a ROS raw image topic, transform to OpenCV image. And process the image.

"""
import threading

import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped  #, Point32
from sensor_msgs.msg import Image #, CameraInfo

# from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix
from ros2_markertracker.transformations import quaternion_from_euler

#from tf import TransformBroadcaster

import math

from ros2_markertracker_interfaces.msg import FiducialMarker, FiducialMarkerArray
from ros2_markertracker.ArucoWrapper import ArucoWrapper

# ---
# Ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# from std_msgs.msg import String




def main(args=None):

    rclpy.init(args=args)

    markertracker_node = ProcessFramePubSub()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(markertracker_node,), daemon=True)
    thread.start()
    # Processing rate can be adjusted to only process the last image frame received
    r = markertracker_node.create_rate(30)  #30 Hz # 10 Hz
    try:
        while rclpy.ok():
            # markertracker_node.get_logger().info('New loop')
            if markertracker_node.new_msg_available:
                markertracker_node.new_msg_available = False
                markertracker_node.process_frame(markertracker_node.latest_msg)
            r.sleep()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    markertracker_node.destroy_node()
    rclpy.shutdown()
    thread.join()



# -----
class ReusableIdGenerator:

    def __init__(self, number):
        self.index = 1
        self.number = number

    def get_id(self):
        self.index += 1

        if self.index == self.number: self.index = 1

        return int(self.index)

class ProcessFramePubSub(Node):
    """
     Subscribe to a image topic, call a process callback and publish results
    """

    def __init__(self):

        super().__init__('markertracker_node')

        # Publisher topics
        _result_image_topic = '/image_result'
        _result_poses_topic = '/poses'
        _result_markers_viz_topic = '/visualization_markers'
        _result_marker_topic = '/fiducial_markers'

        # Declare and read parameters
        self.declare_parameter("input_image_topic", "/camera/image_raw")
        _input_image_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value

        self.declare_parameter("marker_length", 10)
        self.marker_length = self.get_parameter("marker_length").get_parameter_value().double_value

        self.declare_parameter("publish_topic_image_result", False)

        self.declare_parameter("camera_frame_id", "camera")
        self._camera_frame_id = self.get_parameter("camera_frame_id").get_parameter_value().string_value

        # self.declare_parameter("camera_info_topic", "/camera/camera_info")
        # info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
        _aruco_dictionary_id = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value

        self.declare_parameter("path_to_camera_file", "calibration/camerav2_1280x720.yaml")
        _path_to_camera_file = self.get_parameter("path_to_camera_file").get_parameter_value().string_value

        # TODO: Setup all remain params for Aruco

        self.publish_topic_image_result = True


        # Load camera file
        self.get_logger().info(f'Loading camera file: {_path_to_camera_file}')
        fs = cv2.FileStorage(_path_to_camera_file, cv2.FILE_STORAGE_READ)
        _camera_matrix = fs.getNode("camera_matrix").mat()
        _dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        assert(len(_camera_matrix) and len(_dist_coeffs))
        self.get_logger().debug(f'Camera Matrix: {_camera_matrix}')
        self.get_logger().debug(f'Dist Coeff: {_dist_coeffs}')



        # Setup OpenCV
        self.bridge = CvBridge()
        self.detector = ArucoWrapper(self.marker_length,
                                     _camera_matrix, _dist_coeffs,
                                     aruco_dictionary_name=_aruco_dictionary_id)

        ## --
        ## Subscribers

        # Image raw topic

        self.image_sub = self.create_subscription(Image,
                                                  _input_image_topic,
                                                  self._image_callback,
                                                  qos_profile=qos_profile_sensor_data)

        # self.image_sub = rospy.Subscriber(_input_image_topic, Image, self._callback, queue_size=1)
        self.get_logger().info(f'Subscribed to {_input_image_topic}')


        self.latest_msg = None  # keep latest received message
        self.new_msg_available = False

        ## ---
        ## Publishers

        # TF broadcaster
        #self.tf_br = TransformBroadcaster()

        # Image Publisher
        if self.publish_topic_image_result:
            self.image_pub = self.create_publisher(Image, _result_image_topic, 1)

        # Marker viz marker publisher
        self.marker_viz_pub = self.create_publisher(MarkerArray, _result_markers_viz_topic, 100)

        # Marker viz pose publisher
        self.poses_pub = self.create_publisher(PoseArray, _result_poses_topic, 100)

        # FiducialMarkerArray publisher
        self.fiducial_markers_pub = self.create_publisher(FiducialMarkerArray, _result_marker_topic, 100)

        self.id_gen = ReusableIdGenerator(500)


    def _publish_cv_image(self, image):
        if image is None: return
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def _image_callback(self, data):
        # self.get_logger().info('Got New camera image')
        self.latest_msg = data
        self.new_msg_available = True

    def process_frame(self, image):

        if image is None:
            return

        # self.get_logger().info('New image')

        ## Preprocess
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            raise e

        ## Pose and corners used
        cv_image_result, poses = self.detector.get_poses_from_image(cv_image,
                                                                    draw_image=self.publish_topic_image_result)

        # process poses to messages
        if poses is not None:
            self._create_and_publish_markers_msgs_from_pose_results(poses, image.header.stamp, self._camera_frame_id)


        # Publish CV debug image
        if self.publish_topic_image_result:
            if cv_image_result is not None:
                self._publish_cv_image(cv_image_result)


    def create_viz_marker_object(self, pose):

        marker = Marker()
        marker.id = self.id_gen.get_id()

        # marker.ns = self.node_name
        marker.header.frame_id = self._camera_frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

        # Size of the viz marker
        marker.scale.x = 0.025
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.10

        marker.pose.position = pose.position
        marker.pose.orientation = pose.orientation

        return marker

    def _create_and_publish_markers_msgs_from_pose_results(self, poses, image_timestamp, camera_frame_id):
        #print(poses)

        marker_array = MarkerArray()  # For Rviz visualization

        pose_array = PoseArray()  # For Rviz visualization with current time stamp
        pose_array.header.stamp = image_timestamp
        pose_array.header.frame_id = camera_frame_id

        gate_marker_array = FiducialMarkerArray()  # For output results
        gate_marker_array.header.stamp = self.get_clock().now().to_msg()
        gate_marker_array.camera_frame_stamp = image_timestamp

        _index = -1
        for e in poses:

            _index += 1

            if e['marker_id'] != 10: continue # TODO: use params

            gate_pose = Pose()

            # Debug OpenCV Ouput
            # self.get_logger().info(f"0:{e['tvec'][0]} 1:{e['tvec'][1]} 2:{e['tvec'][2]}")
            # self.get_logger().debug(f"roll:{e['ros_rpy'][0]} pitch:{e['ros_rpy'][1]} yaw:{e['ros_rpy'][2]}")

            self.get_logger().info(f"y:{e['tvec'][0]} z:{e['tvec'][1]} x:{e['tvec'][2]}")

            # z, -x, -y
            gate_pose.position.x = e['tvec'][2]
            gate_pose.position.y = -e['tvec'][0]
            gate_pose.position.z = -e['tvec'][1]

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
        self.fiducial_markers_pub.publish(gate_marker_array)

    def create_gate_marker_object(self, pose, corners, frame_id, image_timestamp, marker_id):

        marker = FiducialMarker()
        marker.id = int(marker_id)
        # marker.corners = corners # TODO: debug corners
        # self.get_logger().info(f'corners: {corners}')
        marker.pose_cov_stamped.header.frame_id = frame_id
        marker.pose_cov_stamped.header.stamp = image_timestamp
        marker.pose_cov_stamped.pose.pose = pose

        # TODO: simple covariances... Get from latest ego pose/odometry
        _covariance = [1e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 1e-6, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 1e-3]

        marker.pose_cov_stamped.pose.covariance = _covariance

        return marker




if __name__ == '__main__':
    main()
