import cv2
import cv2.aruco as aruco
import math

'''
/** this conversion uses conventions as described on page:
*   https://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
*   Coordinate System: right hand
*   Positive angle: right hand
*   Order of euler angles: heading first, then attitude, then bank
*   matrix row column ordering:
*   [m00 m01 m02]
*   [m10 m11 m12]
*   [m20 m21 m22]*/
'''
def rvec2rpy_ros2(rvec):
    """
    From an OpenCV rotational vector, convert to RPY euler angles for ROS2
    """

    m, _ = cv2.Rodrigues(rvec)

    # // Assuming the angles are in radians.
    if m[1, 0] > 0.998:  # // singularity at north pole
        yaw = math.atan2(m[0, 2], m[2, 2])
        roll = math.PI / 2
        pitch = 0
    elif m[1, 0] < -0.998:  # // singularity at south pole
        yaw = math.atan2(m[0, 2], m[2, 2])
        roll = -math.PI / 2
        pitch = 0

    else:
        roll = -math.atan2(-m[2, 0], m[0, 0]) + math.pi
        pitch = -math.atan2(m[2, 2], m[1, 2]) + math.pi / 2
        yaw = -math.asin(m[1, 0])

    return roll, pitch, yaw


class ArucoWrapper:

    def __init__(self, marker_length, camera_matrix, dist_coeffs, aruco_dictionary_name="DICT_4X4_50"):

            # Make sure we have a valid dictionary name:
            try:
                _dictionary_id = cv2.aruco.__getattribute__(aruco_dictionary_name)
                if type(_dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                    raise AttributeError
            except AttributeError:
                self.get_logger().error("bad aruco_dictionary_id: {}".format(aruco_dictionary_name))
                options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
                self.get_logger().error("valid options: {}".format(options))


            self.aruco_dict = aruco.getPredefinedDictionary(_dictionary_id)
            self.parameters = aruco.DetectorParameters_create()

            self.marker_length = marker_length
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs


    def find_corners_from_image(self, image, draw_image=False, image_is_gray = False):

        if image_is_gray:
            image_gray = image
        else:
            image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        image_result = None

        aruco_corners, aruco_ids, aruco_rejected_points = aruco.detectMarkers(image_gray,
                                                                              self.aruco_dict,
                                                                              parameters=self.parameters)

        if draw_image is True and aruco_ids is not None and len(aruco_ids) > 0:
            image_result = aruco.drawDetectedMarkers(image, aruco_corners, aruco_ids)

        return image_result, aruco_corners, aruco_ids, aruco_rejected_points


    def find_poses_from_corners(self, aruco_ids, aruco_corners, image_color=None, draw_image=False):

        poses_result = None

        if aruco_ids is not None and len(aruco_ids) > 0:

            poses_result = list()

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(aruco_corners,
                                                                       self.marker_length,
                                                                       self.camera_matrix,
                                                                       self.dist_coeffs)

            for i in range(len(aruco_ids)):

                # euler, R = self._get_euler_vector_from_rvec(rvecs[i])
                # R, _ = cv2.Rodrigues(rvecs[i])

                pose = { 'marker_id': aruco_ids[i][0],
                         'corners': aruco_corners[i].flatten(),
                         'tvec': tvecs[i].flatten(),
                         'rvec': rvecs[i].flatten(),
                         'ros_rpy': rvec2rpy_ros2(rvecs[i]),
                         # 'rot_m': R,
                         # 'euler': euler
                         }

                poses_result.append(pose)

                if draw_image is True and image_color is not None:
                    image_color = aruco.drawAxis(image_color,
                                                self.camera_matrix,
                                                self.dist_coeffs,
                                                rvecs[i], tvecs[i],
                                                self.marker_length / 2)

        return image_color, poses_result

    def get_poses_from_image(self,image, draw_image=False):
        _, aruco_corners, aruco_ids, _ = self.find_corners_from_image(image)
        return self.find_poses_from_corners(aruco_ids, aruco_corners, image, draw_image=draw_image)


