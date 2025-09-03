import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import image_geometry

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # 파라미터 선언 (마커 크기 등)
        self.declare_parameter('marker_size', 0.1)  # Gazebo 모델의 크기와 일치
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        # 카메라 정보를 받기 위한 변수 초기화
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_model = image_geometry.PinholeCameraModel()

        # CV Bridge
        self.bridge = CvBridge()

        # ArUco 사전 정의
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # ROS 구독자, 발행자, TF Broadcaster 설정
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Aruco Detector Node has been started.')

    def cam_info_callback(self, msg):
        # 카메라 정보를 한번만 받아서 저장
        if self.camera_matrix is None:
            self.camera_model.fromCameraInfo(msg)
            self.camera_matrix = self.camera_model.intrinsicMatrix()
            self.dist_coeffs = self.camera_model.distortionCoeffs()
            self.get_logger().info('Camera calibration info received.')
            # 구독 취소
            self.cam_info_sub.destroy()


    def image_callback(self, msg):
        # 카메라 정보가 없으면 아무것도 하지 않음
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera calibration info...')
            return

        # ROS 이미지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ArUco 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # 3D Pose 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                
                # --- PoseStamped 메시지 발행 ---
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = msg.header.frame_id # 카메라 프레임
                
                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]
                
                # rvec (회전 벡터) -> 쿼터니언 변환
                rot_matrix = cv2.Rodrigues(rvec)[0]
                # Scipy나 다른 라이브러리를 사용하여 변환하거나 직접 계산
                # 간단한 예시로 여기서는 직접 계산
                q = self.rotation_matrix_to_quaternion(rot_matrix)
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]
                
                self.pose_pub.publish(pose_msg)

                # --- TF 발행 ---
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = msg.header.frame_id # 부모 프레임: 카메라
                t.child_frame_id = f'aruco_marker_{ids[i][0]}' # 자식 프레임: 마커
                
                t.transform.translation.x = tvec[0]
                t.transform.translation.y = tvec[1]
                t.transform.translation.z = tvec[2]
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                
                self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, m):
        # 변환 로직 (numpy를 이용하면 더 간단하게 구현 가능)
        t = np.trace(m)
        if t > 0:
            s = 0.5 / np.sqrt(t + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        return x, y, z, w

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()