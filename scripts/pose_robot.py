import cv2
import numpy as np
import rospy
import cv2.aruco as aruco
from geometry_msgs.msg import Pose2D

class ArucoPoseEstimator:
    def __init__(self):
        rospy.init_node('aruco_pose_estimator', anonymous=True)

        self.pose_pub = rospy.Publisher('/odom', Pose2D, queue_size=10)

        self.marker_length = 0.1  # Kích thước cạnh của marker (mét)
        self.camera_matrix = np.array([[2683.153435692087, 0., 321.89466335337528],
                                       [0.,1182.9815351617285, 232.76060343756683],
                                       [0., 0., 1.]], dtype=np.float32)
        self.dist_coeffs = np.array([-9.3820054004119005, 178.69685393040132, 0.35328237069124296,
       -0.027958402430900357, 3.3230360354020156], dtype=np.float32)  # Hệ số méo (distortion coefficients)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        self.cap = cv2.VideoCapture(0)
        self.marker_size = 0.196  # Define the size of the marker in meters

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
                for rvec, tvec in zip(rvecs, tvecs):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_size)
                    self.publish_pose(rvec, tvec)

            
            cv2.imshow("Aruco Pose Estimation", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def publish_pose(self, rvec, tvec):
        # Chuyển đổi tọa độ từ hệ tọa độ camera sang hệ tọa độ robot
        tvec_robot = -np.dot(cv2.Rodrigues(rvec)[0].T, tvec[0].T).T

        pose = Pose2D()
        pose.x = tvec_robot[0][0]
        pose.y = tvec_robot[0][1]

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        pose.theta = self.rotation_matrix_to_yaw(rotation_matrix)

        print(pose)
        
        self.pose_pub.publish(pose)

    def rotation_matrix_to_yaw(self, R):
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return yaw

if __name__ == '__main__':
    try:
        estimator = ArucoPoseEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
