import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

class LidarImuCalibration(Node):
    def __init__(self):
        super().__init__('lidar_imu_calibration_node')

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(Odometry, '/odom_rf2o', self.lidar_callback, 10)

        self.imu_data = []
        self.lidar_data = []

        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg):
        T = self.imu_msg_to_matrix(msg)
        self.imu_data.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, T))

    def lidar_callback(self, msg):
        T = self.odom_msg_to_matrix(msg)
        self.lidar_data.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, T))

        # 进行匹配并尝试计算标定
        if len(self.imu_data) >= 2 and len(self.lidar_data) >= 2:
            A_list, B_list = self.get_motion_pairs()
            if len(A_list) >= 2:
                X = self.solve_hand_eye(A_list, B_list)
                self.print_transform(X)
    
    def imu_msg_to_matrix(self, imu_msg):
        quat = imu_msg.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        rot = R.from_quat(q)
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        return T

    def odom_msg_to_matrix(self, odom_msg):
        pose = odom_msg.pose.pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot = R.from_quat(q)
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return T

    def get_motion_pairs(self):
        # 取前后两个时刻间的运动变换
        A_list, B_list = [], []
        for i in range(1, min(len(self.imu_data), len(self.lidar_data))):
            _, T_imu1 = self.imu_data[i - 1]
            _, T_imu2 = self.imu_data[i]
            A = np.linalg.inv(T_imu1) @ T_imu2

            _, T_lidar1 = self.lidar_data[i - 1]
            _, T_lidar2 = self.lidar_data[i]
            B = np.linalg.inv(T_lidar1) @ T_lidar2

            A_list.append(A)
            B_list.append(B)
        return A_list, B_list

    def solve_hand_eye(self, A_list, B_list):
        # 使用 Tsai-Lenz 方法或SVD 近似方法
        assert len(A_list) == len(B_list)
        R_As = [A[0:3, 0:3] for A in A_list]
        R_Bs = [B[0:3, 0:3] for B in B_list]

        R_X = self.solve_rotation(R_As, R_Bs)

        t_As = [A[0:3, 3] for A in A_list]
        t_Bs = [B[0:3, 3] for B in B_list]

        t_X = self.solve_translation(R_As, t_As, R_Bs, t_Bs, R_X)

        T_X = np.eye(4)
        T_X[0:3, 0:3] = R_X
        T_X[0:3, 3] = t_X
        return T_X

    def solve_rotation(self, R_As, R_Bs):
        M = np.zeros((3, 3))
        for R_A, R_B in zip(R_As, R_Bs):
            M += R_B @ R_A.T
        U, _, Vt = np.linalg.svd(M)
        R_X = U @ Vt
        return R_X

    def solve_translation(self, R_As, t_As, R_Bs, t_Bs, R_X):
        A = []
        b = []
        for R_A, t_A, R_B, t_B in zip(R_As, t_As, R_Bs, t_Bs):
            A.append(np.eye(3) - R_A)
            b.append(t_A - R_X @ t_B)
        A = np.concatenate(A, axis=0)
        b = np.concatenate(b, axis=0)
        t_X, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return t_X

    def print_transform(self, T):
        self.get_logger().info("Estimated Lidar to IMU Transform (T_lidar_to_imu):")
        self.get_logger().info("\n" + np.array2string(T, precision=4, suppress_small=True))

def main(args=None):
    rclpy.init(args=args)
    node = LidarImuCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
