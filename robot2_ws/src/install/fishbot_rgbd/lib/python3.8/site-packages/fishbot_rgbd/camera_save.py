import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class CameraSaveNode(Node):
    def __init__(self, name='camera_save_node'):
        super().__init__(name)

        #============= 1. 定义输出文件夹、文本路径等 =============
        self.dataset_path = '/home/zqh/robot2/robot2_ws/src/fishbot_rgbd/dataset'
        os.makedirs(self.dataset_path, exist_ok=True)
        self.traj_file_path = os.path.join(self.dataset_path, "traj.txt")
        # 若想启动时清空，可执行:
        # open(self.traj_file_path, "w").close()

        #============= 2. TF Buffer 和 Listener =============
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #============= 3. CVBridge =============
        self.bridge = CvBridge()

        #============= 4. 订阅单一相机的图像 (RGB + Depth) =============
        self.latest_rgb = None
        self.latest_depth = None
        self.subscription_rgb = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.rgb_cb,
            10
        )
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera_sensor/depth/image_raw',
            self.depth_cb,
            10
        )

        #============= 5. 全局图像计数器，用于匹配"一张图 -> 一行外参" =============
        self.global_image_id = 0

        #============= 6. 定时器：处理单一相机的数据 =============
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("CameraSaveNode initialized! Waiting for camera images & TF...")

    #========== 相机图像回调 ==========
    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def depth_cb(self, msg):
        self.latest_depth = msg

    #========== 定时器回调：保存单一相机数据 ==========
    def timer_callback(self):
        self.process_camera()

    def process_camera(self):
        """
        如果RGB和Depth都有数据，则保存成 JPG/PNG，并用 odom->camera_link 的外参写入 traj.txt。
        重点：这里将 depth float32 (米) 转为 uint16 (毫米) 保存。
        """
        if self.latest_rgb is None or self.latest_depth is None:
            self.get_logger().warn("[Camera] Missing rgb/depth => skip.")
            return

        #=== 1) 保存彩色图 (JPG) ===
        self.global_image_id += 1
        color_name = f"color_{self.global_image_id}.jpg"
        depth_name = f"depth_{self.global_image_id}.png"

        color_path = os.path.join(self.dataset_path, color_name)
        depth_path = os.path.join(self.dataset_path, depth_name)

        cv_rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding="bgr8")

        #--- 将 float32 (单位:米) 转成 uint16 (单位:毫米) ---
        cv_depth_float = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
        mm_depth = np.clip(cv_depth_float * 1000.0, 0, 65535).astype(np.uint16)

        #=== 保存到硬盘 ===
        cv2.imwrite(color_path, cv_rgb)
        cv2.imwrite(depth_path, mm_depth)

        self.get_logger().info(f"[Camera] Saved => {color_path}, {depth_path} (depth in mm)")

        #=== 2) 获取 odom->camera_link 的 4x4 外参，并写入 traj.txt ===
        extrinsic_ok, matrix_4x4 = self.lookup_extrinsic("map", "camera_optical_link")
        if not extrinsic_ok:
            self.get_logger().warn("[Camera] Cannot lookup transform => skip extrinsic.")
            return

        #=== 3) 写入 traj.txt ===
        with open(self.traj_file_path, "a") as f:
            line_str = " ".join(f"{val:.18e}" for val in matrix_4x4)
            f.write(line_str + "\n")

        self.get_logger().info(f"[Camera] Wrote extrinsic => {self.traj_file_path} (line={self.global_image_id})")

    def lookup_extrinsic(self, source_frame, target_frame):
        """
        从 TF Buffer 中查询 source->target 的变换，并转为 4x4 矩阵(list[16])。
        比如 source_frame='odom', target_frame='camera_link'。
        """
        try:
            tfs = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"lookup_transform {source_frame}->{target_frame} failed: {e}")
            return False, None

        mat_4x4 = self.transform_to_matrix(tfs)
        return True, mat_4x4

    def transform_to_matrix(self, transform_stamped: TransformStamped):
        """
        将 geometry_msgs/TransformStamped 转为 4×4 齐次变换矩阵 (list[16], 行优先)
        """
        from tf_transformations import quaternion_matrix
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        quat = [q.x, q.y, q.z, q.w]
        mat = quaternion_matrix(quat)
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z

        return mat.flatten().tolist()

def main():
    rclpy.init()
    node = CameraSaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
