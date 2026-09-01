import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SearchDetector(Node):
    def __init__(self):
        super().__init__('search_detector')
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image, '/search_cam/image_raw', self.img_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/search_lidar/scan', self.lidar_callback, 10)

        self.busy_sub = self.create_subscription(
            Bool, '/pickup/busy', self.busy_callback, 10)

        self.angle_pub = self.create_publisher(
            Float64, '/search_cam/obj_angle', 10)
        self.cam_cmd_pub = self.create_publisher(
            JointTrajectory, '/search_cam_controller/joint_trajectory', 10)
        self.position_pub = self.create_publisher(
            Point, '/search_cam/object_position', 10)

        self.search_angle = 0.0
        self.search_step = math.radians(4.0)
        self.centre_tolerance = 20  

        self.min_angle = -math.pi
        self.max_angle = math.pi
        self.direction = 1

        self.object_detected = False
        self.center_count = 0
        self.object_x = None
        self.object_y = None
        self.image_width = 640

        self.object_distance = None
        self.fallback_dist = 0.35

        self.arm_busy = False
        self._prev_arm_busy = False

        self.search_timer = self.create_timer(0.2, self.search_callback)

        self.get_logger().info('Search detector ready')


    def busy_callback(self, msg: Bool):
        was_busy = self.arm_busy
        self.arm_busy = msg.data

        if was_busy and not self.arm_busy:
            self.get_logger().info('Arm is not busy')
            self._reset_detection()

    def _reset_detection(self):
        self.object_detected = False
        self.center_count = 0
        self.object_x = None
        self.object_y = None
        self.object_distance = None

    def command_cam(self, angle):
        msg = JointTrajectory()
        msg.joint_names = ['search_cam_joint']
        point = JointTrajectoryPoint()
        point.positions = [angle]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 0.2 s
        msg.points.append(point)
        self.cam_cmd_pub.publish(msg)

    def lidar_callback(self, msg):
        if not msg.ranges:
            self.object_distance = None
            return

        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
        self.object_distance = min(valid_ranges) if valid_ranges else None

    def img_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        self.image_width = width

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = (0, 120, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (170, 120, 100)
        upper_red2 = (179, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.object_x = None
            self.object_y = None
            self.center_count = 0
            self.object_detected = False
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if area < 25:
            self.object_x = None
            self.object_y = None
            self.center_count = 0
            self.object_detected = False
            return

        x, y, w, h = cv2.boundingRect(contour)
        self.object_x = x + w / 2.0
        self.object_y = y + h / 2.0

    def publish_position(self):
        distance = self.object_distance if self.object_distance is not None else self.fallback_dist
        total_radius = distance + 0.090

        if 0.15 <= total_radius <= 2.0:
            obj_x = total_radius * math.cos(self.search_angle)
            obj_y = total_radius * math.sin(self.search_angle)

            pos_msg = Point()
            pos_msg.x = obj_x
            pos_msg.y = obj_y
            pos_msg.z = 0.025
            self.position_pub.publish(pos_msg)
            return True
        return False

    def search_callback(self):
        if self.arm_busy:
            return

        if self.object_detected and self.object_x is None:
            self.get_logger().info('Object not visible')
            self.object_detected = False
            self.center_count = 0

        if self.object_detected:
            angle_msg = Float64()
            angle_msg.data = self.search_angle
            self.angle_pub.publish(angle_msg)
            self.publish_position()
            return

        if self.object_x is None:
            self.search_angle += self.search_step * self.direction

            if self.search_angle >= self.max_angle:
                self.search_angle = self.max_angle
                self.direction = -1

            if self.search_angle <= self.min_angle:
                self.search_angle = self.min_angle
                self.direction = 1

            self.command_cam(self.search_angle)
            self.get_logger().info(f'Searching camera angle = {math.degrees(self.search_angle):.1f} deg')
            return

        img_centre = self.image_width / 2.0
        pixel_offset = self.object_x - img_centre

        if abs(pixel_offset) <= self.centre_tolerance:
            self.center_count += 1
            if self.center_count >= 2:
                self.object_detected = True
                self.command_cam(self.search_angle)

                angle_msg = Float64()
                angle_msg.data = self.search_angle
                self.angle_pub.publish(angle_msg)
                self.publish_position()
            return
        
        self.center_count = 0
        self.object_detected = False

        rad_per_pixel = 0.0012
        angle_adjust = pixel_offset * rad_per_pixel
        angle_adjust = max(-0.12, min(0.12, angle_adjust))

        self.search_angle -= angle_adjust
        self.search_angle = max(self.min_angle, min(self.max_angle, self.search_angle))

        self.command_cam(self.search_angle)
        direction_str = 'RIGHT' if pixel_offset > 0 else 'LEFT'
        self.get_logger().info(f'Aligning Object {direction_str} , pixel_error={pixel_offset:.1f}')


def main():
    rclpy.init()
    node = SearchDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
