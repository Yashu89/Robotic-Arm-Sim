import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy
import cv2
from cv_bridge import CvBridge


class PickupController(Node):
    def __init__(self):
        super().__init__('pickup_controller')

        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for arm_controller action server...')
        if not self.arm_action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('arm_controller action server not available!')

        self.get_logger().info('arm_controller action server connected!')

        moveit_config = (
            MoveItConfigsBuilder("ur5", package_name="robotic_arm_moveit")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )
        moveit_dict = moveit_config.to_dict()
        moveit_dict["planning_pipelines"] = {
            "pipeline_names": ["ompl"]
        }
        moveit_dict["default_planning_pipeline"] = "ompl"
        moveit_dict["use_sim_time"] = True
        moveit_dict["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "planning_attempts": 15,
            "max_velocity_scaling_factor": 0.5,
            "max_acceleration_scaling_factor": 0.5,
        }

        self.moveit = MoveItPy(node_name='pickup_controller_moveit', config_dict=moveit_dict)
        self.arm = self.moveit.get_planning_component('arm')

        self.position_sub = self.create_subscription(
            Point, '/search_cam/object_position', self.object_callback, 10)

        self.wrist_cam_sub = self.create_subscription(
            Image, '/wrist_cam/image_raw', self.wrist_cam_callback, 10)

        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.busy_pub = self.create_publisher(Bool, '/pickup/busy', 10)

        self.bridge = CvBridge()
        self.wrist_img = None

        self.busy = False

        self.approach_height = 0.45
        self.wrist_step = 0.02
        self.min_z = 0.040

        self.basket_x = -0.8
        self.basket_y = 0.3
        self.drop = 0.2

        self.end_effector_link = 'tool0'
        self.wrist_precheck_timeout = 5.0
        self.wrist_xy_gain = 0.0001

        self._publish_busy(False)
        self.get_logger().info('Pickup Controller ready!')

    def _publish_busy(self, state: bool):
        msg = Bool()
        msg.data = state
        self.busy_pub.publish(msg)

    def _set_busy(self, state: bool):
        self.busy = state
        self._publish_busy(state)

    def move_to_scan_pose(self, attempt: int = 1, max_attempts: int = 3) -> bool:
        for i in range(1, max_attempts + 1):
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(configuration_name='scan_pose')
            plan_result = self.arm.plan()

            if plan_result and plan_result.trajectory:
                self.moveit.execute(plan_result.trajectory, controllers=['arm_controller'])
                time.sleep(1.5)
                return True
            else:
                if i < max_attempts:
                    time.sleep(1.0)
        return False

    def wrist_cam_callback(self, msg):
        self.wrist_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def wrist_obj_det(self):
        if self.wrist_img is None:
            return None, None, None

        hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)

        lower_red1 = (0, 120, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (170, 120, 100)
        upper_red2 = (179, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None, None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if area < 100:
            return None, None, None

        x, y, w, h = cv2.boundingRect(contour)
        centre_x = x + (w / 2.0)
        centre_y = y + (h / 2.0)

        return centre_x, centre_y, area

    def wrist_precheck(self) -> bool:
        deadline = time.time() + self.wrist_precheck_timeout
        while time.time() < deadline:
            obj_x, obj_y, area = self.wrist_obj_det()
            if obj_x is not None:
                return True
            time.sleep(0.1)
        # Object not visible at approach height — not fatal, proceed with search-cam coords
        self.get_logger().warn('Object not visible in wrist cam at approach height, proceeding with search-cam position')
        return True

    def wrist_down(self, x, y, start_z):
        self.get_logger().info(f'Wrist descent: fixed target=({x:.3f}, {y:.3f}), descending from z={start_z:.3f} to z={self.min_z:.3f}')

        self.open_gripper()

        current_x = x
        current_y = y
        current_z = start_z
    

        while current_z > self.min_z:
            obj_x, obj_y, obj_area = self.wrist_obj_det()
            if obj_x is not None:
                img_cx = self.wrist_img.shape[1] / 2.0
                img_cy = self.wrist_img.shape[0] / 2.0
                current_x = obj_x
                current_y = obj_y
                if abs(obj_x - img_cx) < 0.1 and abs(obj_y - img_cy) < 0.1:
                    self.get_logger().info(f'Grasp trigger at z={current_z:.3f}')
                    return True

            next_z = max(self.min_z, current_z - self.wrist_step)
            pose = self.create_pose(current_x, current_y, next_z)
            if not self.move_to_pose(pose):
                self.get_logger().warn(f'Move to z={next_z:.3f} failed')
                return False
            current_z = next_z
            time.sleep(0.1)

        self.get_logger().info(f'Reached max z descent')
        return True

    def object_callback(self, msg):
        if self.busy:
            return

        self.error = 0.02

        if msg.x > 0.1:
            x_d = 1
        elif -0.1 < msg.x < 0.1:
            x_d = 0
        else:
            x_d = -1

        if msg.y > 0.1:
            y_d = 1
        elif -0.1 < msg.y < 0.1:
            y_d = 0
        else:
            y_d = -1

        object_x = msg.x + (self.error * x_d)
        object_y = msg.y + (self.error * y_d)

        dist = math.hypot(object_x, object_y)
        if dist > 2.0 or dist < 0.15:
            self.get_logger().info(f'Position outside reach: x={object_x:.3f}, y={object_y:.3f} (dist={dist:.3f}m)')
            return

        self._set_busy(True)
        self.get_logger().info(f'Received object position: x={object_x:.3f}, y={object_y:.3f}')

        scan_ok = self.move_to_scan_pose()
        if not scan_ok:
            self._set_busy(False)
            return
        time.sleep(1.0)

        self.open_gripper()
        time.sleep(0.5)

        approach_pose = self.create_pose(object_x, object_y, self.approach_height)
        self.get_logger().info('Moving above object to approach pose')
        success = self.move_to_pose(approach_pose)
        if not success:
            self.move_to_scan_pose()
            self._set_busy(False)
            return

        

        if not self.wrist_precheck():
            self.move_to_scan_pose()
            self._set_busy(False)
            return
        
        self.get_logger().info('wrist descending')
        success = self.wrist_down(object_x, object_y, self.approach_height)
        if not success:
            self.get_logger().error('wrist descent failed')
            self.move_to_scan_pose()
            self._set_busy(False)
            return

        self.get_logger().info('closing gripper')
        self.close_gripper()
        time.sleep(1.5)

        self.get_logger().info('lifting object')
        lift_pose = self.create_pose(object_x, object_y, self.approach_height)
        success = self.move_to_pose(lift_pose)
        if not success:
            self.get_logger().error('lift failed')
            self.open_gripper()
            time.sleep(0.5)
            self.move_to_scan_pose()
            self._set_busy(False)
            return

        drop_pose = self.create_pose(self.basket_x, self.basket_y, self.drop)
        success = self.move_to_pose(drop_pose)
        if not success:
            self.get_logger().error('failed to reach basket')
            self.open_gripper()
            time.sleep(0.5)
            self.move_to_scan_pose()
            self._set_busy(False)
            return

        self.open_gripper()
        time.sleep(1.0)

        self.move_to_scan_pose()

        self._set_busy(False)
        self.get_logger().info('Ready for next object.')


    def create_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Point end-effector straight down towards ground
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def move_to_pose(self, pose):
        time.sleep(0.2)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=self.end_effector_link)
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error('Planning failed')
            return False

        self.get_logger().info('Executing trajectory...')
        self.moveit.execute(plan_result.trajectory, controllers=['arm_controller'])
        return True

    def open_gripper(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 1
        point.velocities = [0.0, 0.0]
        trajectory.points.append(point)
        self.gripper_pub.publish(trajectory)
        time.sleep(1.0)

    def close_gripper(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [-0.024, 0.024]
        point.time_from_start.sec = 1
        point.velocities = [0.0, 0.0]
        trajectory.points.append(point)
        self.gripper_pub.publish(trajectory)
        time.sleep(1.0)


def main():
    rclpy.init()
    node = PickupController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
