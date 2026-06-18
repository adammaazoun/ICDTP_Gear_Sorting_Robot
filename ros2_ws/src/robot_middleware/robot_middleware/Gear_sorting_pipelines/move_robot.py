#!/usr/bin/env python3
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class RobotMover:

    def __init__(self, node):
        """
        node = ROS2 node (InspectionNode)
        """
        self.node = node

        self.traj_pub = node.create_publisher(
            JointTrajectory, '/planned_trajectory', 10)

        self.cmd_pub = node.create_publisher(
            String, '/stm32_cmd', 10)

        self.joint_names = [
            'joint_basis_arm1',
            'joint_arm1_arm2',
            'joint_arm2_arm3',
            'joint_arm3_greifer'
        ]

        self.current_positions = [0.0, 0.0, 0.0, 0.0]

        node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self._wait_for_connection()

    # ─────────────────────────────────────────────
    # Connection check (NO SPIN)
    # ─────────────────────────────────────────────
    def _wait_for_connection(self):
        self.node.get_logger().info("Waiting for subscribers...")

        start = time.time()
        while True:
            traj_ready = self.traj_pub.get_subscription_count() > 0
            cmd_ready  = self.cmd_pub.get_subscription_count() > 0

            if traj_ready and cmd_ready:
                self.node.get_logger().info("Publishers connected.")
                return

            if time.time() - start > 10.0:
                self.node.get_logger().warn("Timeout waiting for subscribers")
                return

            time.sleep(0.1)

    # ─────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        self.current_positions = [
            pos_map.get(n, 0.0) for n in self.joint_names
        ]

    # ─────────────────────────────────────────────
    def wait_for_motion(self, duration=3.5):
        self.node.get_logger().info(
            f"Waiting {duration}s for motion to complete..."
        )
        time.sleep(duration)

    # ─────────────────────────────────────────────
    def send_trajectory(self, waypoints: list, duration_sec=2):

        points = []

        p0 = JointTrajectoryPoint()
        p0.positions = list(self.current_positions)
        p0.time_from_start.sec = 0
        points.append(p0)

        for i, pos in enumerate(waypoints):
            p = JointTrajectoryPoint()
            p.positions = pos
            p.time_from_start.sec = duration_sec * (i + 1)
            points.append(p)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points = points

        self.traj_pub.publish(traj)

        self.node.get_logger().info(
            f"Trajectory sent with {len(points)} points"
        )

    # ─────────────────────────────────────────────
    def send_gripper(self, state: str):
        msg = String()
        msg.data = state
        self.cmd_pub.publish(msg)
        self.node.get_logger().info(f"Gripper: {state}")

    # ─────────────────────────────────────────────
    def move_to_station(self):
        self.node.get_logger().info("Moving to INSPECTION STATION")

        target = [-3.5, -1.0, -1.0, 0.0]
        cur = self.current_positions

        self.send_trajectory([
            [cur[0], -0.5, -0.5, cur[3]],
            [target[0], -0.5, -0.5, target[3]],
            target
        ])

        self.wait_for_motion(10)
        self.send_gripper("grip_close")
        self.wait_for_motion(5)

    # ─────────────────────────────────────────────
    def move_robot_to_good_bin(self):
        self.node.get_logger().info("Moving to GOOD bin")

        target = [-5.0, -1.0, -0.8, 0.0]
        cur = self.current_positions

        self.send_trajectory([
            [cur[0], -0.5, -0.5, cur[3]],
            [target[0], -0.5, -0.5, target[3]],
            target
        ])

        self.wait_for_motion(10)
        self.send_gripper("grip_open")
        self.wait_for_motion(5)

    # ─────────────────────────────────────────────
    def move_robot_to_reject_bin(self):
        self.node.get_logger().info("Moving to REJECT bin")

        target = [-5.5, -1.0, -0.9, -0.5]
        cur = self.current_positions

        self.send_trajectory([
            [cur[0], -0.5, -0.5, cur[3]],
            [target[0], -0.5, -0.5, target[3]],
            target
        ])

        self.wait_for_motion(10)
        self.send_gripper("grip_open")
        self.wait_for_motion(5)