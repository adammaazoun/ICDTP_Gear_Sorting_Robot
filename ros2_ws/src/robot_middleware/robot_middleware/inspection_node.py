#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_middleware.platform_controller import PlatformController

# Import your pipelines
from top_view_pipeline import validate_top_view
from side_view_pipeline import validate_side_view

class InspectionNode(Node):

    def __init__(self):
        super().__init__('inspection_node')


        self.get_logger().info("Inspection node started")

        self.run_inspection()


    def run_inspection(self):

        self.get_logger().info("Running top view inspection")

        top_status, top_details = validate_top_view()

        self.get_logger().info(
            f"Top View => {top_status} | {top_details}"
        )

        self.get_logger().info("Running side view inspection")

        side_status, side_details = validate_side_view()

        self.get_logger().info(
            f"Side View => {side_status} | {side_details}"
        )

        # First inspection decision
        if top_status == "Defected" or side_status == "Defected":

            self.handle_defective_part(
                top_details,
                side_details
            )

            return

        self.get_logger().info(
            "Part passed first inspection"
        )

        # Rotate platform
        self.rotate_platform()
        # Second side inspection
        self.get_logger().info(
            "Running second side inspection"
        )

        side2_status, side2_details = validate_side_view()

        self.get_logger().info(
            f"Second Side => {side2_status} | {side2_details}"
        )

        # Final decision
        if side2_status == "Defected":

            self.handle_defective_part(
                side2_details
            )

        else:

            self.handle_good_part()


    def handle_good_part(self):

        self.get_logger().info(
            "GOOD PART"
        )

        move_robot_to_good_bin()


    def handle_defective_part(self, *details):

        self.get_logger().warn(
            f"DEFECTIVE PART : {details}"
        )

        move_robot_to_reject_bin()

    def rotate_platform(self):
        self.get_logger().info("Rotating platform via ROS command")

        msg = Float32()
        msg.data = 180.0
        self.servo_pub.publish(msg)

        self.get_logger().info("Waiting for rotation...")

        rclpy.spin_once(self, timeout_sec=2.0)

        msg.data = 0.0
        self.servo_pub.publish(msg)
    


def main(args=None):

    rclpy.init(args=args)
    node = InspectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()