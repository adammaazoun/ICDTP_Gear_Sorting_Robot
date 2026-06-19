import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from my_robot_interfaces.msg import RobotCommand
import time

class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        # Create publisher on /robot_cmd topic
        self.publisher_ = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        
        # Wait a bit to ensure publisher connection is established
        time.sleep(1.0)
        self.run_sequence()

    def send_move(self, x, y, z):
        msg = RobotCommand()
        msg.base_id = "unity_web_client"
        
        # Set target position (x, y, z)
        msg.target_pose.position.x = float(x)
        msg.target_pose.position.y = float(y)
        msg.target_pose.position.z = float(z)
        
        # Set target orientation (default quaternion, pitch=0)
        msg.target_pose.orientation.x = 0.0
        msg.target_pose.orientation.y = 0.0
        msg.target_pose.orientation.z = 0.0
        msg.target_pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending movement command: pos=({x}, {y}, {z})')
        self.publisher_.publish(msg)

    def send_gripper(self, open_gripper):
        msg = RobotCommand()
        # Special command bypassing MoveIt planning
        msg.base_id = "GRIP_OPEN" if open_gripper else "GRIP_CLOSE"
        
        # Set default orientation for safety
        msg.target_pose.orientation.w = 1.0
        msg.gripper_state = open_gripper
        
        state = "OPEN" if open_gripper else "CLOSED"
        self.get_logger().info(f'Sending gripper command: {state} (base_id: {msg.base_id})')
        self.publisher_.publish(msg)

    def send_reset(self):
        msg = RobotCommand()
        # Triggers StallGuard homing for all 5 motors
        msg.base_id = "RESET"
        msg.target_pose.orientation.w = 1.0
        
        self.get_logger().info('Sending initialization command (RESET / StallGuard Homing)...')
        self.publisher_.publish(msg)

    def run_sequence(self):
        # Point X (where the gear/part is located)
        point_x = (0.2, 0.0, 0.1) 
        
        # Point Z (where we want to drop the part)
        point_z = (0.0, 0.2, 0.1) 
        
        # Safe/rest position
        safe_pos = (0.15, 0.0, 0.25)
        
        # 0. Initialization (Homing all motors)
        self.send_reset()
        time.sleep(5.0) # Wait for StallGuard homing to complete
        
        # 1. Move to safe position and open the gripper
        self.get_logger().info("1. Gripper opening and safe position move...")
        self.send_move(safe_pos[0], safe_pos[1], safe_pos[2])
        time.sleep(2.0)
        self.send_gripper(open_gripper=True)
        time.sleep(2.0)
        
        # 2. Move to Point X (part pick-up location) with gripper open
        self.get_logger().info("2. Moving to Point X (pick-up part)...")
        self.send_move(point_x[0], point_x[1], point_x[2])
        time.sleep(2.0) # Wait for the robot to reach position
        
        # 3. Close the gripper to grasp the part
        self.get_logger().info("3. Closing gripper to grasp the part...")
        self.send_gripper(open_gripper=False)
        time.sleep(2.0)
        
        # 4. Lift the part before moving to avoid collisions
        self.get_logger().info("4. Lifting the part to avoid collisions...")
        self.send_move(point_x[0], point_x[1], safe_pos[2])
        time.sleep(1.5)
        
        # 5. Move above Point Z (deposit location) with gripper closed
        self.get_logger().info("5. Moving above Point Z...")
        self.send_move(point_z[0], point_z[1], safe_pos[2])
        time.sleep(2.0)
        
        # 6. Lower to Point Z
        self.get_logger().info("6. Lowering to Point Z...")
        self.send_move(point_z[0], point_z[1], point_z[2])
        time.sleep(1.5)
        
        # 7. Open the gripper to release the part
        self.get_logger().info("7. Opening gripper to release the part...")
        self.send_gripper(open_gripper=True)
        time.sleep(2.0)
        
        # 8. Return to safe position
        self.get_logger().info("8. End of sequence. Returning to safe position.")
        self.send_move(safe_pos[0], safe_pos[1], safe_pos[2])
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    demo_node = PickAndPlaceDemo()
    # Destroy the node once the synchronous sequence finishes
    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
