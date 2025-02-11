import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from visualization_msgs.msg import Marker, MarkerArray
import math

# Parameters
MAX_REACH = 0.28  # Maximum reach (28 cm)
FIXED_BASE_HEIGHT = 0.08945  # Fixed base height of the robot in meters

class PX100PointingNode(Node):
    def __init__(self):
        super().__init__('px100_pointing_node')

        # Initialize the robot
        self.bot = InterbotixManipulatorXS(
            robot_model='mobile_px100',
            group_name='arm',
            gripper_name='gripper',
        )

        # Publisher for the target marker in RViz (MarkerArray)
        # self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # self.get_logger().info('PX100PointingNode has been started.')
        # joint_names = self.bot.arm.group_info.joint_names
        # self.get_logger().info(f"Current joint names: {joint_names}")

    # Checks if the target point is within the robot's maximum reach. If not, it scales the target position to fit within the maximum reach while maintaining the direction.
    def adjust_target_to_reach_limit(self, x, y, z):
        # Calculate distance relative to the base height
        z_relative = z - FIXED_BASE_HEIGHT
        distance = math.sqrt(x**2 + y**2 + z_relative**2)
        self.get_logger().info(f"Target distance: {distance:.3f} m")

        # If the point is reachable, return it as-is
        if distance <= MAX_REACH:
            return x, y, z

        # Adjust the target to fit within the maximum reach
        self.get_logger().warn(f"Target point is out of reach (distance: {distance:.2f} m). Adjusting to maximum reach ({MAX_REACH:.2f} m).")

        scale = MAX_REACH / distance
        adjusted_x = x * scale
        adjusted_y = y * scale
        adjusted_z = FIXED_BASE_HEIGHT + z_relative * scale

        self.get_logger().info(f"Adjusted pose: x={adjusted_x:.3f}, y={adjusted_y:.3f}, z={adjusted_z:.3f}")
        return adjusted_x, adjusted_y, adjusted_z

    # Publishes a marker at the target position in RViz using MarkerArray
    # def publish_target_marker(self, x, y, z):
    #     marker_array = MarkerArray()
    #     marker = Marker()
    #     marker.header.frame_id = "base_link"  # Reference frame
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "target_point"
    #     marker.id = 0
    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD

    #     # Position of the point in 3D space
    #     marker.pose.position.x = x
    #     marker.pose.position.y = y
    #     marker.pose.position.z = z

    #     # Orientation (default)
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0

    #     # Size of the point (in meters)
    #     marker.scale.x = 0.05
    #     marker.scale.y = 0.05
    #     marker.scale.z = 0.05

    #     # Color (red)
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0  # 

        # Add marker to the array and publish it
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    # Moves the robot's end-effector to the specified target position. If the target is out of reach, it adjusts the position and attempts to move to the reachable point.
    def point_to_target(self, x, y, z, yaw=0.0):
        self.get_logger().info(f"Setting target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Publish the target marker for visualization in RViz
        #self.publish_target_marker(x, y, z)

        # Adjust the target point if necessary
        adjusted_x, adjusted_y, adjusted_z = self.adjust_target_to_reach_limit(x, y, z)

        # Move the robot to the home position before executing the move
        self.get_logger().info("Moving robot to home pose...")
        self.bot.arm.go_to_home_pose()

        # Attempt to move to the adjusted pose
        self.get_logger().info(f"Moving to adjusted pose: x={adjusted_x:.3f}, y={adjusted_y:.3f}, z={adjusted_z:.3f}, yaw={yaw:.2f}")
        success = self.bot.arm.set_ee_pose_components(x=adjusted_x, y=adjusted_y, z=adjusted_z, yaw=yaw)

        if success:
            self.get_logger().info(f"Successfully moved to target pose: x={adjusted_x:.3f}, y={adjusted_y:.3f}, z={adjusted_z:.3f}")
        else:
            self.get_logger().error("Failed to reach the target pose!")

def main(args=None):
    rclpy.init(args=args)
    node = PX100PointingNode()

    # Example of an unreachable target
    # node.point_to_target(0.3, 0.3, 0.4, yaw=0.0)

    # Example of a reachable target 
    node.point_to_target(-0.05, 0.10, 0.15, yaw=0.0)



    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
