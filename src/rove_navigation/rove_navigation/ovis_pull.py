import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
import tf2_ros


class PositionLockNode(Node):
    def __init__(self):
        super().__init__("position_lock_node")

        self.declare_parameter("rove_name", "rove")
        self.declare_parameter("ovis_name", "ovis")
        self.declare_parameter("update_rate", 10.0)  # Hz
        self.declare_parameter("offset_x", 0.25)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.25)

        self.rove_name = (
            self.get_parameter("rove_name").get_parameter_value().string_value
        )
        self.ovis_name = (
            self.get_parameter("ovis_name").get_parameter_value().string_value
        )
        self.update_rate = (
            self.get_parameter("update_rate").get_parameter_value().double_value
        )
        self.offset_x = (
            self.get_parameter("offset_x").get_parameter_value().double_value
        )
        self.offset_y = (
            self.get_parameter("offset_y").get_parameter_value().double_value
        )
        self.offset_z = (
            self.get_parameter("offset_z").get_parameter_value().double_value
        )

        # Set up the TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up the SetEntityPose service client
        self.set_entity_pose_client = self.create_client(
            SetEntityPose, "/world/default/set_pose"
        )
        while not self.set_entity_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for SetEntityPose service...")

        # Timer to periodically update the position
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_position)

    def update_position(self):
        try:
            # Look up the latest transform for `rove`
            transform = self.tf_buffer.lookup_transform(
                "world", "base_link", rclpy.time.Time()
            )
            # Calculate the target pose for `ovis` with the given offsets
            target_pose = Pose()
            target_pose.position.x = transform.transform.translation.x + self.offset_x
            target_pose.position.y = transform.transform.translation.y + self.offset_y
            target_pose.position.z = transform.transform.translation.z + self.offset_z

            # Use the quaternion orientation from `rove` directly
            target_pose.orientation = transform.transform.rotation

            # Set the new pose of `ovis`
            set_pose_request = SetEntityPose.Request()
            set_pose_request.name = self.ovis_name
            set_pose_request.pose = target_pose
            future = self.set_entity_pose_client.call_async(set_pose_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None or not future.result().success:
                self.get_logger().error(f"Failed to set pose of {self.ovis_name}")
            else:
                self.get_logger().info(f"Successfully updated pose of {self.ovis_name}")
        except Exception as e:
            self.get_logger().error(f"Error retrieving transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PositionLockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
