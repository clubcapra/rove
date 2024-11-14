import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class ModelAttacherNode(Node):
    def __init__(self):
        super().__init__('model_attacher_node')
        
        self.declare_parameter('parent_model', 'rove')
        self.declare_parameter('child_model', 'ovis')
        self.declare_parameter('parent_link', 'base_link')
        self.declare_parameter('child_link', 'base_link')
        
        self.parent_model = self.get_parameter('parent_model').get_parameter_value().string_value
        self.child_model = self.get_parameter('child_model').get_parameter_value().string_value
        self.parent_link = self.get_parameter('parent_link').get_parameter_value().string_value
        self.child_link = self.get_parameter('child_link').get_parameter_value().string_value

        # Create service client
        self.spawn_entity_client = self.create_client(
            SpawnEntity, 
            '/world/default/create'
        )
        
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpawnEntity service...')
            
        self.attach_models()

    def attach_models(self):
        # Create an SDF string for a fixed joint
        joint_sdf = f"""
        <?xml version="1.0" ?>
        <sdf version="1.8">
            <model name="joint_attachment">
                <joint name="rove_ovis_joint" type="fixed">
                    <parent>{self.parent_model}::{self.parent_link}</parent>
                    <child>{self.child_model}::{self.child_link}</child>
                </joint>
            </model>
        </sdf>
        """
        
        request = SpawnEntity.Request()
        request.xml = joint_sdf
        request.name = "joint_attachment"
        
        future = self.spawn_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Successfully created joint between models')
        else:
            self.get_logger().error('Failed to create joint between models')

def main(args=None):
    rclpy.init(args=args)
    node = ModelAttacherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 