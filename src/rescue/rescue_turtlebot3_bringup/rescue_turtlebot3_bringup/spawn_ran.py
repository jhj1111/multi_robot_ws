import os
import rclpy
import random
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

def main():
    # Start node
    sdf_pose_x = random.uniform(1.0, 3.0)
    sdf_pose_y = random.uniform(1.0, 3.0)
    sdf_pose_z = 0.1

    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    # script_dir = os.path.dirname(os.path.realpath(__file__))
    # sdf_file_path = os.path.join(script_dir, "models/model.sdf") 
    script_dir = os.path.join(get_package_share_directory('rescue_turtlebot3_bringup'), 'models')
    sdf_file_path = os.path.join(script_dir, "model.sdf") 

    # Set data for request
    request = SpawnEntity.Request()
    request.name = "red"
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = "red"
    request.initial_pose.position.x = float(sdf_pose_x)
    request.initial_pose.position.y = float(sdf_pose_y)
    request.initial_pose.position.z = float(sdf_pose_z)

    node.get_logger().info("Sending service request to `/spawn_entity`")

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()