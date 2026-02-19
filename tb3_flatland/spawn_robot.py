#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from flatland_msgs.srv import SpawnModel
import time


class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        
        self.declare_parameter('robot_yaml', '')
        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('robot_ns', 'robot')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('theta', 0.0)
        
        self.robot_yaml = self.get_parameter('robot_yaml').value
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_ns = self.get_parameter('robot_ns').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.theta = self.get_parameter('theta').value
        
        self.spawn_done = False
        
        self.get_logger().info(f'Spawning {self.robot_name} at ({self.x}, {self.y}, {self.theta})')
        self.get_logger().info(f'YAML: {self.robot_yaml}')
        
        self.client = self.create_client(SpawnModel, '/spawn_model')
        
        retry_count = 0
        max_retries = 30
        
        # Checking service availability
        while not self.client.wait_for_service(timeout_sec=1.0):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error('/spawn_model service is not available after 30 attempts!')
                self.spawn_done = True 
                return
            self.get_logger().info(f'Waiting for /spawn_model service... (attempt {retry_count}/{max_retries})')
        
        self.send_request()
    
    def send_request(self):
        request = SpawnModel.Request()
        request.yaml_path = self.robot_yaml
        request.name = self.robot_name
        request.ns = self.robot_ns
        request.pose.x = self.x
        request.pose.y = self.y
        request.pose.theta = self.theta
        
        self.get_logger().info(f'Sending spawn request...')
        self.get_logger().info(f'  Name: {request.name}')
        self.get_logger().info(f'  Namespace: {request.ns}')
        self.get_logger().info(f'  Position: ({request.pose.x}, {request.pose.y}, {request.pose.theta})')
        
        # Async call to prevent blocking
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ {self.robot_name} successfully spawned!')
            else:
                self.get_logger().error(f'✗ Failed to spawn {self.robot_name}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'✗ Exception occurred during spawn: {str(e)}')
        finally:
            self.spawn_done = True


def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    try:
        # Spin until the spawn process is complete
        while rclpy.ok() and not spawner.spawn_done:
            rclpy.spin_once(spawner, timeout_sec=0.1)
        time.sleep(0.5)
    except KeyboardInterrupt:
        spawner.get_logger().info('Spawn process interrupted by user')
    finally:
        spawner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()