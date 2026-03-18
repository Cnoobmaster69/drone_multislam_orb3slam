#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node

# Import the service type from slam_msgs package
from slam_msgs.srv import GetGlobalPointCloud


class GlobalPointCloudClient(Node):
    def __init__(self):
        super().__init__('global_point_cloud_client')

        self.declare_parameter('service_name', 'trigger_global_cloud')
        self.declare_parameter('max_attempts', 8)
        self.declare_parameter('retry_delay_sec', 2.0)
        self.declare_parameter('global_voxel_resolution', 0.05)
        self.declare_parameter('local_voxel_resolution', 0.05)
        self.declare_parameter('z_thresh_max', 50.0)
        self.declare_parameter('get_grayscale', False)
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        self.global_voxel_resolution = float(self.get_parameter('global_voxel_resolution').value)
        self.local_voxel_resolution = float(self.get_parameter('local_voxel_resolution').value)
        self.z_thresh_max = float(self.get_parameter('z_thresh_max').value)
        self.get_grayscale = bool(self.get_parameter('get_grayscale').value)
        self.service_name = self._resolve_service_name(service_name)

        # Create a client for the resolved service name
        self.client = self.create_client(GetGlobalPointCloud, self.service_name)
        self.get_logger().info(f'Using service: "{self.service_name}"')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service "{self.service_name}" not available, waiting...')

        self.get_logger().info(f'Service "{self.service_name}" is available.')

    def _resolve_service_name(self, service_name: str) -> str:
        if service_name.startswith('/'):
            return service_name
        ns = self.get_namespace().strip('/')
        if ns:
            return f'/{ns}/{service_name}'
        return f'/{service_name}'

    def send_request(self, request_params):
        """Send a request to the 'trigger_global_cloud' service."""
        req = GetGlobalPointCloud.Request()

        # Populate the request fields with provided parameters
        req.global_voxel_resolution = request_params.get('global_voxel_resolution', 0.1)
        req.local_voxel_resolution = request_params.get('local_voxel_resolution', 0.05)
        req.z_thresh_max = request_params.get('z_thresh_max', 1.5)
        req.get_grayscale = request_params.get('get_grayscale', False)

        self.get_logger().info(f'Sending request to "{self.service_name}" service...')
        future = self.client.call_async(req)
        return future


def main(args=None):
    rclpy.init(args=args)

    client_node = GlobalPointCloudClient()

    # Define request parameters
    # !!!!! EDIT THESE IF NEEDED !!!!! 
    request_parameters = {
        'global_voxel_resolution': client_node.global_voxel_resolution,
        'local_voxel_resolution': client_node.local_voxel_resolution,
        'z_thresh_max': client_node.z_thresh_max,
        'get_grayscale': client_node.get_grayscale
    }

    success = False
    for attempt in range(1, client_node.max_attempts + 1):
        client_node.get_logger().info(
            f'Trigger attempt {attempt}/{client_node.max_attempts}'
        )
        future = client_node.send_request(request_parameters)

        while rclpy.ok():
            rclpy.spin_once(client_node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    client_node.get_logger().error(f'Service call failed: {e}')
                    response = None

                if response is not None and response.response:
                    client_node.get_logger().info('Global point cloud triggered successfully.')
                    success = True
                else:
                    client_node.get_logger().warn('Trigger returned false; retrying if attempts remain.')
                break

        if success:
            break
        if attempt < client_node.max_attempts:
            time.sleep(client_node.retry_delay_sec)

    # Shutdown the node
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
