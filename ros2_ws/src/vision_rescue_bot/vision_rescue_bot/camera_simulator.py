#!/usr/bin/env python3
"""
Camera Simulator Node
This simulates camera input for testing your vision node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # Publisher for simulated camera images
        self.image_publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        # Timer to publish images regularly (like NSTimer in iOS)
        self.timer = self.create_timer(0.1, self.publish_test_image)  # 10 FPS
        
        self.bridge = CvBridge()
        self.frame_counter = 0
        
        self.get_logger().info('Camera simulator started - publishing test images')
    
    def publish_test_image(self):
        """
        Creates and publishes test images
        This simulates what a real camera would send
        """
        
        # Create a test image (640x480 is common camera resolution)
        height, width = 480, 640
        
        # Create different test patterns to simulate various scenarios
        if self.frame_counter % 100 < 30:
            # Simulate "empty" scene (blue background)
            test_image = np.zeros((height, width, 3), dtype=np.uint8)
            test_image[:, :] = [100, 50, 0]  # Dark blue
            
        elif self.frame_counter % 100 < 60:
            # Simulate scene with skin-colored object (potential victim)
            test_image = np.zeros((height, width, 3), dtype=np.uint8)
            test_image[:, :] = [50, 50, 50]  # Dark gray background
            
            # Add a skin-colored rectangle (simulating a person)
            cv2.rectangle(test_image, (200, 150), (400, 350), (132, 165, 210), -1)  # Skin color in BGR
            
        else:
            # Simulate debris/rubble scene
            test_image = np.random.randint(0, 100, (height, width, 3), dtype=np.uint8)
        
        # Add frame number as text (for debugging)
        cv2.putText(test_image, f'Frame: {self.frame_counter}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Convert OpenCV image to ROS2 message format
        try:
            image_msg = self.bridge.cv2_to_imgmsg(test_image, 'bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            # Publish the image
            self.image_publisher.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
        
        self.frame_counter += 1

def main(args=None):
    rclpy.init(args=args)
    simulator = CameraSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
