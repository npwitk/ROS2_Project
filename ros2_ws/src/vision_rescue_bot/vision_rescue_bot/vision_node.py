#!/usr/bin/env python3
"""
Basic ROS2 Vision Node for Rescue Robot

This node demonstrates:
1. Subscribing to camera images
2. Processing images with OpenCV (like Core Image filters)
3. Publishing results (like delegation pattern)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionRescueNode(Node):
    
    def __init__(self):
        # Like calling super.viewDidLoad() in iOS
        super().__init__('vision_rescue_node')
        
        # Initialize the CV Bridge (converts between ROS2 and OpenCV formats)
        # This is like a data transformer in iOS
        self.bridge = CvBridge()
        
        # Create subscribers (like NotificationCenter observers)
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic name - like a notification name
            self.image_callback,  # Callback function - like selector method
            10  # Queue size - like buffer size
        )
        
        # Create publishers (like delegation or NotificationCenter posting)
        self.victim_detection_pub = self.create_publisher(
            Bool, 
            '/vision/victim_detected', 
            10
        )
        
        self.status_pub = self.create_publisher(
            String, 
            '/vision/status', 
            10
        )
        
        self.frame_count = 0
        self.detection_threshold = 100  # Minimum area for detection
        
        # Log startup (like print statements in viewDidLoad)
        self.get_logger().info('Vision Rescue Node started - ready for camera input!')
        
        # Publish initial status
        self.publish_status("Vision system initialized")
    
    def image_callback(self, msg):
        """
        Main image processing function - like handling a UIImage in iOS 555
        This is called every time a new image arrives (like delegate method)
        """
        try:
            # Convert ROS2 image message to OpenCV format
            # Like converting between UIImage and CGImage
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the image for rescue scenarios
            victim_detected = self.detect_potential_victim(cv_image)
            
            # Publish results (like calling delegate methods)
            self.victim_detection_pub.publish(Bool(data=victim_detected))
            
            # Update frame counter and log progress
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Every 30 frames (like reducing log spam)
                self.publish_status(f"Processed {self.frame_count} frames")
                self.get_logger().info(f"Processed {self.frame_count} frames")
            
            # Optional: Display image for debugging
            # cv2.imshow('Rescue Vision Debug', processed_image)
            # cv2.waitKey(1)
            
        except Exception as e:
            # Error handling (like try-catch in Swift)
            self.get_logger().error(f'Error processing image: {str(e)}')
            self.publish_status(f"Error: {str(e)}")
    
    def detect_potential_victim(self, cv_image):
        """
        Basic victim detection using color detection
        In real rescue scenarios, this would be much more sophisticated
        
        For now, we'll detect human-skin-like colors as a starting point
        """
        
        # Convert to HSV color space (better for color detection)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for human skin color in HSV
        # This is a simplified approach - real systems use thermal imaging
        lower_skin = np.array([0, 20, 70])
        upper_skin = np.array([20, 255, 255])
        
        # Create mask for skin-like colors
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        # Find contours (shapes) in the mask
        contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check if we found any significant areas
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.detection_threshold:
                # Found something that might be a person
                self.get_logger().info(f'Potential victim detected! Area: {area}')
                return True
        
        return False
    
    def publish_status(self, message):
        """
        Helper method to publish status messages
        """
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)

def main(args=None):
    # Initialize ROS2 (like UIKit initialization)
    rclpy.init(args=args)
    
    # Create our node instance (like creating view controller)
    vision_node = VisionRescueNode()
    
    try:
        # Start the event loop (like UIApplication.main())
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully (like applicationWillTerminate)
        pass
    finally:
        # Cleanup (like viewDidUnload or deinit)
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
