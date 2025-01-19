#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2  # Add this for visualization during debugging

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        
        # Initialize YOLO
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            
        self.bridge = CvBridge()
        
        # Create subscriber for image
        self.image_sub = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10
        )
        self.get_logger().info('Subscribed to /rgb topic')
        
        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/human_detections',
            10
        )
        
    def image_callback(self, msg):
        
        try:
            # Convert ROS Image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            
            # Run detection
            results = self.model(cv_image)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Process results
            for r in results[0].boxes:
                class_id = int(r.cls[0])
                
                if class_id == 0:  # person class
                    detection = Detection2D()
                    # Convert tensor to float
                    bbox = r.xyxy[0].cpu().numpy()  # Convert to numpy array
                    detection.bbox.center.position.x = (float(bbox[0]) + float(bbox[2])) / 2
                    detection.bbox.center.position.y = (float(bbox[1]) + float(bbox[3])) / 2
                    detection.bbox.size_x = float(bbox[2]) - float(bbox[0])
                    detection.bbox.size_y = float(bbox[3]) - float(bbox[1])
                    
                    # Add confidence score
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = "person"
                    hypothesis.hypothesis.score = float(r.conf[0])
                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)
                    
                
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = HumanDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()