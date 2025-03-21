#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO

class DetectionNode:
    def __init__(self):
        rospy.init_node('detection_node')
        
        # Load YOLO model
        self.model = YOLO('best.pt')
        self.class_names = ['crop', 'weed']
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = rospy.Publisher('/weed_crop_detection', Int32MultiArray, queue_size=10)
        self.detection_image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Detection node initialized")
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform detection
            results = self.model(cv_image)
            
            # Process results
            counts = {'crop': 0, 'weed': 0}
            annotated_img = cv_image.copy()
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    class_name = self.class_names[cls]
                    counts[class_name] += 1
                    
                    # Draw bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    color = (0, 255, 0) if class_name == 'crop' else (0, 0, 255)
                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated_img, f"{class_name} {conf:.2f}", 
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish detection results
            counts_msg = Int32MultiArray()
            counts_msg.data = [counts['crop'], counts['weed']]
            self.detection_pub.publish(counts_msg)
            
            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
            self.detection_image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in detection: {e}")
    
if __name__ == '__main__':
    try:
        node = DetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass