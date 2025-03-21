#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray, String
from geometry_msgs.msg import Twist, Point
import time

class WeedActionNode:
    def __init__(self):
        rospy.init_node('weed_action_node')
        
        # Robot state
        self.weed_positions = []
        self.current_action = None
        self.robot_stopped = True
        
        # Parameters
        self.min_weeds_for_action = rospy.get_param('~min_weeds', 3)
        self.action_duration = rospy.get_param('~action_duration', 2.0)  # seconds
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sprayer_pub = rospy.Publisher('/sprayer_control', String, queue_size=10)
        self.status_pub = rospy.Publisher('/weed_action/status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/weed_crop_detection', Int32MultiArray, self.detection_callback)
        
        # Timer for action loop
        rospy.Timer(rospy.Duration(0.2), self.action_loop)
        
        rospy.loginfo("Weed action node initialized")
    
    def detection_callback(self, msg):
        # Extract weed count
        crop_count, weed_count = msg.data
        
        if weed_count >= self.min_weeds_for_action and not self.current_action:
            # Store position and initiate action
            self.weed_positions.append(Point())  # Would use actual position in real system
            self.current_action = {
                'type': 'spray',
                'start_time': rospy.Time.now(),
                'duration': rospy.Duration(self.action_duration)
            }
            
            # Stop the robot during spraying
            self.stop_robot()
            
            # Log action
            rospy.loginfo(f"Initiating weed control action for {weed_count} weeds")
            self.status_pub.publish(f"Starting spraying: {weed_count} weeds detected")
    
    def stop_robot(self):
        if not self.robot_stopped:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.robot_stopped = True
    
    def resume_robot(self):
        # In a real system, you would resume the previous movement
        # This is simplified
        self.robot_stopped = False
        self.status_pub.publish("Action complete, resuming navigation")
    
    def action_loop(self, event):
        if not self.current_action:
            return
        
        action = self.current_action
        now = rospy.Time.now()
        
        # Check if action is still in progress
        if now - action['start_time'] <= action['duration']:
            # Continue the action
            if action['type'] == 'spray':
                # Send command to activate sprayer
                self.sprayer_pub.publish("ON")
        else:
            # Action is complete
            if action['type'] == 'spray':
                # Stop sprayer
                self.sprayer_pub.publish("OFF")
            
            # Clear current action
            self.current_action = None
            
            # Resume movement
            self.resume_robot()

if __name__ == '__main__':
    try:
        node = WeedActionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass