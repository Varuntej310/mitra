#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix
import math

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')
        
        # Robot state
        self.position = {'x': 0, 'y': 0, 'theta': 0}
        self.gps_position = {'lat': 0, 'lon': 0}
        self.weed_detections = []
        self.current_path = []
        self.autonomous_mode = False
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/weed_crop_detection', Int32MultiArray, self.detection_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Service for mode switching could be added here
        
        # Parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s
        self.row_spacing = rospy.get_param('~row_spacing', 0.5)  # meters
        
        # Timer for control loop
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Navigation node initialized")
    
    def odom_callback(self, msg):
        # Update robot position from odometry
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # Extract orientation (yaw) from quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position['theta'] = euler[2]  # yaw
    
    def gps_callback(self, msg):
        # Update GPS position
        self.gps_position['lat'] = msg.latitude
        self.gps_position['lon'] = msg.longitude
    
    def detection_callback(self, msg):
        # Process detection results
        crop_count, weed_count = msg.data
        
        if weed_count > 0:
            # Store current position if weeds detected
            # (In a real system, you'd also store the specific locations of weeds)
            self.weed_detections.append({
                'x': self.position['x'],
                'y': self.position['y'],
                'count': weed_count
            })
    
    def goal_callback(self, msg):
        # Handle new navigation goal
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Plan path to goal (simplified)
        self.plan_path(goal_x, goal_y)
        
        # Enable autonomous mode
        self.autonomous_mode = True
    
    def plan_path(self, goal_x, goal_y):
        # Simplified path planning - just a straight line to goal
        # In a real system, you'd implement A*, RRT, or use ROS's navigation stack
        start_x = self.position['x']
        start_y = self.position['y']
        
        # Create a simple direct path
        self.current_path = []
        
        # Number of waypoints
        num_points = 10
        
        for i in range(num_points + 1):
            waypoint = {
                'x': start_x + (goal_x - start_x) * i / num_points,
                'y': start_y + (goal_y - start_y) * i / num_points
            }
            self.current_path.append(waypoint)
        
        # Publish path for visualization
        self.publish_path()
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for waypoint in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            pose.pose.position.z = 0
            
            # Set orientation to identity quaternion
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def control_loop(self, event):
        if not self.autonomous_mode or not self.current_path:
            return
        
        # Get the next waypoint
        target = self.current_path[0]
        
        # Calculate distance and angle to target
        dx = target['x'] - self.position['x']
        dy = target['y'] - self.position['y']
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Angle difference
        angle_diff = target_angle - self.position['theta']
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create velocity command
        cmd = Twist()
        
        # If we need to rotate significantly, do that first
        if abs(angle_diff) > 0.1:
            cmd.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
        else:
            # Otherwise, move forward and make minor angle adjustments
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed * angle_diff
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # If we're close to the waypoint, remove it and move to the next
        if distance < 0.1:
            self.current_path.pop(0)
            
            # If path is complete, stop autonomous mode
            if not self.current_path:
                self.autonomous_mode = False
                
                # Send stop command
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass