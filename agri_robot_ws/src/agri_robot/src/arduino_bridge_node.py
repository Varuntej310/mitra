#!/usr/bin/env python3
import rospy
import serial
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

class ArduinoBridgeNode:
    def __init__(self):
        rospy.init_node('arduino_bridge_node')
        
        # Serial connection to Arduino
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baudrate', 115200)
        
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            rospy.loginfo(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to Arduino: {e}")
            self.serial = None
            rospy.signal_shutdown("Failed to connect to Arduino")
            return
        
        # Publishers
        self.sensor_pub = rospy.Publisher('/arduino/sensors', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/arduino/status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Timer for reading from Arduino
        rospy.Timer(rospy.Duration(0.05), self.read_from_arduino)
        
        rospy.loginfo("Arduino bridge node initialized")
    
    def cmd_vel_callback(self, msg):
        if not self.serial:
            return
        
        # Convert Twist message to Arduino command
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to left and right wheel speeds (differential drive)
        # Adjust wheel_separation according to your robot
        wheel_separation = 0.3  # meters
        
        left_speed = linear_x - (angular_z * wheel_separation / 2.0)
        right_speed = linear_x + (angular_z * wheel_separation / 2.0)
        
        # Create command JSON
        command = {
            'type': 'motor',
            'left': left_speed,
            'right': right_speed
        }
        
        # Send to Arduino
        self.send_to_arduino(command)
    
    def send_to_arduino(self, data):
        if not self.serial:
            return
        
        try:
            # Convert dict to JSON string and add newline
            json_str = json.dumps(data) + '\n'
            # Send data
            self.serial.write(json_str.encode())
        except Exception as e:
            rospy.logerr(f"Error sending data to Arduino: {e}")
    
    def read_from_arduino(self, event):
        if not self.serial:
            return
        
        try:
            if self.serial.in_waiting > 0:
                # Read line from Arduino
                line = self.serial.readline().decode('utf-8').strip()
                
                # Try to parse as JSON
                try:
                    data = json.loads(line)
                    
                    # Process different message types
                    if 'type' in data:
                        if data['type'] == 'sensors':
                            # Process sensor data
                            sensor_msg = Float32MultiArray()
                            sensor_msg.data = data.get('values', [])
                            self.sensor_pub.publish(sensor_msg)
                        
                        elif data['type'] == 'status':
                            # Process status message
                            status_msg = String()
                            status_msg.data = data.get('message', '')
                            self.status_pub.publish(status_msg)
                
                except json.JSONDecodeError:
                    # Not valid JSON, just log the raw message
                    rospy.loginfo(f"Arduino message: {line}")
        
        except Exception as e:
            rospy.logerr(f"Error reading from Arduino: {e}")

if __name__ == '__main__':
    try:
        node = ArduinoBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass