#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__("servo_node_ros")
        
        # Initialize serial communication with Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise
        
        # Create subscriber for servo control commands
        self.subscription = self.create_subscription(
            String,
            'servo_control',
            self.command_callback,
            10
        )
        
        self.get_logger().info("Serial node initialized")
    
    def command_callback(self, msg):
        self.get_logger().info("Sending command: %s" % msg.data)
        myCmd = msg.data + '\r'
        self.arduino.write(myCmd.encode())
    
    def destroy_node(self):
        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info("Closed serial connection")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    
    try:
        # This will allow manual input while ROS node is running
        import threading
        
        def input_thread():
            while True:
                try:
                    myCmd = input('please input your command: ')
                    myCmd = myCmd + '\r'
                    serial_node.arduino.write(myCmd.encode())
                except KeyboardInterrupt:
                    break
        
        # Start input thread
        thread = threading.Thread(target=input_thread)
        thread.daemon = True
        thread.start()
        
        # Spin ROS node
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 