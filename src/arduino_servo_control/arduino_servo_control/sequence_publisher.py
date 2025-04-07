#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SequencePublisher(Node):
    def __init__(self):
        super().__init__('sequence_publisher')
        self.publisher = self.create_publisher(String, 'servo_control', 10)
        self.get_logger().info('Sequence publisher initialized')
        
    def publish_sequence(self, delay=2.0):
        """Publish commands 1-9 with specified delay between each."""
        self.get_logger().info(f'Starting to publish sequence with {delay} second interval')
        
        for command in range(1, 10):
            # Create and publish message
            msg = String()
            msg.data = str(command)
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {command}')
            
            # Wait for the specified delay
            time.sleep(delay)
            
        self.get_logger().info('Sequence completed')

def main(args=None):
    rclpy.init(args=args)
    publisher = SequencePublisher()
    
    try:
        # Run the sequence once
        publisher.publish_sequence(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 