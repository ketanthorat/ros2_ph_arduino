#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('keyboard_publisher')
    publisher = node.create_publisher(String, 'servo_control', 10)
    
    # Initialize curses
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    
    try:
        # Display instructions
        stdscr.clear()
        stdscr.addstr(0, 0, "Arduino Servo Control Interface")
        stdscr.addstr(2, 0, "Press keys 1-9 to control servos:")
        stdscr.addstr(4, 0, "1: OPEN    - All fingers open")
        stdscr.addstr(5, 0, "2: CLOSE   - All fingers closed")
        stdscr.addstr(6, 0, "3: VICTORY - V sign")
        stdscr.addstr(7, 0, "4: ONE     - Index finger extended")
        stdscr.addstr(8, 0, "5: TWO     - Two fingers extended")
        stdscr.addstr(9, 0, "6: THREE   - Three fingers extended")
        stdscr.addstr(10, 0, "7: FOUR    - Four fingers extended")
        stdscr.addstr(11, 0, "8: FIVE    - All fingers extended")
        stdscr.addstr(12, 0, "9: YO      - Thumb and pinky extended")
        stdscr.addstr(14, 0, "Press 'q' to quit")
        stdscr.refresh()
        
        # Process key inputs
        while True:
            key = stdscr.getch()
            
            if key == ord('q'):
                break
            elif key >= ord('1') and key <= ord('9'):
                # Create and publish message
                msg = String()
                msg.data = chr(key)
                publisher.publish(msg)
                
                # Show status
                cmd_char = chr(key)
                gesture_map = {
                    '1': 'OPEN', '2': 'CLOSE', '3': 'VICTORY',
                    '4': 'ONE', '5': 'TWO', '6': 'THREE',
                    '7': 'FOUR', '8': 'FIVE', '9': 'YO'
                }
                gesture = gesture_map.get(cmd_char, 'UNKNOWN')
                stdscr.addstr(16, 0, f"Command sent: {cmd_char} ({gesture})     ")
                stdscr.refresh()
            
    finally:
        # Clean up curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        
        # Clean up ROS
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 