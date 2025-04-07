import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ketan/abc/ros2_ph_arduino/install/arduino_servo_control'
