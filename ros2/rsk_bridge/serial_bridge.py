#!/usr/bin/env python3
# serial_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import threading
import time

SERIAL_PORT = '/dev/ttyUSB0'   # change to your device
BAUDRATE = 115200

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        self.pub = self.create_publisher(String, '/fpga/state', 10)
        self.sub = self.create_subscription(String, '/fpga/command', self.on_command, 10)
        self.get_logger().info(f"Opened serial on {SERIAL_PORT} @ {BAUDRATE}")
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def on_command(self, msg):
        # Expect msg.data is a JSON string according to protocol
        payload = msg.data
        try:
            # append newline as frame delimiter
            self.ser.write((payload + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def _read_loop(self):
        while rclpy.ok() and not self._stop:
            try:
                line = self.ser.readline()
                if line:
                    try:
                        # Strip and forward raw text as String
                        s = line.decode('utf-8').strip()
                        msg = String()
                        msg.data = s
                        self.pub.publish(msg)
                    except Exception as e:
                        self.get_logger().warn(f"Error decoding line: {e}")
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1.0)

    def destroy_node(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
