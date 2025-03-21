import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus

class UbloxGPSNode(Node):
    def __init__(self):
        super().__init__('ublox_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Connected to {port} at {baudrate} baud')
            self.get_logger().warn('starting up the timer')
            self.timer = self.create_timer(0.1, self.read_gps_data)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            self.serial_port = None
        
        
    
    def read_gps_data(self):
        if self.serial_port is None:
            self.get_logger().info('none serial port')
            return
        
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            #self.get_logger().warn(f'testing')
            self.get_logger().warn(line)
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)
                navsat_msg = NavSatFix()
                navsat_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_msg.header.frame_id = 'gps'
                navsat_msg.status.status = NavSatStatus.STATUS_FIX
                navsat_msg.status.service = NavSatStatus.SERVICE_GPS
                navsat_msg.latitude = msg.latitude
                navsat_msg.longitude = msg.longitude
                navsat_msg.altitude = msg.altitude if msg.altitude is not None else 0.0
                self.publisher_.publish(navsat_msg)
                self.get_logger().info(f'Published GPS fix: {msg.latitude}, {msg.longitude}, {msg.altitude}')
        except pynmea2.ParseError as e:
            self.get_logger().warn(f'Failed to parse NMEA sentence: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UbloxGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
