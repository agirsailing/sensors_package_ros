import rclpy
import sys
import os
import csv
from datetime import datetime, timezone

from rclpy.node import Node
from sensor_msgs.msg import Range

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from .ultrasonic_lib import DFRobot_A02_Distance as Board

class CtrlNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('topic', 'ctrl')
        self.declare_parameter('device_id', 'ultrasonic')

        port = self.get_parameter('port').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().string_value

        self.board = Board(port=port)
        self.board.set_dis_range(0, 4500)

        self.publisher_ = self.create_publisher(Range, topic, 10)

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"ctrl_{self.device_id}_{timestamp}.csv")

        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp_utc", "device_id",
                "ride_height_m", "flap_angle_deg", "rudder_deg",
            ])

        self.get_logger().info(f"{self.device_id} using port {port}")
        self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        distance_mm = self.board.getDistance()

        if self.board.last_operate_status != self.board.STA_OK:
            if self.board.last_operate_status == self.board.STA_ERR_CHECKSUM:
                self.get_logger().warn(f"{self.device_id}: checksum error")
            elif self.board.last_operate_status == self.board.STA_ERR_SERIAL:
                self.get_logger().error(f"{self.device_id}: serial open failed")
            elif self.board.last_operate_status == self.board.STA_ERR_DATA:
                self.get_logger().warn(f"{self.device_id}: no data received")
            return

        ride_height_m = round(distance_mm/1000.0, 4)

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.0
        msg.max_range = 4.5
        msg.range = ride_height_m

        self.publisher_.publish(msg)

        self.get_logger().info(f"{self.device_id}: publishing {msg.range} m")

        utc = datetime.now(timezone.utc).isoformat()
        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, self.device_id,
                ride_height_m, float('nan'), float('nan'),
            ])

def main(args=None):
    rclpy.init(args=args)
    node = CtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()