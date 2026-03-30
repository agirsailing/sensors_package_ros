import rclpy
import sys
import os
import csv
from datetime import datetime, timezone
from rclpy.node import Node
from std_msgs.msg import Float32

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from .ultrasonic_lib import DFRobot_A02_Distance as Board

class CtrlNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.board = Board()
        self.board.set_dis_range(0, 4500)

        self.publisher_ = self.create_publisher(Float32, 'ctrl', 10)

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp         = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"ctrl_{self.get_name()}_{timestamp}.csv")

        # Schema: telemetry_ctrl
        # Required: timestamp_utc, device_id, ride_height_m, flap_angle_deg, rudder_deg
        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp_utc", "device_id",
                "ride_height_m", "flap_angle_deg", "rudder_deg",
            ])

        self.get_logger().info(f"Logging ctrl data to {self.csv_filepath}")

        self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        distance_mm = self.board.getDistance()

        if self.board.last_operate_status != self.board.STA_OK:
            if self.board.last_operate_status == self.board.STA_ERR_CHECKSUM:
                self.get_logger().warn("Checksum Error!")
            elif self.board.last_operate_status == self.board.STA_ERR_SERIAL:
                self.get_logger().error("Serial open failed!")
            elif self.board.last_operate_status == self.board.STA_ERR_DATA:
                self.get_logger().warn("No data received.")
            return

        ride_height_m  = round(distance_mm / 1000.0, 4)
        flap_angle_deg = float('nan')   # placeholder — wire in when available
        rudder_deg     = float('nan')   # placeholder — wire in when available

        msg      = Float32()
        msg.data = ride_height_m
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        utc = datetime.now(timezone.utc).isoformat()

        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, self.get_name(),
                ride_height_m, flap_angle_deg, rudder_deg,
            ])

    def destroy_node(self):
        super().destroy_node()

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