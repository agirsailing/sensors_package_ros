import rclpy
import os
import csv
from datetime import datetime, timezone

from rclpy.node import Node
from sensor_msgs.msg import Range


class TreatmentNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_filter_node')

        self.left_value = None
        self.right_value = None
        self.left_stamp = None
        self.right_stamp = None
        self.max_dt = 0.05  # seconds

        self.sub_left = self.create_subscription(
            Range,
            'ctrl_raw_left',
            self.left_callback,
            10
        )

        self.sub_right = self.create_subscription(
            Range,
            'ctrl_raw_right',
            self.right_callback,
            10
        )

        self.publisher_ = self.create_publisher(Range, 'ctrl_mean', 10)

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"ctrl_mean_{timestamp}.csv")

        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp_utc", "device_id",
                "ride_height_m", "flap_angle_deg", "rudder_deg",
            ])

        self.get_logger().info(f"Logging mean data to {self.csv_filepath}")

    def left_callback(self, msg):
        self.left_value = msg.range
        self.left_stamp = msg.header.stamp
        self.compute_and_save()

    def right_callback(self, msg):
        self.right_value = msg.range
        self.right_stamp = msg.header.stamp
        self.compute_and_save()

    def compute_and_save(self):
        if self.left_value is None or self.right_value is None:
            return

        left_t = self.left_stamp.sec + self.left_stamp.nanosec * 1e-9
        right_t = self.right_stamp.sec + self.right_stamp.nanosec * 1e-9

        if abs(left_t - right_t) > self.max_dt:
            return

        mean_value = round((self.left_value + self.right_value) / 2.0, 4)

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_mean"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.0
        msg.max_range = 4.5
        msg.range = mean_value
        self.publisher_.publish(msg)

        self.get_logger().info(f"Mean ride height: {mean_value} m")

        utc = datetime.now(timezone.utc).isoformat()
        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, "ultrasonic_mean",
                mean_value, float('nan'), float('nan'),
            ])

        self.left_value = None
        self.right_value = None
        self.left_stamp = None
        self.right_stamp = None


def main(args=None):
    rclpy.init(args=args)
    node = TreatmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()