import rclpy
import sys
import os
import csv
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Float32

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from .ultrasonic_lib import DFRobot_A02_Distance as Board

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        self.board = Board()
        self.board.set_dis_range(0, 4500)
        
        self.publisher_ = self.create_publisher(Float32, 'ultrasonic', 10)
        
        self.timer_period = 0.3 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        log_directory = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_directory, exist_ok=True)

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        node_name = self.get_name()

        self.csv_filename = f"{node_name}_{timestamp}.csv"
        self.csv_filepath = os.path.join(log_directory, self.csv_filename)

        with open(self.csv_filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "distance_mm"])

        self.get_logger().info(f'Ultrasonic Node started. Logging to {self.csv_filepath}')
        self.get_logger().info('Ultrasonic Node has been started.')

    def timer_callback(self):
        distance_mm = self.board.getDistance()
        
        if self.board.last_operate_status == self.board.STA_OK:
            
            msg = Float32()
            msg.data = float(distance_mm)
            self.publisher_.publish(msg)
        
            with open(self.csv_filepath, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    distance_mm
                ])

        elif self.board.last_operate_status == self.board.STA_ERR_CHECKSUM:
            self.get_logger().warn("Checksum Error!")
        elif self.board.last_operate_status == self.board.STA_ERR_SERIAL:
            self.get_logger().error("Serial open failed!")
        elif self.board.last_operate_status == self.board.STA_ERR_DATA:
            self.get_logger().warn("No data received.")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
