import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus
import lgpio
import subprocess
import sys

I2C_ADDR    = 0x08
MUX_ADDR    = 0x70
MUX_CHANNEL = 0x02

BATTERY_LOW_PIN = 13


class VoltageNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        # self.publisher_ = self.create_publisher(Float32, 'voltage', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        # self.bus = SMBus(1)
        # self.bus.write_byte(MUX_ADDR, MUX_CHANNEL)

        # --- lgpio setup ---
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_alert(self.chip, BATTERY_LOW_PIN, lgpio.FALLING_EDGE, lgpio.SET_PULL_UP)
        lgpio.callback(self.chip, BATTERY_LOW_PIN, lgpio.FALLING_EDGE, self._battery_low_cb)

        self.get_logger().info('Voltage node started')

    def _battery_low_cb(self, chip, gpio, level, tick):
        if level == 0:
            self.get_logger().fatal(
                f'Battery low! GPIO{BATTERY_LOW_PIN} went LOW — shutting down.'
            )
            self._shutdown_system()

    def _shutdown_system(self):
        self.get_logger().info('Stopping ROS2 nodes...')
        try:
            subprocess.run(['pkill', '-f', 'ros2'], check=False)
        except Exception as e:
            self.get_logger().error(f'Failed to kill ROS2 nodes: {e}')

        self.get_logger().info('Initiating system shutdown...')
        subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=False)
        sys.exit(0)

    # def timer_callback(self):
    #     try:
    #         data = self.bus.read_i2c_block_data(I2C_ADDR, 0, 2)
    #         mv = (data[0] << 8) | data[1]
    #         voltage = mv / 1000.0
    #         msg = Float32()
    #         msg.data = voltage
    #         self.publisher_.publish(msg)
    #         self.get_logger().info(f'Voltage: {voltage:.3f} V')
    #     except Exception as e:
    #         self.get_logger().error(f'I2C read failed: {e}')

    def destroy_node(self):
        try:
            lgpio.gpiochip_close(self.chip)
        except Exception:
            pass
        # self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoltageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()