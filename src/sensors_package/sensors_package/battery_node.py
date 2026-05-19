import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus

I2C_ADDR = 0x08
MUX_ADDR = 0x70
MUX_CHANNEL = 0x02


class VoltageNode(Node):
    def __init__(self):
        super().__init__('voltage_node')

        self.publisher_ = self.create_publisher(Float32, 'voltage', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.bus = SMBus(1)
        self.bus.write_byte(MUX_ADDR, MUX_CHANNEL)

        self.get_logger().info('Voltage node started')

    def timer_callback(self):
        try:
            data = self.bus.read_i2c_block_data(I2C_ADDR, 0, 2)
            mv = (data[0] << 8) | data[1]
            voltage = mv / 1000.0

            msg = Float32()
            msg.data = voltage
            self.publisher_.publish(msg)

            self.get_logger().info(f'Voltage: {voltage:.3f} V')
        except Exception as e:
            self.get_logger().error(f'I2C read failed: {e}')

    def destroy_node(self):
        self.bus.close()
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