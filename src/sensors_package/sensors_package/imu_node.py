import math
import time
import os
import csv
import rclpy
from datetime import datetime, timezone
from rclpy.node import Node
from std_msgs.msg import String
from smbus2 import SMBus

ACCEL_GYRO   = 0x6A
MAG          = 0x1C
ACCEL_SCALE  = 0.000732
MAG_SCALE    = 0.000146
MAG_OFFSET_X = 1082.0
MAG_OFFSET_Y =  229.0
MAG_OFFSET_Z = -5286.0


def s16(low, high):
    r = (high << 8) | low
    return r - 65536 if r >= 32768 else r


def setup_imu(bus: SMBus) -> None:
    bus.write_byte_data(ACCEL_GYRO, 0x22, 0x01)   # accel/gyro SW reset
    time.sleep(0.5)
    bus.write_byte_data(MAG, 0x21, 0x0C)           # mag reboot + reset
    time.sleep(0.1)
    bus.write_byte_data(MAG, 0x21, 0x00)           # clear reset
    time.sleep(0.1)
    bus.write_byte_data(ACCEL_GYRO, 0x20, 0x68)   # accel 119Hz ±16g
    bus.write_byte_data(MAG, 0x20, 0x70)           # XY high perf 40Hz
    bus.write_byte_data(MAG, 0x23, 0x0C)           # Z high perf
    bus.write_byte_data(MAG, 0x21, 0x00)           # ±4 gauss
    bus.write_byte_data(MAG, 0x22, 0x00)           # continuous — must be last
    time.sleep(0.2)


def read_accel(bus: SMBus) -> tuple[float, float, float]:
    d = bus.read_i2c_block_data(ACCEL_GYRO, 0x28 | 0x80, 6)
    return (s16(d[0], d[1]) * ACCEL_SCALE,
            s16(d[2], d[3]) * ACCEL_SCALE,
            s16(d[4], d[5]) * ACCEL_SCALE)


def read_mag(bus: SMBus) -> tuple[float, float, float]:
    xl = bus.read_byte_data(MAG, 0x28)
    xh = bus.read_byte_data(MAG, 0x29)
    yl = bus.read_byte_data(MAG, 0x2A)
    yh = bus.read_byte_data(MAG, 0x2B)
    zl = bus.read_byte_data(MAG, 0x2C)
    zh = bus.read_byte_data(MAG, 0x2D)
    mx = (s16(xl, xh) - MAG_OFFSET_X) * MAG_SCALE
    my = (s16(yl, yh) - MAG_OFFSET_Y) * MAG_SCALE
    mz = (s16(zl, zh) - MAG_OFFSET_Z) * MAG_SCALE
    return mx, my, mz


def compute_orientation(bus: SMBus, yaw_zero: float) -> tuple[float, float, float]:
    ax, ay, az = read_accel(bus)
    mx, my, mz = read_mag(bus)

    roll  = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

    mx_c = mx * math.cos(pitch) + mz * math.sin(pitch)
    my_c = (mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch))
    raw_yaw = math.degrees(math.atan2(-my_c, mx_c))

    yaw = raw_yaw - yaw_zero
    while yaw >  180: yaw -= 360
    while yaw < -180: yaw += 360

    return math.degrees(roll), math.degrees(pitch), yaw


def calibrate_yaw_zero(bus: SMBus, samples: int = 20) -> float:
    readings = []
    for _ in range(samples):
        ax, ay, az = read_accel(bus)
        mx, my, mz = read_mag(bus)
        roll  = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        mx_c  = mx * math.cos(pitch) + mz * math.sin(pitch)
        my_c  = (mx * math.sin(roll) * math.sin(pitch)
                 + my * math.cos(roll)
                 - mz * math.sin(roll) * math.cos(pitch))
        readings.append(math.degrees(math.atan2(-my_c, mx_c)))
        time.sleep(0.05)
    return sum(readings) / len(readings)


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.bus = SMBus(1)
        setup_imu(self.bus)

        self.get_logger().info("Hold board still — calibrating yaw zero reference...")
        self.yaw_zero = calibrate_yaw_zero(self.bus)
        self.get_logger().info(
            f"Yaw zero set to {self.yaw_zero:.1f}° magnetic. Current heading is 0°."
        )

        self.publisher_ = self.create_publisher(String, 'imu', 10)

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp       = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"imu_{self.get_name()}_{timestamp}.csv")

        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
            "timestamp_utc", "device_id",
            "roll_deg", "pitch_deg", "yaw_deg",
            "heading_mag_deg",
        ])

        self.get_logger().info(f"Logging IMU data to {self.csv_filepath}")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            roll, pitch, yaw = compute_orientation(self.bus, self.yaw_zero)
        except OSError as e:
            self.get_logger().warn(f"IMU read error: {e}")
            return

        msg      = String()
        msg.data = f"roll={roll:.1f}deg, pitch={pitch:.1f}deg, yaw={yaw:.1f}deg"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                datetime.now(timezone.utc).isoformat(),
                self.get_name(),
                round(roll,  3),
                round(pitch, 3),
                round(yaw,   3),
                float('nan'),   # heading_mag_deg — add if you want raw mag heading
            ])

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()