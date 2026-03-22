import math
import time
import os
import csv
import rclpy
from datetime import datetime, timezone
from rclpy.node import Node
from std_msgs.msg import String
from serial import Serial
from pyubx2 import UBXReader
from smbus2 import SMBus

MAG_I2C_ADDR = 0x0E
REG_DATA_X_L = 0x03
REG_CNTL1    = 0x0A
REG_CNTL2    = 0x0B

def setup_compass() -> SMBus:
    """Reset MAG3110 and prepare for single-measurement mode."""
    bus = SMBus(1)
    # Reset first
    bus.write_byte_data(MAG_I2C_ADDR, REG_CNTL2, 0x01)
    time.sleep(0.1)
    bus.write_byte_data(MAG_I2C_ADDR, REG_CNTL1, 0x01)
    time.sleep(0.02)
    return bus


def read_mag(bus: SMBus) -> tuple[int, int, int]:
    """Trigger one measurement then read it (MAG3110 single-measurement mode)."""
    bus.write_byte_data(MAG_I2C_ADDR, REG_CNTL1, 0x01)  
    time.sleep(0.02)                                      
    data = bus.read_i2c_block_data(MAG_I2C_ADDR, REG_DATA_X_L, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z
    return x, y, z


def get_heading(x: int, y: int) -> float:
    """Return compass heading in degrees [0, 360)."""
    heading = math.degrees(math.atan2(y, x))
    return heading + 360 if heading < 0 else heading

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.publisher_ = self.create_publisher(String, 'gps', 10)

        self.stream = Serial('/dev/ttyUSB0', 230400, timeout=0.1)
        self.ubr    = UBXReader(self.stream)

        self.mag_bus = setup_compass()

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp  = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        node_name  = self.get_name()
        csv_path   = os.path.join(log_dir, f"{node_name}_{timestamp}.csv")
        self.csv_filepath = csv_path

        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp", "latitude", "longitude",
                "speed_ms", "heading_gps", "heading_compass",
                "fix_type", "num_satellites"
            ])

        self.get_logger().info(f"Logging GPS data to {self.csv_filepath}")

        self.timer = self.create_timer(0.1, self.read_gps)

    def read_gps(self):
        try:
            raw_data, parsed_data = self.ubr.read()
        except Exception as e:
            self.get_logger().warn(f"UBX read error: {e}")
            return

        if parsed_data is None or parsed_data.identity != "NAV-PVT":
            return

        fix_type = parsed_data.fixType
        num_sv   = parsed_data.numSV

        if fix_type == 0:
            msg      = String()
            msg.data = "No GPS signal"
            self.publisher_.publish(msg)
            self.get_logger().warn("No GPS signal")
            return

        lat     = parsed_data.lat
        lon     = parsed_data.lon
        speed   = parsed_data.gSpeed / 1000.0        # mm/s → m/s
        hdg_gps = parsed_data.headMot / 100000.0     # 1e-5 deg → deg

        if speed <= 0.3:
            speed   = 0.0
            hdg_gps = 0.0

        try:
            mx, my, _ = read_mag(self.mag_bus)
            hdg_mag   = get_heading(mx, my)
        except Exception as e:
            self.get_logger().warn(f"Compass read error: {e}")
            hdg_mag = float('nan')

        try:
            utc = datetime(
                parsed_data.year, parsed_data.month,  parsed_data.day,
                parsed_data.hour, parsed_data.min,    parsed_data.second,
                tzinfo=timezone.utc,
            ).isoformat()
        except ValueError:
            utc = "Invalid Date/Time"

        msg      = String()
        msg.data = (
            f"lat={lat:.7f}, lon={lon:.7f}, "
            f"speed={speed:.2f}m/s, "
            f"hdg_gps={hdg_gps:.1f}deg, "
            f"hdg_compass={hdg_mag:.1f}deg"
        )
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, lat, lon,
                speed, hdg_gps, hdg_mag,
                fix_type, num_sv,
            ])

    def destroy_node(self):
        if self.stream.is_open:
            self.stream.close()
        self.mag_bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
