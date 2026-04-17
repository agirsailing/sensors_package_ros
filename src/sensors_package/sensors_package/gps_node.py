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
    bus = SMBus(1)
    bus.write_byte_data(MAG_I2C_ADDR, REG_CNTL2, 0x01)
    time.sleep(0.1)
    bus.write_byte_data(MAG_I2C_ADDR, REG_CNTL1, 0x01)
    time.sleep(0.02)
    return bus

def read_mag(bus: SMBus) -> tuple[int, int, int]:
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
    heading = math.degrees(math.atan2(y, x))
    return heading + 360 if heading < 0 else heading

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.publisher_ = self.create_publisher(String, 'gps', 10)
        self.stream = Serial('/dev/ttyUSB1', 230400, timeout=0.1)
        self.ubr    = UBXReader(self.stream)
        self.mag_bus = setup_compass()

        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp         = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"gps_{self.get_name()}_{timestamp}.csv")

        # Required: timestamp_utc, device_id, lat, lon, speed_kn, heading_deg
        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp_utc", "device_id",
                "lat", "lon",
                "speed_kn", "heading_deg",
                "alt_m", "hdop", "sats_used", "fix_quality",
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

        lat       = parsed_data.lat
        lon       = parsed_data.lon
        speed_ms  = parsed_data.gSpeed / 1000.0        # mm/s → m/s
        speed_kn  = speed_ms * 1.94384                 # m/s → knots
        hdg_deg   = parsed_data.headMot / 100000.0     # 1e-5 deg → deg
        alt_m     = parsed_data.hMSL / 1000.0          # mm → m
        hdop      = getattr(parsed_data, 'pDOP', float('nan')) / 100.0

        try:
            mx, my, _ = read_mag(self.mag_bus)
            hdg_mag   = get_heading(mx, my)
        except Exception as e:
            self.get_logger().warn(f"Compass read error: {e}")
            hdg_mag = float('nan')

        if speed_ms > 0.5:
            hdg_deg = parsed_data.headMot/100000.0     # GPS heading
        else:
            hdg_deg = hdg_mag                          # compass heading

        try:
            utc = datetime(
                parsed_data.year,  parsed_data.month,  parsed_data.day,
                parsed_data.hour,  parsed_data.min,    parsed_data.second,
                tzinfo=timezone.utc,
            ).isoformat()
        except ValueError:
            utc = datetime.now(timezone.utc).isoformat()

        msg      = String()
        msg.data = (
            f"lat={lat:.7f}, lon={lon:.7f}, "
            f"speed={speed_kn:.2f}kn, "
            f"heading={hdg_deg:.1f}deg"
        )
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, self.get_name(),
                lat, lon,
                round(speed_kn, 4), round(hdg_deg, 2),
                round(alt_m, 2), round(hdop, 2), num_sv, fix_type,
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
