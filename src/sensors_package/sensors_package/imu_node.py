import rclpy
import math
import smbus2
import os
import csv
from datetime import datetime, timezone

from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# PCA9546 mux
MUX_ADDR    = 0x70
MUX_CHANNEL = 2          # SC2/SD2 → write 1<<2 = 0x04

# LSM6DS3TR-C register addresses
IMU_ADDR  = 0x6A
CTRL1_XL  = 0x10   # accelerometer config register
CTRL2_G   = 0x11   # gyroscope config register
OUTX_L_G  = 0x22   # first of 6 gyro output bytes  (Xlo, Xhi, Ylo, Yhi, Zlo, Zhi)
OUTX_L_XL = 0x28   # first of 6 accel output bytes (same layout)

# Physical unit conversions 
# At ODR=104 Hz, ±4g:     1 LSB = 0.122 mg  → ×9.80665 → m/s²
# At ODR=104 Hz, ±500 dps: 1 LSB = 17.5 mdps → ×π/180  → rad/s
ACCEL_SENS = 0.122e-3 * 9.80665   # m/s² per raw LSB
GYRO_SENS  = 17.50e-3 * (math.pi / 180.0)  # rad/s per raw LSB

# Calibration settings
CALIB_SAMPLES = 200   # ~2 s at 100 Hz 


def _signed16(high, low):
    """Convert two raw bytes (high, low) into a signed 16-bit integer."""
    val = (high << 8) | low
    return val - 65536 if val >= 32768 else val


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ROS parameters
        self.declare_parameter('i2c_bus',   1)
        self.declare_parameter('topic',     'imu/data_raw')
        self.declare_parameter('device_id', 'imu')
        self.declare_parameter('rate_hz',   50.0)

        bus_num        = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        topic          = self.get_parameter('topic').get_parameter_value().string_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().string_value
        rate_hz        = self.get_parameter('rate_hz').get_parameter_value().double_value

        # I2C setup
        self.bus = smbus2.SMBus(bus_num)
        self._mux_open()
        self._init_imu()
        self._mux_close()

        # Calibration
        # Collect CALIB_SAMPLES readings while perfectly still.
        # The average of each axis becomes its "zero offset".
        # After subtracting these offsets, a still sensor reads (0,0,0) gyro
        # and (0,0,9.81) accel — making roll=pitch=yaw=0 at the boot pose.
        self.get_logger().info(f"{self.device_id}: calibrating — keep sensor still...")
        (self.ax_off, self.ay_off, self.az_off,
         self.gx_off, self.gy_off, self.gz_off) = self._calibrate()
        self.get_logger().info(
            f"{self.device_id}: calibration done  "
            f"gyro offsets rad/s → gx={self.gx_off:.4f} gy={self.gy_off:.4f} gz={self.gz_off:.4f}"
        )

        # Complementary filter state
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self.dt    = 1.0 / rate_hz
        self.alpha = 0.98   # 98% gyro, 2% accel

        # CSV logging
        log_dir = os.path.expanduser("~/ros2_ws/CSVs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filepath = os.path.join(log_dir, f"imu_{self.device_id}_{timestamp}.csv")

        with open(self.csv_filepath, mode='w', newline='') as f:
            csv.writer(f).writerow([
                "timestamp_utc", "device_id",
                "roll_deg", "pitch_deg", "yaw_deg",
                "ax_mps2", "ay_mps2", "az_mps2",
                "gx_rads", "gy_rads", "gz_rads",
            ])

        # ROS publisher
        self.publisher_ = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f"{self.device_id} publishing '{topic}' @ {rate_hz} Hz  |  "
            f"CSV → {self.csv_filepath}"
        )

    # Mux helpers 

    def _mux_open(self):
        """Enable channel MUX_CHANNEL on the PCA9546 so the IMU is reachable."""
        self.bus.write_byte(MUX_ADDR, 1 << MUX_CHANNEL)

    def _mux_close(self):
        """Disable all PCA9546 channels so other sensors are not blocked."""
        self.bus.write_byte(MUX_ADDR, 0x00)

    # IMU hardware init

    def _init_imu(self):
        """
        Write config registers to start the sensor.
        CTRL1_XL = 0x48 → ODR 104 Hz, ±4g full scale
        CTRL2_G  = 0x54 → ODR 104 Hz, ±500 dps full scale
        The ODR and range must match the ACCEL_SENS / GYRO_SENS constants above.
        """
        self.bus.write_byte_data(IMU_ADDR, CTRL1_XL, 0x48)
        self.bus.write_byte_data(IMU_ADDR, CTRL2_G,  0x54)

    # Raw read

    def _read_raw(self):
        """
        Read 6 bytes each for gyro and accel, convert to physical units.
        Each axis is a little-endian signed 16-bit int: byte[1]<<8 | byte[0].
        Returns ax,ay,az (m/s²) and gx,gy,gz (rad/s) — NOT offset-corrected yet.
        """
        raw_g  = self.bus.read_i2c_block_data(IMU_ADDR, OUTX_L_G,  6)
        raw_xl = self.bus.read_i2c_block_data(IMU_ADDR, OUTX_L_XL, 6)

        gx = _signed16(raw_g[1],  raw_g[0])  * GYRO_SENS
        gy = _signed16(raw_g[3],  raw_g[2])  * GYRO_SENS
        gz = _signed16(raw_g[5],  raw_g[4])  * GYRO_SENS

        ax = _signed16(raw_xl[1], raw_xl[0]) * ACCEL_SENS
        ay = _signed16(raw_xl[3], raw_xl[2]) * ACCEL_SENS
        az = _signed16(raw_xl[5], raw_xl[4]) * ACCEL_SENS

        return ax, ay, az, gx, gy, gz

    # Calibration 

    def _calibrate(self):
        """
        Sample the sensor CALIB_SAMPLES times while perfectly still.
        The mean of each axis is the bias offset to subtract from future reads.

        Gyro:  mean should be 0 — any non-zero value is pure sensor error (bias).
        Accel: mean should be (0, 0, 9.81) — we subtract the XY offset and
               remove only the non-gravity part of Z.
        """
        self._mux_open()
        sums = [0.0] * 6
        for _ in range(CALIB_SAMPLES):
            vals = self._read_raw()
            for i in range(6):
                sums[i] += vals[i]
        self._mux_close()

        ax_off = sums[0] / CALIB_SAMPLES
        ay_off = sums[1] / CALIB_SAMPLES
        az_off = sums[2] / CALIB_SAMPLES - 9.80665  # leave gravity, remove offset
        gx_off = sums[3] / CALIB_SAMPLES
        gy_off = sums[4] / CALIB_SAMPLES
        gz_off = sums[5] / CALIB_SAMPLES

        return ax_off, ay_off, az_off, gx_off, gy_off, gz_off

    def _read_imu(self):
        """Read sensor and subtract calibration offsets."""
        ax, ay, az, gx, gy, gz = self._read_raw()
        return (
            ax - self.ax_off,
            ay - self.ay_off,
            az - self.az_off,
            gx - self.gx_off,
            gy - self.gy_off,
            gz - self.gz_off,
        )

    # Complementary filter

    def _update_angles(self, ax, ay, az, gx, gy, gz):
        """
        Fuse accelerometer + gyroscope into roll, pitch, yaw.

        WHY TWO SENSORS?
          Gyroscope:     integrates angular rate → gives angle change per tick.
                         Fast and smooth, but error accumulates (drift) over time.
          Accelerometer: measures direction of gravity → absolute tilt angle.
                         No long-term drift, but noisy and disturbed by vibration.

        COMPLEMENTARY FILTER IDEA:
          new_angle = alpha  × (gyro prediction)     ← fast, short-term accurate
                    + (1-alpha) × (accel correction)  ← slow, long-term stable

          With alpha=0.98, each tick:
            1. Predict new angle by integrating gyro:  angle + gx × dt
            2. Gently correct toward accel estimate:   blend in 2% of accel

        accel_roll  = atan2(ay, az)
          Geometry: gravity projects onto Y and Z axes based on roll.
          Flat → ay=0, az=9.81 → atan2(0,9.81) = 0°
          Rolled 45° right → ay grows → atan2 returns +45°

        accel_pitch = atan2(-ax, √(ay²+az²))
          The denominator is the horizontal gravity magnitude — stays stable
          even when rolled, so pitch estimate doesn't couple into roll.

        YAW cannot be measured from gravity (gravity is vertical regardless of
        heading). So yaw is gyro-only and drifts slowly. Your GPS compass
        (magnetometer) is the natural fix — it gives absolute heading.
        """
        accel_roll  = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        self.roll  = self.alpha * (self.roll  + gx * self.dt) + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * self.dt) + (1.0 - self.alpha) * accel_pitch
        self.yaw  += gz * self.dt   # gyro-only until compass fusion is added

    # Main timer callback

    def timer_callback(self):
        try:
            self._mux_open()
            ax, ay, az, gx, gy, gz = self._read_imu()
            self._mux_close()
        except Exception as e:
            self.get_logger().error(f"{self.device_id}: read failed — {e}")
            return

        self._update_angles(ax, ay, az, gx, gy, gz)

        roll_deg  = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg   = math.degrees(self.yaw)

        # ROS message ──────
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.device_id

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation = self._rpy_to_quaternion(self.roll, self.pitch, self.yaw)

        # Covariance: 9-element flat array representing a 3×3 uncertainty matrix.
        # Setting [0] = -1.0 is the ROS convention meaning "covariance unknown".
        # Downstream nodes (robot_localization, rviz) will ignore the matrix.
        # Fill in real noise values once you characterise the sensor.
        msg.orientation_covariance[0]         = -1.0
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"{self.device_id}: "
            f"roll={roll_deg:+7.2f}°  pitch={pitch_deg:+7.2f}°  yaw={yaw_deg:+7.2f}°"
        )

        # CSV log 
        utc = datetime.now(timezone.utc).isoformat()
        with open(self.csv_filepath, mode='a', newline='') as f:
            csv.writer(f).writerow([
                utc, self.device_id,
                round(roll_deg,  4),
                round(pitch_deg, 4),
                round(yaw_deg,   4),
                round(ax, 6), round(ay, 6), round(az, 6),
                round(gx, 6), round(gy, 6), round(gz, 6),
            ])

    # Quaternion conversion 

    @staticmethod
    def _rpy_to_quaternion(roll, pitch, yaw):
        """
        Convert roll/pitch/yaw (radians) to a unit quaternion.
        ROS uses quaternions instead of Euler angles because they avoid
        gimbal lock and interpolate smoothly. Standard ZYX intrinsic convention.
        """
        cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()