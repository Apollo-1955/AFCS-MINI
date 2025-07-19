from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

# Kalman Filter
class KalmanFilter:
    def __init__(self, Q=0.01, R=0.8):
        self.Q = Q
        self.R = R
        self.P = 1.0
        self.X = 0.0

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.X += K * (measurement - self.X)
        self.P *= (1 - K)
        return self.X

# MPU6050 Reader
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        try:
            self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')
            sleep_ms(100)
        except OSError as e:
            raise Exception("MPU6050 not found") from e

        self.z_bias = self.calibrate_z()

    def read_accel_z(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        az_raw = self._twos_complement(data[4] << 8 | data[5])
        az = az_raw / 16384.0 * GRAVITY
        return az - self.z_bias

    def read_total_accel(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        ax = self._twos_complement(data[0] << 8 | data[1]) / 16384.0 * GRAVITY
        ay = self._twos_complement(data[2] << 8 | data[3]) / 16384.0 * GRAVITY
        az = self._twos_complement(data[4] << 8 | data[5]) / 16384.0 * GRAVITY
        total = math.sqrt(ax**2 + ay**2 + az**2)
        return total

    def calibrate_z(self, samples=100):
        print("Calibrating Z-axis... Hold still.")
        total = 0
        for _ in range(samples):
            data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
            az_raw = self._twos_complement(data[4] << 8 | data[5])
            total += az_raw / 16384.0 * GRAVITY
            sleep_ms(10)
        z_bias = total / samples
        print("Z-bias calibrated: {:.3f} m/s²".format(z_bias))
        return z_bias

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Altitude Estimator with Fake Force
class AltitudeEstimator:
    def __init__(self, mpu, still_threshold=0.1, still_timeout=1500):
        self.mpu = mpu
        self.velocity = 0.0
        self.altitude = 0.0
        self.kalman = KalmanFilter()
        self.last_time = ticks_ms()
        self.last_motion_time = ticks_ms()
        self.still_threshold = still_threshold
        self.still_timeout = still_timeout
        self.fake_force_enabled = True

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        self.last_time = now

        az = self.mpu.read_accel_z()
        total_acc = self.mpu.read_total_accel()

        # Inject strong fake force if free fall detected
        if self.fake_force_enabled and total_acc < 2.0:
            az = -1.1 * GRAVITY  # simulate strong falling acceleration
            self.velocity += az * dt * 0.8  # dampen slightly to prevent runaway
        elif abs(az) > self.still_threshold:
            self.velocity += az * dt
            self.last_motion_time = now
        else:
            if ticks_diff(now, self.last_motion_time) > self.still_timeout:
                self.velocity = 0.0
            else:
                self.velocity *= 0.95  # strong decay

        # Cap extreme velocities
        self.velocity = max(min(self.velocity, 50), -50)

        self.altitude += self.velocity * dt

        # Ground detection and damping
        if self.altitude < 0:
            self.altitude = 0
            self.velocity = 0

        return self.kalman.update(self.altitude)

# Utility to get a new file name with incremental number
def get_new_filename(prefix="altitude_", ext=".csv"):
    i = 1
    while True:
        filename = "{}{}{}".format(prefix, i, ext)
        try:
            # Check if file exists by trying to open
            with open(filename, "r"):
                i += 1
        except OSError:
            # File does not exist, so use this filename
            return filename

# Main
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
mpu = MPU6050(i2c)
altitude_estimator = AltitudeEstimator(mpu)

filename = get_new_filename()
print("Logging altitude data to:", filename)

# Create file and write header
with open(filename, "w") as f:
    f.write("time_s,altitude_m\n")

start_time = ticks_ms()
last_save_time = start_time
buffer = []

while True:
    try:
        altitude = altitude_estimator.update()
        elapsed_time = ticks_diff(ticks_ms(), start_time) / 1000.0

        # Print altitude to terminal
        print("Time: {:.2f} s, Altitude: {:.2f} m".format(elapsed_time, altitude))

        # Append data point to buffer
        buffer.append("{:.2f},{:.2f}\n".format(elapsed_time, altitude))

        # Save every 10 seconds
        if ticks_diff(ticks_ms(), last_save_time) > 10000:
            with open(filename, "a") as f:
                for line in buffer:
                    f.write(line)
            buffer = []
            last_save_time = ticks_ms()
            print("Data saved at {:.2f} s".format(elapsed_time))

    except Exception as e:
        print("Error:", e)

    sleep_ms(100)