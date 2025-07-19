from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math

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
    def __init__(self, mpu, still_threshold=0.2, still_timeout=1500):
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

        # Fake force injection during free fall (total acceleration ~0g)
        if self.fake_force_enabled and total_acc < 1.5:
            # Add artificial downward acceleration
            az = -GRAVITY * 0.85  # simulate falling
            print("⚠️ Free fall detected — injecting fake force.")

        # Apply velocity update
        if abs(az) > self.still_threshold:
            self.velocity += az * dt
            self.last_motion_time = now
        else:
            if ticks_diff(now, self.last_motion_time) > self.still_timeout:
                self.velocity = 0.0
            else:
                self.velocity *= 0.99

        # Update altitude
        self.altitude += self.velocity * dt
        return self.kalman.update(self.altitude)

# Init
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
mpu = MPU6050(i2c)
altitude_estimator = AltitudeEstimator(mpu)

# Main loop
while True:
    try:
        altitude = altitude_estimator.update()
        print("Estimated Altitude (m): {:.2f}".format(altitude))
    except Exception as e:
        print("Error:", e)
    sleep_ms(100)
