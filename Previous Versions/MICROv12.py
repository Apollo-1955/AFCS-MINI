from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

# Kalman Filter class
class KalmanFilter:
    def __init__(self, Q=0.1, R=0.5):
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

# MPU6050 Sensor
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.z_bias = 0.0

        try:
            self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')  # Wake up MPU6050
            sleep_ms(100)
        except OSError as e:
            raise Exception("MPU6050 not found") from e

        self.calibrate_z()

    def read_accel_z(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        az_raw = self._twos_complement(data[4] << 8 | data[5])
        az = az_raw / 16384.0 * GRAVITY
        return az - self.z_bias  # Return calibrated Z-acceleration

    def calibrate_z(self, samples=100):
        print("Calibrating Z-axis (keep MPU6050 still)...")
        total = 0
        for _ in range(samples):
            data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
            az_raw = self._twos_complement(data[4] << 8 | data[5])
            total += az_raw / 16384.0 * GRAVITY
            sleep_ms(10)
        self.z_bias = total / samples
        print("Calibration complete. Z-bias = {:.3f} m/s²".format(self.z_bias))

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Altitude Estimator Class
class AltitudeEstimator:
    def __init__(self, mpu):
        self.mpu = mpu
        self.velocity = 0
        self.altitude = 0
        self.last_time = ticks_ms()
        self.kalman = KalmanFilter()

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        self.last_time = now

        az = self.mpu.read_accel_z()
        self.velocity += az * dt
        self.altitude += self.velocity * dt

        return self.kalman.update(self.altitude)

# Main Program
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)  # Adjust to your board
mpu = MPU6050(i2c)
alt_estimator = AltitudeEstimator(mpu)

while True:
    try:
        alt = alt_estimator.update()
        print("Estimated Altitude (m): {:.2f}".format(alt))
    except Exception as e:
        print("Error:", e)
    sleep_ms(100)
