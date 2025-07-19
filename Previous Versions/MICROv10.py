from machine import I2C, Pin
from time import sleep_ms
import math

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Kalman Filter class
class KalmanFilter:
    def __init__(self, Q=0.01, R=0.1):
        self.Q = Q  # process noise
        self.R = R  # measurement noise
        self.P = 1.0
        self.X = 0.0

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.X += K * (measurement - self.X)
        self.P *= (1 - K)
        return self.X

# MPU6050 class
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        try:
            self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')  # Wake up
            sleep_ms(100)
        except OSError as e:
            raise Exception("MPU6050 not found or not responding.") from e

    def read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 14)
        acc = [self._twos_complement(data[i] << 8 | data[i + 1]) / 16384.0 for i in range(0, 6, 2)]
        gyro = [self._twos_complement(data[i] << 8 | data[i + 1]) / 131.0 for i in range(8, 14, 2)]
        return acc, gyro

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Orientation Estimator
class OrientationEstimator:
    def __init__(self, mpu):
        self.mpu = mpu
        self.kalman_pitch = KalmanFilter()
        self.kalman_roll = KalmanFilter()

    def get_angles(self):
        acc, _ = self.mpu.read_raw()
        ax, ay, az = acc

        pitch = math.atan2(ax, math.sqrt(ay * ay + az * az)) * (180 / math.pi)
        roll = math.atan2(ay, math.sqrt(ax * ax + az * az)) * (180 / math.pi)

        pitch_f = self.kalman_pitch.update(pitch)
        roll_f = self.kalman_roll.update(roll)

        return pitch_f, roll_f

# Main
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)  # Adjust pins as needed
mpu = MPU6050(i2c)
estimator = OrientationEstimator(mpu)

while True:
    try:
        pitch, roll = estimator.get_angles()
        print("Pitch: {:.2f}°, Roll: {:.2f}°".format(pitch, roll))
    except Exception as e:
        print("Error:", e)
    sleep_ms(100)
