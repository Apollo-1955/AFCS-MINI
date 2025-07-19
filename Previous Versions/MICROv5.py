from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import atan2, sqrt, cos, sin, radians, pi
from math import atan2, sqrt, cos, radians

# I2C Setup (adjust pins)
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)

# Calibration biases (replace with your calibration values)
ACCEL_BIAS = {'x': 0.01, 'y': -0.02, 'z': 1.00}
GYRO_BIAS = {'x': -0.5, 'y': 0.2, 'z': 0.1}

# Complementary filter constants
alpha = 0.98

# Kalman filter class (same as before)
class SimpleKalman:
    def __init__(self, q=0.01, r=0.5, p=1, x=0):
        self.q = q
        self.r = r
        self.p = p
        self.x = x
    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# Initialize variables
velocity = 0
altitude = 0
prev_time = ticks_ms()
prev_accel = 0
pitch = 0.0  # degrees
kalman = SimpleKalman()

print("Starting altitude estimation with orientation compensation...\n")

while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000
    prev_time = now

    accel = mpu.accel
    gyro = mpu.gyro

    # Apply calibration and convert accel to m/s²
    ax = (accel.x - ACCEL_BIAS['x']) * 9.81
    ay = (accel.y - ACCEL_BIAS['y']) * 9.81
    az = (accel.z - ACCEL_BIAS['z']) * 9.81

    # Apply calibration to gyro (in deg/s)
    gx = gyro.x - GYRO_BIAS['x']

    # Integrate gyro X to update pitch (degrees)
    pitch += gx * dt

    # Accelerometer angle estimate (degrees)
    pitch_acc = atan2(ax, sqrt(ay*ay + az*az)) * (180/pi)

    # Complementary filter to fuse gyro + accel
    pitch = alpha * pitch + (1 - alpha) * pitch_acc

    # Convert pitch to radians for projection
    pitch_rad = radians(pitch)

    # Project accelerometer vector to vertical axis
    acc_vertical = az * cos(pitch_rad) + ax * sin(pitch_rad)

    # Remove gravity (assuming Z points down, so gravity is +9.81 m/s²)
    true_accel = acc_vertical - 9.81

    # Filter acceleration
    filtered_accel = kalman.update(true_accel)

    # Integrate velocity and altitude
    velocity += 0.5 * (prev_accel + filtered_accel) * dt
    altitude += velocity * dt
    prev_accel = filtered_accel

    print("Pitch: {:.2f}°, Acc_vertical: {:.2f} m/s², Vel: {:.2f} m/s, Alt: {:.2f} m".format(
        pitch, true_accel, velocity, altitude))

    sleep_ms(50)
