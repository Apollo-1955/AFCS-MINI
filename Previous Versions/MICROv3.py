from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import atan2, sqrt, pi, cos

# === I2C SETUP (your pins) ===
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)

# === HARDCODED CALIBRATION VALUES ===
# Replace with your own from calibration script
ACCEL_BIAS = {'x': 0.01, 'y': -0.02, 'z': 1.00}
GYRO_BIAS  = {'x': -0.5, 'y': 0.2, 'z': 0.1}

# === SIMPLE KALMAN FILTER CLASS ===
class SimpleKalman:
    def __init__(self, q=0.01, r=0.5, p=1, x=0):
        self.q = q  # process noise
        self.r = r  # measurement noise
        self.p = p  # estimation error
        self.x = x  # estimated value

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# === VARIABLES ===
velocity = 0
altitude = 0
prev_time = ticks_ms()
prev_accel = 0
kalman = SimpleKalman()

print("Starting altitude estimation...\n")

# === MAIN LOOP ===
while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000  # seconds
    prev_time = now

    # === READ RAW ACCEL DATA AND APPLY CALIBRATION ===
    accel = mpu.accel
    ax = (accel.x - ACCEL_BIAS['x']) * 9.81
    ay = (accel.y - ACCEL_BIAS['y']) * 9.81
    az = (accel.z - ACCEL_BIAS['z']) * 9.81

    # === OPTIONAL: PITCH CORRECTION ===
    # pitch = atan2(ax, sqrt(ay**2 + az**2))
    # acc_vertical = az * cos(pitch)
    # true_accel = acc_vertical + 9.81

    # === SIMPLIFIED VERTICAL FLIGHT CASE ===
    true_accel = az + 9.81

    # === KALMAN FILTER ===
    filtered_accel = kalman.update(true_accel)

    # === DOUBLE INTEGRATION ===
    velocity += 0.5 * (prev_accel + filtered_accel) * dt
    altitude += velocity * dt
    prev_accel = filtered_accel

    # === DEBUG OUTPUT ===
    print("az_raw: {:.2f}, accel: {:.2f}, vel: {:.2f}, alt: {:.2f}".format(
        az, true_accel, velocity, altitude))

    sleep_ms(50)
