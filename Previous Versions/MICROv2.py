from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import atan2, sqrt, pi, cos

# === SETUP ===
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
imu = MPU6050(i2c)

# === HARDCODED CALIBRATION VALUES ===
# Use your own values from the calibration script
ACCEL_BIAS = {'x': 0.01, 'y': -0.02, 'z': 1.00}
GYRO_BIAS  = {'x': -0.5, 'y': 0.2, 'z': 0.1}

# === KALMAN FILTER CLASS ===
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

# === LOOP ===
print("Starting altitude estimation...\n")

while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000  # in seconds
    prev_time = now

    # === READ SENSOR ===
    accel = imu.accel
    ax = accel.x - ACCEL_BIAS['x']
    ay = accel.y - ACCEL_BIAS['y']
    az = accel.z - ACCEL_BIAS['z']

    # Convert to m/s²
    ax *= 9.81
    ay *= 9.81
    az *= 9.81

    # === ESTIMATE PITCH FROM ACCEL ===
    pitch = atan2(ax, sqrt(ay**2 + az**2))  # in radians

    # === PROJECT ACCELERATION TO GLOBAL Z ===
    acc_vertical = az * cos(pitch)

    # === REMOVE GRAVITY ===
    true_accel = acc_vertical - 9.81  # now in m/s²

    # === FILTER ACCELERATION ===
    filtered_accel = kalman.update(true_accel)

    # === INTEGRATE TO VELOCITY & ALTITUDE ===
    velocity += 0.5 * (prev_accel + filtered_accel) * dt
    altitude += velocity * dt
    prev_accel = filtered_accel

    # === OUTPUT ===
    print("Altitude: {:.2f} m | Velocity: {:.2f} m/s | Acc: {:.2f} m/s²".format(
        altitude, velocity, filtered_accel))

    sleep_ms(50)
