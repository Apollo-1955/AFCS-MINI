from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import atan2, sqrt, pi, cos

# === I2C SETUP ===
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)

# === HARDCODED CALIBRATION VALUES ===
ACCEL_BIAS = {'x': 0.01, 'y': -0.02, 'z': 1.00}
GYRO_BIAS  = {'x': -0.5, 'y': 0.2, 'z': 0.1}

# === KALMAN FILTER CLASS ===
class SimpleKalman:
    def __init__(self, q=0.01, r=0.5, p=1, x=0):
        self.q = q  # process noise covariance
        self.r = r  # measurement noise covariance
        self.p = p  # estimation error covariance
        self.x = x  # state estimate

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

# === AUTO DETECT GRAVITY SIGN ===
print("Detecting gravity direction, keep MPU6050 still...")
sleep_ms(2000)

# Take several readings and average Z accel
num_samples = 100
accel_z_sum = 0
for _ in range(num_samples):
    accel = mpu.accel
    az = (accel.z - ACCEL_BIAS['z']) * 9.81
    accel_z_sum += az
    sleep_ms(10)
avg_az = accel_z_sum / num_samples
print("Average Z accel at rest:", avg_az)

if avg_az > 0:
    GRAVITY_SIGN = -1  # Z axis points down, subtract gravity
else:
    GRAVITY_SIGN = 1   # Z axis points up, add gravity

print("Gravity sign set to:", GRAVITY_SIGN)
print("Starting altitude estimation...\n")

# === MAIN LOOP ===
while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000  # seconds
    prev_time = now

    # READ RAW ACCEL, APPLY CALIBRATION
    accel = mpu.accel
    ax = (accel.x - ACCEL_BIAS['x']) * 9.81
    ay = (accel.y - ACCEL_BIAS['y']) * 9.81
    az = (accel.z - ACCEL_BIAS['z']) * 9.81

    # OPTIONAL: pitch correction if needed (commented out)
    # pitch = atan2(ax, sqrt(ay**2 + az**2))
    # acc_vertical = az * cos(pitch)
    # true_accel = acc_vertical + GRAVITY_SIGN * 9.81

    # SIMPLE vertical accel without pitch correction
    true_accel = az + GRAVITY_SIGN * 9.81

    # KALMAN FILTER
    filtered_accel = kalman.update(true_accel)

    # DOUBLE INTEGRATION TO GET VELOCITY & ALTITUDE
    velocity += 0.5 * (prev_accel + filtered_accel) * dt
    altitude += velocity * dt
    prev_accel = filtered_accel

    # DEBUG PRINT
    print("az_raw: {:.2f}, true_accel: {:.2f}, velocity: {:.2f}, altitude: {:.2f}".format(
        az, true_accel, velocity, altitude))

    sleep_ms(50)
