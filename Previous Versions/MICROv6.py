from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import atan2, sqrt, cos, sin, radians, pi

# === I2C SETUP ===
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)

# === CALIBRATION (replace with your calibration) ===
ACCEL_BIAS = {'x': 0.01, 'y': -0.02, 'z': 1.00}
GYRO_BIAS = {'x': -0.5, 'y': 0.2, 'z': 0.1}

# === FILTER CLASSES ===
class SimpleKalman:
    def __init__(self, q=0.01, r=0.3, p=1, x=0):
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

# Simple low-pass filter
def low_pass_filter(prev, curr, alpha=0.2):
    return alpha * curr + (1 - alpha) * prev

# === VARIABLES ===
velocity1 = 0        # from double integration
altitude1 = 0

velocity_zupt = 0    # velocity with zero-velocity updates
altitude_zupt = 0

altitude_norm = 0    # altitude estimated from acceleration norm

prev_time = ticks_ms()
prev_accel_vert = 0
prev_accel_norm = 9.81
kalman_vert = SimpleKalman()
kalman_norm = SimpleKalman()
LPF_ALPHA = 0.2

pitch = 0.0

# Complementary filter constant for pitch
alpha_cf = 0.98

launched = False
LAUNCH_ACCEL_THRESHOLD = 2.0

print("Starting combined altitude estimation...\n")

while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000
    prev_time = now

    accel = mpu.accel
    gyro = mpu.gyro

    # Calibrate accel & gyro
    ax = (accel.x - ACCEL_BIAS['x']) * 9.81
    ay = (accel.y - ACCEL_BIAS['y']) * 9.81
    az = (accel.z - ACCEL_BIAS['z']) * 9.81

    gx = gyro.x - GYRO_BIAS['x']

    # === Pitch estimation (complementary filter) ===
    pitch_acc = atan2(ax, sqrt(ay*ay + az*az)) * (180/pi)
    pitch += gx * dt  # gyro integration
    pitch = alpha_cf * pitch + (1 - alpha_cf) * pitch_acc
    pitch_rad = radians(pitch)

    # === 1. Double integration method ===
    # Project acceleration on vertical axis
    acc_vert = az * cos(pitch_rad) + ax * sin(pitch_rad)

    # Remove gravity (assuming Z points down)
    true_accel_vert = acc_vert - 9.81

    # Filter vertical accel
    filtered_accel_vert = kalman_vert.update(true_accel_vert)
    filtered_accel_vert = low_pass_filter(prev_accel_vert, filtered_accel_vert, LPF_ALPHA)
    prev_accel_vert = filtered_accel_vert

    # Launch detection to start integration
    if not launched:
        if abs(filtered_accel_vert) > LAUNCH_ACCEL_THRESHOLD:
            launched = True
            velocity1 = 0
            altitude1 = 0
            velocity_zupt = 0
            altitude_zupt = 0
    else:
        # Integrate velocity and altitude (method 1)
        velocity1 += filtered_accel_vert * dt
        altitude1 += velocity1 * dt

    # === 2. Zero-velocity update (ZUPT) on method 1 ===
    # If accel near zero, reset velocity to zero
    ACCEL_ZERO_THRESHOLD = 0.3
    if abs(filtered_accel_vert) < ACCEL_ZERO_THRESHOLD:
        velocity_zupt = 0
    else:
        velocity_zupt += filtered_accel_vert * dt
    altitude_zupt += velocity_zupt * dt

    # === 3. Acceleration norm method ===
    acc_norm = sqrt(ax*ax + ay*ay + az*az)

    # Remove gravity component approx
    acc_norm_no_gravity = acc_norm - 9.81

    # Filter norm accel
    filtered_accel_norm = kalman_norm.update(acc_norm_no_gravity)
    filtered_accel_norm = low_pass_filter(prev_accel_norm, filtered_accel_norm, LPF_ALPHA)
    prev_accel_norm = filtered_accel_norm

    # Integrate velocity and altitude from norm
    velocity_norm = filtered_accel_norm * dt if launched else 0
    altitude_norm += velocity_norm * dt

    # === 4. Combine altitude estimates (weighted average) ===
    # Weights can be tuned (sum to 1)
    w1 = 0.6   # weight for double integration method
    w2 = 0.4   # weight for norm method

    combined_altitude = w1 * altitude1 + w2 * altitude_norm

    # === Print debug info ===
    print("Pitch:{:.1f}° Acc_v:{:.2f} m/s² Alt1:{:.2f} m Alt_norm:{:.2f} m Comb_alt:{:.2f} m".format(
        pitch, filtered_accel_vert, altitude1, altitude_norm, combined_altitude))

    sleep_ms(50)
