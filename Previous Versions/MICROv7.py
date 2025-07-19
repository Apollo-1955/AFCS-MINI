from machine import I2C, Pin
from imu import MPU6050
from time import ticks_ms, ticks_diff, sleep_ms
from math import sqrt, atan2, degrees, radians, cos, sin

class KalmanAngle:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0

        self.P = [[0, 0], [0, 0]]

    def update(self, new_angle, new_rate, dt):
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        y = new_angle - self.angle
        S = self.P[0][0] + self.R_measure

        K = [self.P[0][0] / S, self.P[1][0] / S]

        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

# I2C and MPU6050 setup
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)

# Set your calibrated offsets here (replace with your calibration)
ACCEL_BIAS = {'x': 0.0, 'y': 0.0, 'z': 1.0}  # In g's; z usually ~1 at rest
GYRO_BIAS = {'x': 0.0, 'y': 0.0, 'z': 0.0}   # In degrees per second

g = 9.81  # gravity constant

velocity = 0.0
altitude = 0.0

prev_time = ticks_ms()

kalman_pitch = KalmanAngle()
kalman_roll = KalmanAngle()

# Thresholds for detecting stationary ground (tune these)
GROUND_ACCEL_THRESHOLD = 0.2  # m/s² tolerance from 9.81g
GROUND_GYRO_THRESHOLD = 1.0   # degrees per second

on_ground = False

while True:
    now = ticks_ms()
    dt_ms = ticks_diff(now, prev_time)
    prev_time = now
    dt = dt_ms / 1000.0
    if dt <= 0:
        dt = 0.01  # Prevent zero or negative dt

    accel = mpu.accel
    gyro = mpu.gyro

    # Apply calibration and convert accel to m/s²
    ax = (accel.x - ACCEL_BIAS['x']) * g
    ay = (accel.y - ACCEL_BIAS['y']) * g
    az = (accel.z - ACCEL_BIAS['z']) * g

    # Calibrate gyro (degrees per second)
    gx = gyro.x - GYRO_BIAS['x']
    gy = gyro.y - GYRO_BIAS['y']
    gz = gyro.z - GYRO_BIAS['z']

    # Compute pitch and roll from accel (degrees)
    pitch_acc = degrees(atan2(-ax, sqrt(ay*ay + az*az)))
    roll_acc = degrees(atan2(ay, az))

    # Update Kalman filters with gyro rates and accel angles
    pitch = kalman_pitch.update(pitch_acc, gx, dt)
    roll = kalman_roll.update(roll_acc, gy, dt)

    # Convert pitch and roll to radians
    pitch_rad = radians(pitch)
    roll_rad = radians(roll)

    # Rotate accelerometer vector to Earth frame (compensate gravity)
    # First rotate around X (roll), then Y (pitch)
    ax1 = ax
    ay1 = ay * cos(roll_rad) + az * sin(roll_rad)
    az1 = -ay * sin(roll_rad) + az * cos(roll_rad)

    ax2 = ax1 * cos(pitch_rad) + az1 * sin(pitch_rad)
    ay2 = ay1
    az2 = -ax1 * sin(pitch_rad) + az1 * cos(pitch_rad)

    # az2 is vertical accel in Earth frame; subtract gravity
    vertical_accel = az2 - g

    # Calculate magnitude of accel and gyro for ground detection
    accel_mag = sqrt(ax*ax + ay*ay + az*az)
    gyro_mag = sqrt(gx*gx + gy*gy + gz*gz)

    # Detect ground (stationary)
    if abs(accel_mag - g) < GROUND_ACCEL_THRESHOLD and gyro_mag < GROUND_GYRO_THRESHOLD:
        if not on_ground:
            print(">>> Ground detected. Resetting altitude and velocity.")
            altitude = 0.0
            velocity = 0.0
            on_ground = True
    else:
        on_ground = False

    # Update altitude and velocity only if not on ground
    if not on_ground:
        displacement = velocity * dt + 0.5 * vertical_accel * dt * dt
        altitude += displacement
        velocity += vertical_accel * dt

    print("Pitch: {:.2f}°, Roll: {:.2f}°, Vert Accel: {:.2f} m/s² | Vel: {:.2f} m/s | Alt: {:.2f} m | On Ground: {}".format(
    pitch, roll, vertical_accel, velocity, altitude, on_ground))

    sleep_ms(50)
