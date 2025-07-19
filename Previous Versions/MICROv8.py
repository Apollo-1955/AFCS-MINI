from machine import I2C, Pin
import time
import math

# You need a MicroPython MPU6050 driver. Example: https://github.com/micropython-IMU/micropython-mpu9x50
# For this example, assume you have a class MPU6050 that gives accel and gyro readings.

class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        # Initialize MPU6050 here

    def get_accel(self):
        # Return accel as tuple (ax, ay, az) in g's
        # Replace with your driver method
        return (0.0, 0.0, 0.0)

    def get_gyro(self):
        # Return gyro as tuple (gx, gy, gz) in degrees/s
        # Replace with your driver method
        return (0.0, 0.0, 0.0)

# Madgwick Filter Implementation (simplified)
class MadgwickAHRS:
    def __init__(self, sample_period=0.01, beta=0.1):
        self.beta = beta
        self.q = [1, 0, 0, 0]  # initial quaternion
        self.sample_period = sample_period

    def update(self, gx, gy, gz, ax, ay, az):
        # gx, gy, gz in radians/s
        # ax, ay, az normalized accel vector
        q1, q2, q3, q4 = self.q

        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return  # invalid data
        ax /= norm
        ay /= norm
        az /= norm

        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _4q3 = 4.0 * q3
        _8q2 = 8.0 * q2
        _8q3 = 8.0 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay

        norm_s = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 /= norm_s
        s2 /= norm_s
        s3 /= norm_s
        s4 /= norm_s

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * self.sample_period
        q2 += qDot2 * self.sample_period
        q3 += qDot3 * self.sample_period
        q4 += qDot4 * self.sample_period

        # Normalize quaternion
        norm_q = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = [q1 / norm_q, q2 / norm_q, q3 / norm_q, q4 / norm_q]

    def get_euler(self):
        q1, q2, q3, q4 = self.q
        # Roll (x-axis rotation)
        roll = math.atan2(2.0 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3))
        # Pitch (y-axis rotation)
        pitch = math.asin(2.0 * (q1 * q3 - q4 * q2))
        # Yaw (z-axis rotation)
        yaw = math.atan2(2.0 * (q1 * q4 + q2 * q3), 1 - 2 * (q3 * q3 + q4 * q4))

        # Convert to degrees
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# Setup I2C
i2c = I2C(1, scl=Pin(19), sda=Pin(18))
mpu = MPU6050(i2c)
madgwick = MadgwickAHRS(sample_period=0.05, beta=0.1)

while True:
    ax, ay, az = mpu.get_accel()  # in g
    gx, gy, gz = mpu.get_gyro()   # in degrees/s

    # Convert gyro degrees/s to radians/s
    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)

    madgwick.update(gx, gy, gz, ax, ay, az)
    roll, pitch, yaw = madgwick.get_euler()

    print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

    time.sleep(0.05)
