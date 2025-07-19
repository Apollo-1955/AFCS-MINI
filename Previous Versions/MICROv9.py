from machine import I2C, Pin
import time
import math

# -- MPU6050 driver for raw accel + gyro data --
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        # Wake up MPU6050 (exit sleep mode)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
    
    def read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        # accel raw
        ax = int.from_bytes(data[0:2], 'big', signed=True)
        ay = int.from_bytes(data[2:4], 'big', signed=True)
        az = int.from_bytes(data[4:6], 'big', signed=True)
        # gyro raw
        gx = int.from_bytes(data[8:10], 'big', signed=True)
        gy = int.from_bytes(data[10:12], 'big', signed=True)
        gz = int.from_bytes(data[12:14], 'big', signed=True)
        return ax, ay, az, gx, gy, gz

# -- Madgwick filter implementation --
class MadgwickAHRS:
    def __init__(self, sample_freq=100.0, beta=0.1):
        self.beta = beta
        self.sample_freq = sample_freq
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update_imu(self, gx, gy, gz, ax, ay, az):
        # Convert gyro deg/sec to rad/sec
        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3

        # Normalize accelerometer measurement
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return  # avoid division by zero
        ax /= norm
        ay /= norm
        az /= norm

        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        # Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay

        norm_s = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm_s == 0:
            return
        s0 /= norm_s
        s1 /= norm_s
        s2 /= norm_s
        s3 /= norm_s

        # Compute rate of change of quaternion
        qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - self.beta * s0
        qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - self.beta * s1
        qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - self.beta * s2
        qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - self.beta * s3

        # Integrate to yield quaternion
        self.q0 += qDot0 / self.sample_freq
        self.q1 += qDot1 / self.sample_freq
        self.q2 += qDot2 / self.sample_freq
        self.q3 += qDot3 / self.sample_freq

        # Normalize quaternion
        norm_q = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 /= norm_q
        self.q1 /= norm_q
        self.q2 /= norm_q
        self.q3 /= norm_q

    def get_euler(self):
        # Quaternion to Euler angles (degrees)
        roll = math.atan2(2*(self.q0*self.q1 + self.q2*self.q3), 1 - 2*(self.q1*self.q1 + self.q2*self.q2))
        pitch = math.asin(2*(self.q0*self.q2 - self.q3*self.q1))
        yaw = math.atan2(2*(self.q0*self.q3 + self.q1*self.q2), 1 - 2*(self.q2*self.q2 + self.q3*self.q3))
        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)

# -- Scale raw data to physical units --
def scale_gyro(raw):
    # Sensitivity scale factor for ±250 deg/sec: 131 LSB/(deg/s)
    return raw / 131.0

def scale_accel(raw):
    # Sensitivity scale factor for ±2g: 16384 LSB/g
    return raw / 16384.0

# -- Low pass filter for smoothing angles --
class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.state = None
    
    def update(self, value):
        if self.state is None:
            self.state = value
        else:
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state

# -- Main setup --
i2c = I2C(1, scl=Pin(19), sda=Pin(18))  # Adjust pins to your board!
mpu = MPU6050(i2c)
madgwick = MadgwickAHRS(sample_freq=100, beta=0.1)
pitch_filter = LowPassFilter(alpha=0.1)

print("MPU6050 + Madgwick sensor fusion starting...")

while True:
    ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = mpu.read_raw()
    
    # Convert raw data to g and deg/s
    ax = scale_accel(ax_raw)
    ay = scale_accel(ay_raw)
    az = scale_accel(az_raw)
    gx = scale_gyro(gx_raw)
    gy = scale_gyro(gy_raw)
    gz = scale_gyro(gz_raw)
    
    # Update Madgwick filter with new IMU data
    madgwick.update_imu(gx, gy, gz, ax, ay, az)
    
    # Get Euler angles
    yaw, pitch, roll = madgwick.get_euler()
    
    # Apply low-pass filter on pitch (altitude-like)
    pitch_smoothed = pitch_filter.update(pitch)
    
    print("Yaw: {:.2f}°, Pitch: {:.2f}°, Roll: {:.2f}°, Pitch_filtered: {:.2f}°".format(
        yaw, pitch, roll, pitch_smoothed))
    
    time.sleep(0.01)  # 100 Hz
