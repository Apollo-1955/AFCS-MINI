from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

# Kalman Filter for smoothing altitude
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

# MPU6050 class
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        # Wake up MPU6050
        self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')
        sleep_ms(100)

    def read_accel_z(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        raw_az = self._twos_complement(data[4] << 8 | data[5])
        az_g = raw_az / 16384.0  # in g's
        az_m_s2 = az_g * GRAVITY
        return az_m_s2

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Altitude estimator class
class AltitudeEstimator:
    def __init__(self, mpu):
        self.mpu = mpu
        self.velocity = 0.0
        self.altitude = 0.0
        self.last_time = ticks_ms()
        self.kalman = KalmanFilter()
        self.z_bias = 0.0

    def calibrate_bias(self, samples=100):
        print("Calibrating Z-axis... Hold still.")
        total = 0.0
        for _ in range(samples):
            total += self.mpu.read_accel_z()
            sleep_ms(10)
        self.z_bias = total / samples
        print(f"Z-bias calibrated: {self.z_bias:.3f} m/s²")

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        self.last_time = now

        az = self.mpu.read_accel_z()
        # Remove bias
        net_accel = az - self.z_bias

        # Fake force to counter unrealistic rapid drop in altitude during free fall or near zero net acceleration
        # This force smooths altitude changes when acceleration is near zero or negative
        if net_accel < -1.0:
            # Downward acceleration detected, apply damping
            damping_force = net_accel * 0.5
        elif net_accel > 0.1:
            # Upward acceleration, add a small boost to counter sensor noise
            damping_force = net_accel * 0.2
        else:
            # Near zero acceleration, minimal force
            damping_force = 0

        net_accel += damping_force

        # Integrate acceleration to velocity and velocity to altitude
        self.velocity += net_accel * dt

        # Limit velocity to prevent unrealistic high speeds (tune as needed)
        max_velocity = 20.0  # m/s max velocity cap
        if self.velocity > max_velocity:
            self.velocity = max_velocity
        elif self.velocity < -max_velocity:
            self.velocity = -max_velocity

        self.altitude += self.velocity * dt

        # Kalman filter smoothes altitude readings
        filtered_altitude = self.kalman.update(self.altitude)

        # Prevent altitude from dropping below 0 (ground level)
        if filtered_altitude < 0:
            filtered_altitude = 0
            self.velocity = 0

        return filtered_altitude

# Function to get next filename to avoid overwrite
def get_next_filename():
    base = "altitude"
    ext = ".csv"
    i = 1
    while True:
        filename = "{}_{}{}".format(base, i, ext)
        if filename not in os.listdir():
            return filename
        i += 1

# Main program
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
mpu = MPU6050(i2c)
alt_estimator = AltitudeEstimator(mpu)

alt_estimator.calibrate_bias()

filename = get_next_filename()
print(f"Logging altitude data to: {filename}")

# Write CSV header
with open(filename, "w") as f:
    f.write("Time(s),Altitude(m)\n")

log_buffer = []
start_time = ticks_ms()
last_write_time = ticks_ms()

while True:
    try:
        altitude = alt_estimator.update()
        current_time = ticks_diff(ticks_ms(), start_time) / 1000.0

        # Print altitude to terminal
        print("Time: {:.2f} s, Altitude: {:.3f} m".format(current_time, altitude))

        # Buffer the data line for writing
        log_buffer.append("{:.2f},{:.3f}\n".format(current_time, altitude))

        # Every 10 seconds, flush buffer to file
        if ticks_diff(ticks_ms(), last_write_time) >= 10000:
            with open(filename, "a") as f:
                f.writelines(log_buffer)
            log_buffer = []
            last_write_time = ticks_ms()

        sleep_ms(100)  # sample rate 10Hz

    except Exception as e:
        print("Error:", e)
        sleep_ms(1000)
