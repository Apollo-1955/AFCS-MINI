from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

# Kalman Filter class
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

# MPU6050
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        try:
            self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')
            sleep_ms(100)
        except OSError as e:
            raise Exception("MPU6050 not found") from e

    def read_accel_z(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        az = self._twos_complement(data[4] << 8 | data[5]) / 16384.0 * GRAVITY
        return az

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Altitude Estimator
class AltitudeEstimator:
    def __init__(self, mpu):
        self.mpu = mpu
        self.velocity = 0
        self.altitude = 0
        self.last_time = ticks_ms()
        self.kalman = KalmanFilter()
        self.z_bias = 0

    def calibrate(self, samples=100, delay_ms=20):
        print("Calibrating Z-axis... Hold still.")
        total = 0
        for _ in range(samples):
            total += self.mpu.read_accel_z()
            sleep_ms(delay_ms)
        self.z_bias = total / samples
        print("Z-bias calibrated: {:.3f} m/s²".format(self.z_bias))

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        if dt <= 0:
            dt = 0.01
        self.last_time = now

        az = self.mpu.read_accel_z()
        net_acc = az - self.z_bias

        # Fake force to reduce zero-G effect in free fall
        if -0.3 < net_acc < 0.3:
            fake_force = 0.3 * (0.3 - abs(net_acc)) / 0.3  # max 0.3 m/s²
            if net_acc < 0:
                net_acc += fake_force
            else:
                net_acc -= fake_force

        # Clip acceleration to realistic limits
        max_acc = 20.0
        net_acc = max(min(net_acc, max_acc), -max_acc)

        # Deadzone threshold to filter noise on net_acc around zero
        acc_deadzone = 0.05
        if abs(net_acc) < acc_deadzone:
            net_acc = 0.0

        # Increase damping to reduce velocity drift
        damping_factor = 0.85  # stronger damping

        # Update velocity with damping and acceleration
        self.velocity = self.velocity * damping_factor + net_acc * dt

        # Velocity reset if small velocity and acceleration to hold altitude steady
        vel_threshold = 0.02
        if abs(self.velocity) < vel_threshold and abs(net_acc) < acc_deadzone:
            self.velocity = 0.0

        # Integrate altitude
        self.altitude += self.velocity * dt

        # Smooth altitude with Kalman filter
        filtered_alt = self.kalman.update(self.altitude)
        return filtered_alt

# Utility to find next filename number
def get_next_filename(base_name="altitude", ext=".csv"):
    existing = [f for f in os.listdir() if f.startswith(base_name) and f.endswith(ext)]
    numbers = []
    for fname in existing:
        try:
            num = int(fname[len(base_name):-len(ext)])
            numbers.append(num)
        except:
            pass
    next_num = max(numbers) + 1 if numbers else 1
    return "{}{}.{}".format(base_name, next_num, ext)

# Main
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
mpu = MPU6050(i2c)
alt_estimator = AltitudeEstimator(mpu)

alt_estimator.calibrate()

filename = get_next_filename()
print("Logging altitude data to:", filename)

buffer = []
start_time = ticks_ms()

with open(filename, "w") as file:
    file.write("time_sec,altitude_m\n")  # CSV header

while True:
    try:
        alt = alt_estimator.update()
        elapsed_s = (ticks_ms() - start_time) / 1000.0
        print("Altitude: {:.2f} m".format(alt))

        # Add to buffer
        buffer.append("{:.2f},{:.3f}\n".format(elapsed_s, alt))

        # Every 10 seconds, write buffer to file and clear buffer
        if elapsed_s > 0 and int(elapsed_s) % 10 == 0 and len(buffer) > 0:
            with open(filename, "a") as f:
                f.writelines(buffer)
            buffer.clear()

    except Exception as e:
        print("Error:", e)

    sleep_ms(100)
