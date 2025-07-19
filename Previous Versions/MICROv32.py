from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

# Kalman Filter
class KalmanFilter:
    def __init__(self, Q=0.05, R=0.3):
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

# MPU6050 Driver
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        try:
            self.i2c.writeto_mem(self.addr, PWR_MGMT_1, b'\x00')
            sleep_ms(100)
        except OSError as e:
            raise Exception("MPU6050 not found") from e

    def read_accel(self):
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 6)
        ax = self._twos_complement(data[0] << 8 | data[1]) / 16384.0 * GRAVITY
        ay = self._twos_complement(data[2] << 8 | data[3]) / 16384.0 * GRAVITY
        az = self._twos_complement(data[4] << 8 | data[5]) / 16384.0 * GRAVITY
        return ax, ay, az

    def _twos_complement(self, val):
        return val - 65536 if val > 32767 else val

# Altitude Estimator
class AltitudeEstimator:
    def __init__(self, mpu):
        self.mpu = mpu
        self.velocity = 0.0
        self.altitude = 0.0
        self.last_time = ticks_ms()
        self.kalman = KalmanFilter()
        self.z_bias = 0.0

    def calibrate(self, samples=100, delay_ms=20):
        print("Calibrating Z-axis... Hold still.")
        total_az = 0.0
        for _ in range(samples):
            _, _, az = self.mpu.read_accel()
            total_az += az
            sleep_ms(delay_ms)
        self.z_bias = total_az / samples
        print("Z-bias calibrated: {:.3f} m/s²".format(self.z_bias))

    def update(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        if dt <= 0:
            dt = 0.01
        self.last_time = now

        ax, ay, az = self.mpu.read_accel()
        net_acc_z = az - self.z_bias
        acc_mag = math.sqrt(ax**2 + ay**2 + az**2)

        # Free-fall detection with fake force
        fake_force = 0.15
        if acc_mag < 1.0:
            if net_acc_z < 0:
                net_acc_z += fake_force
            else:
                net_acc_z -= fake_force
            self.velocity *= 0.7
        else:
            if net_acc_z > 0:
                self.velocity *= 1.0  # allow clean ascent
            else:
                self.velocity *= 0.96  # slight damping when descent starts

        # Only allow descent if net_acc_z is negative
        if net_acc_z < 0:
            self.velocity += net_acc_z * dt
        elif net_acc_z > 0 or self.velocity > 0:
            # Still allow upward motion even if slowing
            self.velocity += net_acc_z * dt

        # Clamp velocity for safety
        self.velocity = max(min(self.velocity, 50.0), -50.0)

        self.altitude += self.velocity * dt

        # Prevent negative altitude
        if self.altitude < 0:
            self.altitude = 0.0
            self.velocity = 0.0

        return self.kalman.update(self.altitude)

# Helper: Next available filename
def get_next_filename(base="altitude", ext=".csv"):
    try:
        files = os.listdir()
    except Exception:
        files = []
    n = 1
    while True:
        name = f"{base}_{n}{ext}"
        if name not in files:
            return name
        n += 1

# Main Program
def main():
    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
    mpu = MPU6050(i2c)
    estimator = AltitudeEstimator(mpu)

    estimator.calibrate()
    filename = get_next_filename()
    print("Logging altitude data to:", filename)

    with open(filename, "w") as f:
        f.write("Time(s),Altitude(m)\n")

    log_buffer = []
    start_time = ticks_ms()
    last_write_time = start_time

    while True:
        try:
            altitude = estimator.update()
            current_time = ticks_diff(ticks_ms(), start_time) / 1000.0

            print("Altitude: {:.2f} m".format(altitude))

            log_buffer.append("{:.2f},{:.2f}\n".format(current_time, altitude))

            if ticks_diff(ticks_ms(), last_write_time) >= 25000:
                with open(filename, "a") as f:
                    f.writelines(log_buffer)
                log_buffer = []
                last_write_time = ticks_ms()

        except Exception as e:
            print("Error:", e)

        sleep_ms(100)

if __name__ == "__main__":
    main()
