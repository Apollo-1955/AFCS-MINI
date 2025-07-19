from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

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

        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

        # Apply fake force if in free fall
        free_fall_threshold = 1.0
        fake_force_strength = 0.15
        if acc_mag < free_fall_threshold:
            if net_acc_z < 0:
                net_acc_z += fake_force_strength
            else:
                net_acc_z -= fake_force_strength
            self.velocity *= 0.7
        else:
            self.velocity *= 0.98  # Light decay

        # Apply deadzone for small positive (upward) acceleration only
        pos_deadzone = 0.07
        if net_acc_z > 0 and net_acc_z < pos_deadzone:
            net_acc_z = 0.0
        # NOTE: negative acceleration (downward) is NOT deadzoned

        self.velocity += net_acc_z * dt

        # Clamp velocity
        max_velocity = 50.0
        if self.velocity > max_velocity:
            self.velocity = max_velocity
        elif self.velocity < -max_velocity:
            self.velocity = -max_velocity

        self.altitude += self.velocity * dt

        # Ground check
        if self.altitude < 0:
            self.altitude = 0.0
            self.velocity = 0.0

        return self.kalman.update(self.altitude)

def get_next_filename(base_name="altitude", ext=".csv"):
    try:
        files = os.listdir()
    except Exception:
        files = []
    number = 1
    while True:
        filename = "{}_{}.csv".format(base_name, number)
        if filename not in files:
            return filename
        number += 1

def main():
    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
    mpu = MPU6050(i2c)
    alt_estimator = AltitudeEstimator(mpu)

    alt_estimator.calibrate()

    filename = get_next_filename()
    print("Logging altitude data to:", filename)

    with open(filename, "w") as f:
        f.write("Time(s),Altitude(m)\n")

    log_buffer = []
    start_time = ticks_ms()
    last_write_time = start_time

    while True:
        try:
            altitude = alt_estimator.update()
            current_time = ticks_diff(ticks_ms(), start_time) / 1000.0

            print("Altitude: {:.2f} m".format(altitude))

            log_buffer.append("{:.2f},{:.3f}\n".format(current_time, altitude))

            if ticks_diff(ticks_ms(), last_write_time) >= 10000:
                with open(filename, "a") as f:
                    for line in log_buffer:
                        f.write(line)
                log_buffer = []
                last_write_time = ticks_ms()

        except Exception as e:
            print("Error:", e)

        sleep_ms(100)

if __name__ == "__main__":
    main()
