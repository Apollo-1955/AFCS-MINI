from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

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
        net_acc_z = az - self.z_bias  # Remove gravity bias on Z axis

        # Calculate total acceleration magnitude (without gravity bias)
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

        # Detect free fall: acc magnitude near zero (or below threshold)
        free_fall_threshold = 1.0  # m/s², adjust if needed

        # Fake force to reduce zero-G effect (when acceleration magnitude is low)
        fake_force_strength = 0.15  # max correction in m/s²

        if acc_mag < free_fall_threshold:
            # In free fall or near free fall, do NOT integrate downward acceleration blindly
            # Instead, apply small fake force pushing acceleration towards zero
            if net_acc_z < 0:
                net_acc_z += fake_force_strength
            else:
                net_acc_z -= fake_force_strength

            # Also, apply strong velocity damping to hold altitude stable during free fall
            self.velocity *= 0.7  # stronger damping in free fall
        else:
            # Normal conditions: mild velocity damping to reduce drift
            self.velocity *= 0.95

        # Deadzone for small noise in net acceleration
        acc_deadzone = 0.03
        if abs(net_acc_z) < acc_deadzone:
            net_acc_z = 0.0

        # Integrate velocity
        self.velocity += net_acc_z * dt

        # Clamp velocity to plausible physical max (e.g., terminal velocity ~50 m/s)
        max_velocity = 50.0
        if self.velocity > max_velocity:
            self.velocity = max_velocity
        elif self.velocity < -max_velocity:
            self.velocity = -max_velocity

        # Integrate altitude
        self.altitude += self.velocity * dt

        # Clamp altitude to >= 0 (no negative altitude)
        if self.altitude < 0:
            self.altitude = 0.0
            self.velocity = 0.0

        # Kalman filter smooth altitude output
        filtered_alt = self.kalman.update(self.altitude)

        return filtered_alt

def get_next_filename(base_name="altitude", ext=".csv"):
    try:
        existing = [f for f in os.listdir() if f.startswith(base_name) and f.endswith(ext)]
    except:
        existing = []
    numbers = []
    for fname in existing:
        try:
            num = int(fname[len(base_name):-len(ext)])
            numbers.append(num)
        except:
            pass
    next_num = max(numbers) + 1 if numbers else 1
    return "{}{}.{}".format(base_name, next_num, ext)

# Main program
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
mpu = MPU6050(i2c)
alt_estimator = AltitudeEstimator(mpu)

alt_estimator.calibrate()

filename = get_next_filename()
print("Logging altitude data to:", filename)

buffer = []
start_time = ticks_ms()

with open(filename, "w") as f:
    f.write("time_sec,altitude_m\n")

while True:
    try:
        alt = alt_estimator.update()
        elapsed_s = (ticks_ms() - start_time) / 1000.0
        print("Altitude: {:.2f} m".format(alt))

        buffer.append("{:.2f},{:.3f}\n".format(elapsed_s, alt))

        if elapsed_s > 0 and int(elapsed_s) % 10 == 0 and len(buffer) > 0:
            with open(filename, "a") as f:
                f.writelines(buffer)
            buffer.clear()

    except Exception as e:
        print("Error:", e)

    sleep_ms(100)
