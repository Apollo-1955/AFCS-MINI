from machine import I2C, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import math
import os

# Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GRAVITY = 9.80665

led = Pin(25, Pin.OUT)  # Pico onboard LED

# Kalman filter
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

# MPU6050 driver
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

# Altitude estimation
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
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)
        net_acc_z = az - self.z_bias

        fake_force_strength = 0.2
        free_fall_threshold = 1.0

        if acc_mag < free_fall_threshold:
            if net_acc_z < 0:
                net_acc_z += fake_force_strength
            else:
                net_acc_z -= fake_force_strength
            self.velocity *= 0.7
        else:
            if net_acc_z > 0:
                self.velocity += net_acc_z * dt
            elif net_acc_z < -0.1:
                self.velocity += net_acc_z * dt

        self.velocity = max(min(self.velocity, 50.0), -50.0)
        self.altitude += self.velocity * dt

        if self.altitude < 0:
            self.altitude = 0
            self.velocity = 0

        filtered_alt = self.kalman.update(self.altitude)
        return filtered_alt, net_acc_z

# Generate unique filename
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

# Main function
def main():
    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
    mpu = MPU6050(i2c)
    alt_estimator = AltitudeEstimator(mpu)

    alt_estimator.calibrate()

    filename = get_next_filename()
    print("Logging altitude data to:", filename)

    f = open(filename, "w")
    f.write("Time(s),Altitude(m),Accel_Z(m/s^2),Event\n")

    start_time = ticks_ms()
    launched = False
    apogee_detected = False
    landed = False
    max_altitude = 0.0
    led_timer = 0

    while True:
        try:
            altitude, az = alt_estimator.update()
            current_time = ticks_diff(ticks_ms(), start_time) / 1000.0
            event = ""

            # Launch detection
            if not launched and az > 1.5:
                launched = True
                event = "LAUNCH"
                print("🚀 LAUNCH detected at {:.2f}s".format(current_time))

            # Apogee detection
            if launched and not apogee_detected:
                if altitude > max_altitude:
                    max_altitude = altitude
                elif altitude < max_altitude - 0.1:
                    apogee_detected = True
                    event = "APOGEE"
                    print("🔺 APOGEE detected at {:.2f}s".format(current_time))

            # Landed detection
            if apogee_detected and not landed:
                if altitude <= 0.1 and abs(az) < 1.5:
                    landed = True
                    event = "LANDED"
                    print("✅ LANDED detected at {:.2f}s".format(current_time))

            # LED behavior update
            led_timer += 1
            if launched and not apogee_detected:
                led.value(1)  # Solid ON during launch until apogee
            elif apogee_detected and not landed:
                if led_timer % 20 == 0:
                    led.toggle()  # Slow blink after apogee detected
            elif landed:
                if led_timer % 5 == 0:
                    led.toggle()  # Fast blink after landing
            else:
                led.value(0)  # Off before launch

            # Terminal & file log
            print("Altitude: {:.2f} m | Accel Z: {:.2f} m/s²".format(altitude, az))
            f.write("{:.2f},{:.3f},{:.2f},{}\n".format(current_time, altitude, az, event))

            # Save and close file after 25 seconds
            if ticks_diff(ticks_ms(), start_time) >= 25000:
                print("📁 File saved and closed: {}".format(filename))
                f.close()
                break

        except Exception as e:
            print("Error:", e)

        sleep_ms(100)

if __name__ == "__main__":
    main()
