from time import ticks_ms, ticks_diff, sleep_ms
from imu import MPU6050
from machine import Pin, I2C
import time

# Initialize I2C and IMU
i2c = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
imu = MPU6050(i2c)

velocity = 0
altitude = 0
prev_time = ticks_ms()
prev_accel = 0

while True:
    now = ticks_ms()
    dt = ticks_diff(now, prev_time) / 1000  # in seconds
    prev_time = now

    accel = imu.accel.z  # Raw acceleration in g's
    accel_m_s2 = accel * 9.81
    true_accel = accel_m_s2 - 9.81  # remove gravity
    
    # Integrate acceleration to get velocity
    velocity += 0.5 * (prev_accel + true_accel) * dt
    
    # Integrate velocity to get altitude
    altitude += velocity * dt

    prev_accel = true_accel

    print("Altitude estimate:", altitude, "m")
    sleep_ms(50)
