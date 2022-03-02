#!/usr/bin/python3
import argparse
import time

import numpy as np
import smbus

# MPU6050 register addresses
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(device_address, SMPLRT_DIV, 7)

    # write to power management register
    bus.write_byte_data(device_address, PWR_MGMT_1, 1)

    # write to configuration register
    bus.write_byte_data(device_address, CONFIG, 0)

    # write to gyro configuration register
    bus.write_byte_data(device_address, GYRO_CONFIG, 24)

    # write to interrupt enable register
    bus.write_byte_data(device_address, INT_ENABLE, 1)

def read_raw_data(addr):
    # accelero and gyro values are 16-bit
    high = bus.read_byte_data(device_address, addr)
    low = bus.read_byte_data(device_address, addr+1)

    # concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536

    return value

bus = smbus.SMBus(1)
device_address = 0x68

MPU_Init() # initialize the IMU


# Read IMU values and return Ax,Ay,Az,Gx,Gy,Gz
def get_reading():
    # read accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    # read gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    # full scale range +/- 250 degrees/C
    ACC_SCL = 16384.0
    Ax = acc_x/ACC_SCL
    Ay = acc_y/ACC_SCL
    Az = acc_z/ACC_SCL
    # Read Gyroscope data
    GYR_SCL = 131.0
    Gx = gyro_x/GYR_SCL
    Gy = gyro_y/GYR_SCL
    Gz = gyro_z/GYR_SCL
    return Ax, Ay, Az, Gx, Gy, Gz

# read from the IMU n_sample times and calculate the
def calc_mean_var(n_samples: int):
    readings = np.array([get_reading() for x in range(n_samples)]) # (n_samples, 6)
    means = readings.mean(axis=0) # (6,)
    variance = readings.var(axis=0) # (6,)
    print("Ax, Ay, Az, Gx, Gy, Gz")
    print(f"Mean: {means}")
    print(f"Variance:{variance}")

# get phi dt seconds after previous reading
def get_phi(dt = 1e-3, prev_phi=0, alpha=0.98):
    Ax, Ay, Az, Gx, Gy, Gz = get_reading()
    raw_pitch = np.arctan2(Ax, np.sqrt(Ay**2 + Az**2)) # radians, very noisy
    gyro_pitch = Gx*dt * np.pi/180  + prev_phi # integrate gyro reading
    comp_pitch = raw_pitch * (1-alpha) + alpha * gyro_pitch # angle fusion with complementray filter
    return comp_pitch


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='IMU Calibration')
    argparser.add_argument('--n_samples', type=int, default=100, help='number of samples for error calculation: default 100')
    argparser.add_argument('--sample_rate', type=float, default=1000, help='sample rate in Hz between samples: default 1000')
    argparser.add_argument('--calc_stats', type=int, choices=[0,1], default=0, help='calculate mean and variance with n_samples, 0 == False, 1 == True: default 0')
    args = argparser.parse_args()

    if args.calc_stats:
        calc_mean_var(n_samples=args.n_samples)


    phi0 = 0
    sample_period = 1/(args.sample_rate)
    phi = get_phi(0, phi0)
    while(True):
        t0 = time.time()
        time.sleep(sample_period)
        dt = time.time() - t0
        phi = get_phi(dt, phi)
        print(f"phi (degrees): {phi * 180/(np.pi)}")
