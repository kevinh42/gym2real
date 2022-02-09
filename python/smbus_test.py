import smbus
from time import sleep

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

MPU_Init()

print("Reading data from gyroscope and accelerometer")

while True:
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

    GYR_SCL = 131.0
    Gx = gyro_x/GYR_SCL
    Gy = gyro_y/GYR_SCL
    Gz = gyro_z/GYR_SCL

    print ("Gx=%.2f" %Gx, u'\u00b0' + "/s", 
         "\tGy=%.2f" %Gy, u'\u00b0' + "/s",
         "\tGz=%.2f" %Gz, u'\u00b0' + "/s",
         "\tAx=%.2f g" %Ax, u'\u00b0' + "/s",
         "\tAy=%.2f g" %Ay, u'\u00b0' + "/s",
         "\tAz=%.2f g" %Az, u'\u00b0' + "/s")
    sleep(0.1)