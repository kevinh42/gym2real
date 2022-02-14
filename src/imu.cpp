#include "imu.h"
#include <sstream>
#include <iostream>
#include <string.h>

IMU::IMU(int address)
    : initialized_(false)
    , address_(address)
{}

IMU::~IMU()
{
    i2c_close(bus_);
}

bool IMU::writeByte(int address, int value)
{
    unsigned char buf[1];
    size_t buf_size = sizeof(buf);
    buf[0] = value;
    size_t write_size;

    if ((write_size = i2c_write(&device_, address, buf, buf_size)) == -1) {
        return false;
    }

    return true;
}

bool IMU::readByte(int address, int* data)
{
    size_t byte_size = sizeof(uint8_t);
    size_t read_size;
    if ((read_size = i2c_read(&device_, address, data, byte_size)) == -1) {
        return false;
    } else if (read_size != byte_size) {
        return false;
    }

    return true;
}

bool IMU::readWord(int address, int* data)
{
    int low_byte;
    if (!readByte(address, data)) {     
        return false;
    }
    if (!readByte(address + 1, &low_byte)) {
        return false;
    }

    *data = (*data << 8) | low_byte;

    // Negative case
    if (*data >= 0x8000) {
        *data = -((65535 - *data) + 1);
    }

    return true;
}

bool IMU::init(int sda_pin, int scl_pin)
{
    std::stringstream ss;
    std::string sda_bus_name = i2c_sda_map.at(sda_pin);
    std::string scl_bus_name = i2c_scl_map.at(scl_pin);

    if (sda_bus_name != scl_bus_name) {
        printf("Pins are from mismatched I2C buses.\n");
        return false;
    }

    if ((bus_ = i2c_open(sda_bus_name.c_str())) == -1) {
        printf("Failed to open device the device [%s].\n", sda_bus_name.c_str());
        return false;
    }

    i2c_init_device(&device_);

    device_.addr = address_;
    device_.bus = bus_;

    if (!writeByte(PWR_MGMT_1, 0x00)) {
        printf("Failed to power on IMU at I2C address [%i].\n", address_);
        return false;
    } else {
        initialized_ = true;
    }

    return true;
}

void IMU::setAccelRange(AccelRange range)
{
    // Reset accelerometer config
    writeByte(ACCEL_CONFIG, 0x00);

    // Write range to accelerometer config
    writeByte(ACCEL_CONFIG, range);
}

void IMU::setGyroRange(GyroRange range)
{
    // Reset gyroscope config
    writeByte(GYRO_CONFIG, 0x00);

    // Write range to gyroscope config
    writeByte(GYRO_CONFIG, range);
}

int IMU::getAccelRange(bool raw)
{
    int raw_val;
    readByte(ACCEL_CONFIG, &raw_val);

    if (raw) {
        return raw_val;
    }

    switch (raw_val)
    {
    case ACCEL_RANGE_2G:
        return 2;
    case ACCEL_RANGE_4G:
        return 4;
    case ACCEL_RANGE_8G:
        return 8;
    case ACCEL_RANGE_16G:
        return 16;
    }

    return -1;
}

int IMU::getGyroRange(bool raw)
{
    int raw_val;
    readByte(GYRO_CONFIG, &raw_val);

    if (raw) {
        return raw_val;
    }

    switch (raw_val)
    {
    case GYRO_RANGE_250DEG:
        return 250;
    case GYRO_RANGE_500DEG:
        return 500;
    case GYRO_RANGE_1000DEG:
        return 1000;
    case GYRO_RANGE_2000DEG:
        return 2000;
    }

    return -1;
}

void IMU::getGyroData(Data* data)
{
    readWord(GYRO_XOUT0, &data->_x);
    readWord(GYRO_YOUT0, &data->_y);
    readWord(GYRO_ZOUT0, &data->_z);

    double scale_modifier;
    int range = getGyroRange(true);

    switch (range)
    {
    case GYRO_RANGE_250DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_250DEG;
        break;
    case GYRO_RANGE_500DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_500DEG;
        break;
    case GYRO_RANGE_1000DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_1000DEG;
        break;
    case GYRO_RANGE_2000DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_2000DEG;
        break;
    default:
        scale_modifier = GYRO_SCALE_MODIFIER_250DEG;
        break;
    }

    data->scale = 1.0 / scale_modifier;
}

void IMU::getAccelData(Data* data, bool g)
{
    readWord(ACCEL_XOUT0, &data->_x);
    readWord(ACCEL_YOUT0, &data->_y);
    readWord(ACCEL_ZOUT0, &data->_z);

    double scale_modifier;
    int range = getAccelRange(true);

    switch (range)
    {
    case ACCEL_RANGE_2G:
        scale_modifier = ACCEL_SCALE_MODIFIER_2G;
        break;
    case ACCEL_RANGE_4G:
        scale_modifier = ACCEL_SCALE_MODIFIER_4G;
        break;
    case ACCEL_RANGE_8G:
        scale_modifier = ACCEL_SCALE_MODIFIER_8G;
        break;
    case ACCEL_RANGE_16G:
        scale_modifier = ACCEL_SCALE_MODIFIER_16G;
        break;
    default:
        scale_modifier = ACCEL_SCALE_MODIFIER_2G;
        break;
    }

    data->scale = 1.0 / scale_modifier;

    if (!g) {
        data->scale *= GRAVITY_MS2;
    }
}

void IMU::getTempData(int* data)
{
    readWord(TEMP_OUT0, data);
}

void IMU::getData(DataIMU* data)
{
    memset(data, 0, sizeof(DataIMU));
    getAccelData(&data->accel);
    getGyroData(&data->gyro);
    getTempData(&data->_temp);
}