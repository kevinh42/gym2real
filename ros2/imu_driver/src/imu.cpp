#include "imu_driver/imu.h"
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

void IMU::checkInitialized()
{
    if (!initialized_) {
        throw std::runtime_error("The IMU has not been initialized.");
    }
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

bool IMU::readByte(int address, int8_t* data)
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

bool IMU::readWord(int address, int16_t* data)
{
    size_t word_size = sizeof(uint16_t);
    size_t read_size;
    if ((read_size = i2c_read(&device_, address, data, word_size)) == -1) {
        return false;
    } else if (read_size != word_size) {
        return false;
    }

    *data = (int16_t) be16toh((uint16_t) *data);

    return true;
}

bool IMU::readXYZ(int address, int16_t* data)
{
    size_t xyz_size = sizeof(uint16_t) * 3;
    size_t read_size;
    if ((read_size = i2c_read(&device_, address, data, xyz_size)) == -1) {
        return false;
    } else if (read_size != xyz_size) {
        return false;
    }

    for (int i = 0; i < 3; i++)
    {
        data[i] = (int16_t) be16toh((uint16_t) data[i]);
    }

    return true;
}

bool IMU::init(int sda_pin, int scl_pin)
{
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

void IMU::configure(int address, int value)
{
    checkInitialized();

    // Write value to address
    writeByte(address, value);
}

void IMU::setAccelRange(AccelRange range)
{
    checkInitialized();

    // Reset accelerometer config
    writeByte(ACCEL_CONFIG, 0x00);

    // Write range to accelerometer config
    writeByte(ACCEL_CONFIG, range);
}

void IMU::setGyroRange(GyroRange range)
{
    checkInitialized();

    // Reset gyroscope config
    writeByte(GYRO_CONFIG, 0x00);

    // Write range to gyroscope config
    writeByte(GYRO_CONFIG, range);
}

int IMU::getAccelRange(bool raw)
{
    checkInitialized();

    int8_t raw_val;
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
    checkInitialized();

    int8_t raw_val;
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

double IMU::getGyroScale(const int& range)
{
    double scale_modifier;
    switch (range)
    {
    case GYRO_RANGE_250DEG:
        scale_modifier = GYRO_SCALE_MODIFIER_250DEG;
        break;
    case GYRO_RANGE_500DEG:
        scale_modifier =  GYRO_SCALE_MODIFIER_500DEG;
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

    return 1.0 / scale_modifier;
}

void IMU::getGyroData(Data* data)
{
    checkInitialized();

    readXYZ(GYRO_XOUT0, &data->_xyz[0]);

    int range = getGyroRange(true);
    double scale = getGyroScale(range);

    data->scale = scale;
}

double IMU::getAccelScale(const int& range, bool g)
{
    double scale_modifier;
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

    double gravity_scale = g ? 1.0 : GRAVITY_MS2;        

    return 1.0 / scale_modifier * gravity_scale;
}

void IMU::getAccelData(Data* data, bool g)
{
    checkInitialized();

    readXYZ(ACCEL_XOUT0, &data->_xyz[0]);

    int range = getAccelRange(true);
    double scale = getAccelScale(range, g);

    data->scale = scale;
}

void IMU::getTempData(int* data)
{
    checkInitialized();

    readWord(TEMP_OUT0, (int16_t*)data);
}

bool IMU::getData(DataIMU* data)
{
    checkInitialized();

    memset(data, 0, sizeof(DataIMU));

    int accel_range = getAccelRange();
    data->accel().scale = getAccelScale(accel_range, false);

    int gyro_range = getGyroRange();
    data->gyro().scale = getGyroScale(gyro_range);

    size_t data_size = sizeof(data->_data);
    size_t read_size;
    if ((read_size = i2c_read(&device_, ACCEL_XOUT0, &data->_data[0], data_size)) == -1) {
        return false;
    } else if (read_size != data_size) {
        return false;
    }

    for (int i = 0; i < 7; i++)
    {
        data->_data[i] = (int16_t) be16toh((uint16_t) data->_data[i]);
    }

    return true;
}