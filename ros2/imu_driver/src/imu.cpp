#include "imu_driver/imu.h"
#include <sstream>
#include <iostream>
#include <string.h>

#include <endian.h>

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

bool IMU::readByte(int address, int* data)
{
    //std::cout<<"Read byte start"<<std::endl;
    size_t byte_size = sizeof(uint8_t);
    size_t read_size;
    if ((read_size = i2c_read(&device_, address, data, byte_size)) == -1) {
        std::cout<<"Read byte false"<<std::endl;
        return false;
    } else if (read_size != byte_size) {
        std::cout<<"Read byte false2"<<std::endl;
        return false;
    }
    //std::cout<<"Read byte true"<<std::endl;
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

    if (!writeByte(SMPLRT_DIV, 0x07)) {
        printf("Failed to change sample rate\n");
        return false;
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
    checkInitialized();

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

int IMU::getInterruptStatus(){
    int raw_val;
    readByte(INT_STATUS, &raw_val);
    return raw_val;
}

void IMU::getGyroData(Data* data)
{
    checkInitialized();

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
    checkInitialized();

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
    checkInitialized();

    readWord(TEMP_OUT0, data);
}

void IMU::getData(DataIMU* data)
{
    checkInitialized();

    memset(data, 0, sizeof(DataIMU));
    getAccelData(&data->accel);
    getGyroData(&data->gyro);
    getTempData(&data->_temp);
}

void IMU::getDataFast(DataIMU* data)
{
    // should modify this to clean it up
    // uses a single read to access all the measurements at once
    checkInitialized();
    memset(data, 0, sizeof(DataIMU));

    int16_t packed_data[7];
    
    size_t data_size = sizeof(packed_data); // read all 6 * 2 bytes of data at once
    size_t read_size;
    if ((read_size = i2c_read(&device_, ACCEL_XOUT0, &packed_data, data_size)) == -1) {
        std::cout<<"Failed read fast 1"<<std::endl;
    } else if (read_size != data_size) {
        std::cout<<"Failed read fast 2"<<std::endl;
    }
    else{
        for (int i = 0; i < 7; i++)
        {
            packed_data[i] = (int16_t) be16toh((uint16_t) packed_data[i]); // convert big endian to host type, this also handles overflows for negative values
        }

        // read accel address
        data->accel._x = packed_data[0];
        data->accel._y = packed_data[1];
        data->accel._z = packed_data[2];
        data->accel.scale = 1.0 / ACCEL_SCALE_MODIFIER_2G;

        // read temp address
        data->_temp = packed_data[3];

        // read gyro address
        data->gyro._x = packed_data[4];
        data->gyro._y = packed_data[5];
        data->gyro._z = packed_data[6];
        data->gyro.scale = 1.0 / GYRO_SCALE_MODIFIER_250DEG;
        

        std::cout<<
        "Accel x: " << data->accel.x() <<
        ", Accel y: " << data->accel.y() <<
        ", Accel z: " << data->accel.z() <<
        ", Gyro x: " << data->accel.x() <<
        ", Gyro y: " << data->gyro.y() <<
        ", Gryo z: " << data->gyro.z() <<
        ", temp: " << data->temp() << std::endl;
    }

}