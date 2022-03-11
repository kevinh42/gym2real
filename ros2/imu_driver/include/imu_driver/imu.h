#ifndef IMU_H_
#define IMU_H_

#include <i2c/i2c.h>
#include <map>
#include <string>

// MPU-6050 registers
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

#define SELF_TEST_X  0x0D
#define SELF_TEST_Y  0x0E
#define SELF_TEST_Z  0x0F
#define SELF_TEST_A  0x10

#define ACCEL_XOUT0  0x3B
#define ACCEL_XOUT1  0x3C
#define ACCEL_YOUT0  0x3D
#define ACCEL_YOUT1  0x3E
#define ACCEL_ZOUT0  0x3F
#define ACCEL_ZOUT1  0x40

#define TEMP_OUT0    0x41
#define TEMP_OUT1    0x42

#define GYRO_XOUT0   0x43
#define GYRO_XOUT1   0x44
#define GYRO_YOUT0   0x45
#define GYRO_YOUT1   0x46
#define GYRO_ZOUT0   0x47
#define GYRO_ZOUT1   0x48

#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38
#define INT_STATUS   0x3A
#define INT_PIN_CFG  0x37

// Pre-defined ranges
#define ACCEL_RANGE_2G     0x00
#define ACCEL_RANGE_4G     0x08 
#define ACCEL_RANGE_8G     0x10
#define ACCEL_RANGE_16G    0x18

#define GYRO_RANGE_250DEG  0x00
#define GYRO_RANGE_500DEG  0x08
#define GYRO_RANGE_1000DEG 0x10
#define GYRO_RANGE_2000DEG 0x18

// Constants
#define GRAVITY_MS2                 9.80665

#define ACCEL_SCALE_MODIFIER_2G     16284.0
#define ACCEL_SCALE_MODIFIER_4G     8192.0
#define ACCEL_SCALE_MODIFIER_8G     4096.0
#define ACCEL_SCALE_MODIFIER_16G    2048.0

#define GYRO_SCALE_MODIFIER_250DEG  131.0
#define GYRO_SCALE_MODIFIER_500DEG  65.5
#define GYRO_SCALE_MODIFIER_1000DEG 32.8
#define GYRO_SCALE_MODIFIER_2000DEG 16.4

static const std::map<int, std::string> i2c_sda_map = {
    {27, "/dev/i2c-0"},
    {3, "/dev/i2c-1"}
};

static const std::map<int, std::string> i2c_scl_map = {
    {28, "/dev/i2c-0"},
    {5, "/dev/i2c-1"}
};

struct Data {
    int16_t _xyz[3];

    double scale;

    Data()
        : _xyz {0, 0, 0}, scale(1.0)
    {}

    Data(const int16_t& x, const int16_t& y, const int16_t& z)
        : _xyz {x, y, z}
    {}

    double x() { return (double) _xyz[0] * scale; }
    double y() { return (double) _xyz[1] * scale; }
    double z() { return (double) _xyz[2] * scale; }
};

struct DataIMU {
    int16_t _data[7];

    DataIMU()
        : _data {0, 0, 0, 0, 0, 0, 0}
    {}

    double temp() { return ((double) _data[3] / 340.0) + 36.53; }
    Data& accel()  
    { 
        _accel._xyz[0] = _data[0];
        _accel._xyz[1] = _data[1];
        _accel._xyz[2] = _data[2];

        return _accel;
    }

    Data& gyro()
    {
        _gyro._xyz[0] = _data[4];
        _gyro._xyz[1] = _data[5];
        _gyro._xyz[2] = _data[6];

        return _gyro;
    }

private:
    Data _accel;
    Data _gyro;
};

class IMU
{
public:
    enum GyroRange { 
        DEG250 = GYRO_RANGE_250DEG,
        DEG500 = GYRO_RANGE_500DEG,
        DEG1000 = GYRO_RANGE_1000DEG,
        DEG2000 = GYRO_RANGE_2000DEG
    };

    enum AccelRange {
        G2 = ACCEL_RANGE_2G,
        G4 = ACCEL_RANGE_4G,
        G8 = ACCEL_RANGE_8G,
        G16 = ACCEL_RANGE_16G
    };

    IMU(int address);

    ~IMU();
    
    /**
     * @brief Initialize the IMU
     * 
     * @param sda_pin The sda pin number
     * @param scl_pin The scl pin number
     */
    bool init(int sda_pin = 3, int scl_pin = 5);

    /**
     * @brief Configure a register on the IMU
     * 
     * @param address The register address
     * @param value The configured value
     */
    void configure(int address, int value);

    /**
     * @brief Retrieve data from the IMU
     * 
     * @param data DataIMU object
     */
    bool getData(DataIMU* data);
    /**
     * @brief Get latest data from the gyroscope
     * 
     * @param data Data object
     */
    void getGyroData(Data* data);
    /**
     * @brief Get latest data from the accelerometer
     * 
     * @param data Data object
     * @param g Return multiple of acceleration due to gravity
     */
    void getAccelData(Data* data, bool g = false);
    /**
     * @brief Get latest data from the thermometer
     * 
     * @param data double value
     */
    void getTempData(int* data);

    /**
     * @brief Set the range of the accelerometer
     * 
     * @param range Pre-defined range
     */
    void setAccelRange(AccelRange range);
    /**
     * @brief Set the range of the gyroscope
     * 
     * @param range Pre-defined range
     */
    void setGyroRange(GyroRange range);

    /**
     * @brief Get the range of the accelerometer
     * 
     * @param raw Return raw data
     */
    int getAccelRange(bool raw = false);
    /**
     * @brief Get the range of the gyroscope
     * 
     * @param raw Return raw data
     */
    int getGyroRange(bool raw = false);

private:
    /**
     * @brief Read one byte of data
     * 
     * @param data uint8_t pointer
     */
    bool readByte(int address, int8_t* data);

    /**
     * @brief Read two bytes of data
     * 
     * @param data uint16_t pointer
     */
    bool readWord(int address, int16_t* data);

    /**
     * @brief Read six bytes of data
     * 
     * @param data uint16_t pointer
     */
    bool readXYZ(int address, int16_t* data);

    /**
     * @brief Write to IMU register
     * 
     * @param address write address
     * @param value int value
     */
    bool writeByte(int address, int value);

    /**
     * @brief Check if the IMU has been properly initialized
     * 
     * @throw std::runtime_error If not initialized
     */
    void checkInitialized();

    /**
     * @brief Get the gyroscope scale modifier
     * 
     * @param range The range setting of the gyroscope
     */
    double getGyroScale(const int& range);

    /**
     * @brief Get the accelerometer scale modifier
     * 
     * @param range The range setting of the accelerometer
     */
    double getAccelScale(const int& range, bool g);

    bool initialized_;
    int address_;
    int bus_;

    I2CDevice device_;
};

#endif