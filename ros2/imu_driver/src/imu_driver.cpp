#include <imu_driver/imu_driver.h>

#include <JetsonGPIO.h>
#include <imu_driver/imu.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

class Callback
{
public:
    typedef boost::function<void(const std::string&)> Handle;

    Callback(Handle fxn, const std::string& name)
        : fxn_(fxn)
        , name_(name)
    {}

    Callback(const Callback&) = default;

    void operator()(const std::string& channel)
    {
        fxn_(channel);
    }

    bool operator==(const Callback& other) const 
    {
        return name_ == other.name_;
    }

private:
    boost::function<void(const std::string&)> fxn_;
    std::string name_;
};

IMUDriver::IMUDriver(int interrupt_pin, int sda_pin, int scl_pin, int address)
    : Node("imu_driver")
    , imu_(new IMU(address))
    , data_(new DataIMU())
    , interrupt_pin_(interrupt_pin)
{
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    cb_ = new Callback(boost::bind(&IMUDriver::data_callback, this, _1), "data_callback");
    imu_->init(sda_pin, scl_pin);
    imu_->configure(INT_ENABLE, 0x01); // enable data interrupt
    imu_->configure(SMPLRT_DIV, 0x07); // set 1kHz sample rate on GYRO
    imu_->configure(INT_PIN_CFG, 0x00); // set interrupt to not clear until status is read
    imu_->setGyroRange(IMU::GyroRange::DEG250);
    imu_->setAccelRange(IMU::AccelRange::G2);

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(interrupt_pin_, GPIO::IN);
    GPIO::add_event_detect(interrupt_pin_, GPIO::RISING, *cb_);
    // read status to clear interrupt
    //imu_->getInterruptStatus();
}

IMUDriver::~IMUDriver()
{
    delete data_;
    GPIO::cleanup();
}

void IMUDriver::data_callback(const std::string& channel)
{
    imu_->getDataFast(data_);
    // read status to clear interrupt
    //imu_->getInterruptStatus();

    sensor_msgs::msg::Imu msg;
    msg.angular_velocity.x = data_->gyro.x();
    msg.angular_velocity.y = data_->gyro.y();
    msg.angular_velocity.z = data_->gyro.z();
    msg.linear_acceleration.x = data_->accel.x();
    msg.linear_acceleration.y = data_->accel.y();
    msg.linear_acceleration.z = data_->accel.z();

    publisher_->publish(msg);
}

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUDriver>(19, 3, 5, 0x68));
    rclcpp::shutdown();
    return 0;
}
