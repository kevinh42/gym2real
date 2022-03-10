import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.data_callback,
            10
        )
        self.subscription
        self.accel = {'x': [], 'y': [], 'z': []}
        self.gyro = {'x': [], 'y': [], 'z': []}
        print("Initialized")

    def data_callback(self, msg):
        self.get_logger().info("Received imu data")
        self.gyro['x'].append(msg.angular_velocity.x)
        self.gyro['y'].append(msg.angular_velocity.y)
        self.gyro['z'].append(msg.angular_velocity.z)
        
        self.accel['x'].append(msg.linear_acceleration.x)
        self.accel['y'].append(msg.linear_acceleration.y)
        self.accel['z'].append(msg.linear_acceleration.z)

def main(args=None):
    rclpy.init(args=args)

    subscriber = ImuSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass

    plt.plot(subscriber.gyro['x'], label="gyro_x")
    plt.plot(subscriber.gyro['y'], label="gyro_y")
    plt.plot(subscriber.gyro['z'], label="gyro_z")
    plt.savefig('/root/code/gym2real/ros2/experiments/images/imu.png')

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
