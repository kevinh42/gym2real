import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

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
        self.q = {'x': [], 'y': [], 'z': [], 'w': []}
        print("Initialized")

    def data_callback(self, msg):
        self.get_logger().info("Received imu data")
        self.gyro['x'].append(msg.angular_velocity.x)
        self.gyro['y'].append(msg.angular_velocity.y)
        self.gyro['z'].append(msg.angular_velocity.z)
        
        self.accel['x'].append(msg.linear_acceleration.x)
        self.accel['y'].append(msg.linear_acceleration.y)
        self.accel['z'].append(msg.linear_acceleration.z)

        self.q['x'].append(msg.orientation.x)
        self.q['y'].append(msg.orientation.y)
        self.q['z'].append(msg.orientation.z)
        self.q['w'].append(msg.orientation.w)

def main(args=None):
    rclpy.init(args=args)

    subscriber = ImuSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass

    plt.plot(subscriber.q['x'], label="quat x")
    plt.plot(subscriber.q['y'], label="quat y")
    plt.plot(subscriber.q['z'], label="quat z")
    plt.plot(subscriber.q['w'], label="quat w")
    plt.title("Quaternion Representation Through IMU Movement")
    plt.legend()
    plt.savefig('/root/code/gym2real/ros2/experiments/images/imu.png')

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
