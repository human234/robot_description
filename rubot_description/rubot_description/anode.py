import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

def smoothstep(t):
    # smooth interpolation: 3t^2 - 2t^3
    return t * t * (3.0 - 2.0 * t)

class JointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('joint_trajectory_node')

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(0.05, self.update)

        self.joint_names = [
            'second_segment_joint',
            'third_segment_joint',
            'forth_segment_joint',
            'gripper_joint',
            'right_gripper_joint',
            'left_gripper_joint'
        ]

        # 5 trajectory waypoints (rad)
        self.waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.5, 0.2, -0.3, 0.4, 0.0, 0.1],
            [1.0, -0.5, 0.2, -0.2, 0.3, 0.0],
            [0.3, 0.6, -0.4, 0.1, -0.2, 0.5],
            [-0.5, 0.0, 0.3, -0.6, 0.4, -0.3],
        ]

        self.segment = 0
        self.t = 0.0

        self.duration = 2.0  # seconds per segment

    def interpolate(self, a, b, t):
        s = smoothstep(t)
        return a + (b - a) * s

    def update(self):
        dt = 0.05
        self.t += dt / self.duration

        if self.t >= 1.0:
            self.t = 0.0
            self.segment += 1

            # loop back
            if self.segment >= len(self.waypoints):
                self.segment = 0

        start = np.array(self.waypoints[self.segment])
        end = np.array(self.waypoints[(self.segment + 1) % len(self.waypoints)])

        joints = self.interpolate(start, end, self.t)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joints.tolist()

        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = JointTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
