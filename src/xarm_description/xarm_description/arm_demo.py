import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# UFACTORY xArm6 demo poses — joint5=-1.57 is the official "hold-up" pose
# All values in radians. Gripper: drive_joint 0=closed, 0.85=open
POSES = [
    # name            j1     j2    j3     j4    j5      j6    gripper
    ("hold_up",      [0.0,   0.0,  0.0,   0.0,  -1.57,  0.0,  0.0]),
    ("grip_open",    [0.0,   0.0,  0.0,   0.0,  -1.57,  0.0,  0.6]),
    ("grip_close",   [0.0,   0.0,  0.0,   0.0,  +1.57,  0.0,  0.0]),
    ("rotate_90",    [1.57,  0.0,  0.0,   0.0,  -1.57,  0.0,  0.0]),
    ("grip_open",    [1.57,  0.0,  0.0,   0.0,  -1.57,  0.0,  0.6]),
    ("grip_close",   [1.57,  0.0,  0.0,   0.0,  +1.57,  0.0,  0.0]),
    ("rotate_180",   [3.14,  0.0,  0.0,   0.0,  -1.57,  0.0,  0.0]),
    ("rotate_back",  [0.0,   0.0,  0.0,   0.0,  -1.57,  0.0,  0.0]),
]

# Active joints + mimic gripper joints (all mimic joints = drive_joint value)
JOINTS = [
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
    'drive_joint',
    'left_finger_joint', 'left_inner_knuckle_joint',
    'right_outer_knuckle_joint', 'right_finger_joint', 'right_inner_knuckle_joint',
]

STEPS = 60    # interpolation steps between poses
RATE_HZ = 30  # publish frequency
HOLD_SEC = 0.8


def smooth(t):
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


class ArmDemo(Node):
    def __init__(self):
        super().__init__('arm_demo')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.pose_idx = 0
        self.step = 0
        self.holding = False
        self.hold_count = 0
        self.hold_steps = int(HOLD_SEC * RATE_HZ)
        self.get_logger().info('Arm demo started — watch in Foxglove')

    def publish(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        drive = positions[6]
        # mimic joints: multiplier=1, offset=0 — all equal drive_joint
        msg.position = positions + [drive, drive, drive, drive, drive]
        self.pub.publish(msg)

    def tick(self):
        current = POSES[self.pose_idx][1]
        next_idx = (self.pose_idx + 1) % len(POSES)
        next_pose = POSES[next_idx][1]

        if self.holding:
            self.publish(current)
            self.hold_count += 1
            if self.hold_count >= self.hold_steps:
                self.holding = False
                self.hold_count = 0
                self.step = 0
                self.pose_idx = next_idx
                self.get_logger().info(f'Moving to: {POSES[self.pose_idx][0]}')
        else:
            t = smooth(self.step / STEPS)
            self.publish(lerp(current, next_pose, t))
            self.step += 1
            if self.step > STEPS:
                self.holding = True


def main():
    rclpy.init()
    node = ArmDemo()
    rclpy.spin(node)
    rclpy.shutdown()
