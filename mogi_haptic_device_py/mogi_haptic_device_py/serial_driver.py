#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from builtin_interfaces.msg import Time
import serial
import math

class SerialDriverNode(Node):
    def __init__(self):
        super().__init__("serial_haptic_driver")
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.speed_publisher_ = self.create_publisher(Int8, 'joint_speed', 10)
        self.r_timer = self.create_timer(0.01, self.reader_callback)
        self.w_timer = self.create_timer(0.2, self.writer_callback)
        self.p_timer = self.create_timer(0.05, self.publisher_callback)

        self.serialPort = '/dev/ttyACM0'

        self.serial = serial.Serial(
            port=self.serialPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.joint_names = [
            'base_to_L1',
            'L1_to_L2',
            'L2_to_L3',
            'L3_to_L4',
            'L4_to_L5_1_A',
            'L4_to_L5_1_B',
            'L4_to_L5_2_A',
            'L4_to_L5_2_B',
            'L5_1_A_to_L5_3_A',
            'L5_1_B_to_L5_3_B'
        ]

        self.status = 1 # 1: not active, 2: active
        self.joint_angles = [0.0,0.0,0.0,0.0,0.0]
        self.speed = 100
        self.buttons = [0,0,0,0]
        self.gripper_gain = 1.85

        # TODO: These offsets should come from a calibration file
        self.joint0_offset = 0.0
        self.joint1_offset = 0.0
        self.joint2_offset = 3.0
        self.joint3_offset = 0.0

        self.serial.close()
        self.serial.open()
        self.serial.isOpen()

        self.get_logger().info("Serial driver has been started.")

    def reader_callback(self):
        out = []
        out.append(self.serial.read_until())

        for line in out:
            line = line.decode('utf-8').rstrip()
            if line != "":
                #self.get_logger().info(line)
                parts = line.split(';')
                if parts[0] == "ANG":
                    self.joint_angles[0] = float(parts[1])/100.0 + self.joint0_offset
                    self.joint_angles[1] = float(parts[2])/100.0 + self.joint1_offset
                    self.joint_angles[2] = float(parts[3])/100.0 + self.joint2_offset
                    self.joint_angles[3] = float(parts[4])/100.0 + self.joint3_offset
                    self.joint_angles[4] = float(parts[5])/100.0*self.gripper_gain
                else:
                    self.get_logger().warning("Unknown angle message received: " + line)

                if parts[6] == "SPD":
                    self.speed = int(parts[7])
                else:
                    self.get_logger().warning("Unknown speed message received: " + line)

                if parts[8] == "BTN":
                    self.buttons[0] = int(parts[9])
                    self.buttons[1] = int(parts[10])
                    self.buttons[2] = int(parts[11])
                    self.buttons[3] = int(parts[12])
                else:
                    self.get_logger().warning("Unknown button message received: " + line)

                if parts[13] == "END":
                    pass
                else:
                    self.get_logger().warning("Unknown end message received: " + line)



    def writer_callback(self):
        packet = bytearray("ST" + str(self.status) + "\n", "utf-8")
        self.serial.write(packet)

    def publisher_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        joint_angles_rad = [math.radians(angle) for angle in self.joint_angles]

        msg.position = [joint_angles_rad[0], joint_angles_rad[1], joint_angles_rad[2], joint_angles_rad[3], joint_angles_rad[4], -joint_angles_rad[4], joint_angles_rad[4], -joint_angles_rad[4], -joint_angles_rad[4], joint_angles_rad[4]]  # Duplicate and invert as needed

        # You can add velocities/efforts if needed
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)

        msg = Int8()
        msg.data = self.speed
        self.speed_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode() # node is now a custom class based on ROS2 Node
    rclpy.spin(node)         # Keeps the node running until it's closed with ctrl+c
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()