#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from std_msgs.msg import Int8
from builtin_interfaces.msg import Time
from mogi_haptic_device_interfaces.msg import TelemanipulatorPosition
import serial
import math

class SerialDriverNode(Node):
    def __init__(self):
        super().__init__("serial_haptic_driver")

        # Publisher for arm joint control
        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        # Action client for GripperCommand
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        self.position_publisher = self.create_publisher(TelemanipulatorPosition, 'telemanipulator_position', 10)
        self.r_timer = self.create_timer(0.01, self.reader_callback)
        self.w_timer = self.create_timer(0.02, self.writer_callback)
        self.p_timer = self.create_timer(0.02, self.publisher_callback)
        self.g_timer = self.create_timer(0.02, self.gripper_callback)

        self.serialPort = '/dev/ttyACM0'

        self.serial = serial.Serial(
            port=self.serialPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.header = False
        self.out = []
        self.send_open_gripper = False
        self.send_close_gripper = False

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        self.status = 1 # 1: not active, 2: active
        self.joint_angles = [0.0,0.0,0.0,0.0]
        self.buttons = [0,0,0,0,0,0]

        self.serial.close()
        self.serial.open()
        self.serial.isOpen()

        self.get_logger().info("Serial driver has been started.")

    def reader_callback(self):

        self.out = []
        self.out.append(self.serial.read_until())

        for line in self.out:
            print(line, len(self.out))
            line = line.decode('utf-8').rstrip()
            try:
                if line != "":

                    if line == "OK;RDS":
                        continue

                    parts = line.split(';')

                    if len(parts) == 11:
                        self.joint_angles[0] = float(parts[1])/100.0*3.14/180.0
                        self.joint_angles[1] = float(parts[2])/100.0*3.14/180.0
                        self.joint_angles[2] = float(parts[3])/100.0*3.14/180.0
                        self.joint_angles[3] = float(parts[4])/100.0*3.14/180.0
                        self.buttons[0] = int(parts[5])
                        self.buttons[1] = int(parts[6])
                        self.buttons[2] = int(parts[7])
                        self.buttons[3] = int(parts[8])
                        self.buttons[4] = int(parts[9])
                        self.buttons[5] = int(parts[10])

                        if self.buttons[3] == 1:
                            self.status = 1
                        elif self.buttons[2] == 1:
                            self.status = 2

                        if self.buttons[0] == 1:
                            self.send_open_gripper = True
                            self.send_close_gripper = False
                        elif self.buttons[1] == 1:
                            self.send_open_gripper = False
                            self.send_close_gripper = True
                        else:
                            self.send_open_gripper = False
                            self.send_close_gripper = False

                    else:
                        self.get_logger().warning("Unknown message received: " + line)

            except Exception as e:
                self.get_logger().error("Error parsing message: " + str(e) + " Line: " + line)



    def writer_callback(self):
        packet = bytearray("RDS\r\n", "utf-8")
        self.serial.write(packet)

    def publisher_callback(self):
        if self.status == 2:
            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = self.joint_angles
            point.time_from_start.sec = 0
            msg.points.append(point)
            self.arm_publisher.publish(msg)

    def gripper_callback(self):
        if self.send_close_gripper or self.send_open_gripper:
            goal_msg = GripperCommand.Goal()
            if self.send_close_gripper:
                goal_msg.command.position = 0.019
            else:
                goal_msg.command.position = -0.008
            goal_msg.command.max_effort = 10.0

            self.gripper_client.wait_for_server()
            send_goal_future = self.gripper_client.send_goal_async(goal_msg)
            #rclpy.spin_until_future_complete(self, send_goal_future)

            


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriverNode() # node is now a custom class based on ROS2 Node
    rclpy.spin(node)         # Keeps the node running until it's closed with ctrl+c
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()