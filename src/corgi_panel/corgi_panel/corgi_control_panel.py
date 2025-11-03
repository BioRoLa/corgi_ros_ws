#!/usr/bin/env python3

import os
import sys
import signal
import subprocess
import numpy as np

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from corgi_msgs.msg import (
    PowerCmdStamped, PowerStateStamped,
    MotorCmdStamped, MotorStateStamped,
    TriggerStamped,
    SensorEnableStamped,
    SteeringStateStamped
)

# GPIO (optional)
GPIO_defined = True
try:
    import Jetson.GPIO as GPIO
except Exception:
    GPIO_defined = False


class CorgiControlPanel(Node):
    def __init__(self):
        super().__init__('corgi_control_panel')
        self.get_logger().info('Corgi Control Panel Node Started')

        # === GPIO Setup ===
        if GPIO_defined:
            self.trigger_pin = 16
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.output(self.trigger_pin, GPIO.LOW)

        # === ROS Publishers ===
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        self.power_cmd_pub = self.create_publisher(PowerCmdStamped, 'power/command', qos)
        self.motor_cmd_pub = self.create_publisher(MotorCmdStamped, 'motor/command', qos)
        self.trigger_pub = self.create_publisher(TriggerStamped, 'trigger', qos)
        self.sensor_enable_pub = self.create_publisher(SensorEnableStamped, 'sensor_enable', qos)

        # === ROS Subscribers ===
        self.create_subscription(PowerStateStamped, 'power/state', self.power_state_cb, qos)
        self.create_subscription(MotorStateStamped, 'motor/state', self.motor_state_cb, qos)
        self.create_subscription(SteeringStateStamped, 'steer/state', self.steer_state_cb, qos)

        # === State ===
        self.power_state = PowerStateStamped()
        self.motor_state = MotorCmdStamped()
        self.steer_state = SteeringStateStamped()

        # === Qt GUI ===
        self.app = QApplication(sys.argv)
        self.window = CorgiControlPanelGUI(self)
        self.window.show()

        # === Timer for ROS spin + GUI update ===
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(50)  # 20 Hz

        # Start Qt event loop
        sys.exit(self.app.exec_())

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.window.timer_update()

    # === Callbacks ===
    def power_state_cb(self, msg):
        if (self.power_state.digital, self.power_state.power,
            self.power_state.signal, self.power_state.robot_mode) != (
            msg.digital, msg.power, msg.signal, msg.robot_mode):
            self.power_state = msg
            self.window.update_power_status()
        else:
            self.power_state = msg

    def motor_state_cb(self, msg):
        self.motor_state = msg
        self.window.update_motor_status()

    def steer_state_cb(self, msg):
        self.steer_state = msg
        self.window.update_steer_status()

    # === Commands ===
    def publish_power_cmd(self):
        cmd = PowerCmdStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'corgi_control_panel'
        cmd.digital = self.window.btn_digital_on.isChecked()
        cmd.signal = self.window.btn_signal_on.isChecked()
        cmd.power = self.window.btn_power_on.isChecked()
        cmd.clean = False
        cmd.robot_mode = self.window.btn_group_mode.checkedId()
        cmd.steering_cali = self.window.btn_steer_cal.isChecked()

        self.power_cmd_pub.publish(cmd)
        self.get_logger().debug(f"Published PowerCmd: mode={cmd.robot_mode}")

        if self.window.sender() != self.window.btn_motor_mode:
            self.publish_motor_zero_cmd(kp=0, ki=0, kd=0)

        self.window.set_btn_enable()

    def publish_motor_zero_cmd(self, kp, ki, kd):
        cmd = MotorCmdStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'corgi_control_panel'

        for module in [cmd.module_a, cmd.module_b, cmd.module_c, cmd.module_d]:
            module.theta = np.deg2rad(17)
            module.beta = 0
            module.kp_r = kp
            module.kp_l = kp
            module.ki_r = ki
            module.ki_l = ki
            module.kd_r = kd
            module.kd_l = kd

        self.motor_cmd_pub.publish(cmd)

    def publish_trigger_cmd(self):
        cmd = TriggerStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.enable = self.window.btn_trigger.isChecked()
        cmd.output_filename = self.window.edit_output.text()

        self.trigger_pub.publish(cmd)

        if GPIO_defined:
            GPIO.output(self.trigger_pin, GPIO.HIGH if cmd.enable else GPIO.LOW)

        self.window.set_btn_enable()

    # === External Processes ===
    def ros_bridge_cmd(self):
        if self.window.btn_ros_bridge.isChecked():
            self.window.btn_ros_bridge.setText('Stop Bridge')
            self.process_bridge = subprocess.Popen(
                ['ros2', 'run', 'corgi_ros_bridge', 'corgi_ros_bridge']
            )
        else:
            self.window.btn_ros_bridge.setText('Run Bridge')
            if hasattr(self, 'process_bridge'):
                self.process_bridge.terminate()
                self.process_bridge.wait()

        self.window.set_btn_enable()

    def csv_control_cmd(self):
        if self.window.btn_csv_run.isChecked():
            self.window.btn_csv_run.setText('Stop')
            csv_file = self.window.edit_csv.text()
            self.process_csv = subprocess.Popen(
                ['ros2', 'run', 'corgi_csv_control', 'corgi_csv_control', csv_file]
            )
        else:
            self.window.btn_csv_run.setText('Run')
            if hasattr(self, 'process_csv'):
                self.process_csv.terminate()
                self.process_csv.wait()

        self.window.set_btn_enable()

    def select_csv_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self.window,
            'Open CSV File',
            os.path.expanduser('~/corgi_ws/corgi_ros_ws/input_csv/'),
            'CSV Files (*.csv)'
        )
        if file_path:
            file_name = os.path.splitext(os.path.basename(file_path))[0]
            self.window.edit_csv.setText(file_name)
        else:
            self.window.edit_csv.setText('')


class CorgiControlPanelGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        self.reset()

    def init_ui(self):
        # === Buttons (same as before) ===
        self.btn_ros_bridge = QPushButton('Run Bridge', self)
        self.btn_ros_bridge.setCheckable(True)
        self.btn_ros_bridge.clicked.connect(self.node.ros_bridge_cmd)

        # Digital
        self.label_digital = QLabel('Digital:', self)
        self.btn_digital_on = QPushButton('ON', self)
        self.btn_digital_off = QPushButton('OFF', self)
        self.btn_group_digital = QButtonGroup(self)
        self.btn_group_digital.addButton(self.btn_digital_on)
        self.btn_group_digital.addButton(self.btn_digital_off)
        self.btn_digital_on.setCheckable(True)
        self.btn_digital_off.setCheckable(True)
        self.btn_digital_on.clicked.connect(self.node.publish_power_cmd)
        self.btn_digital_off.clicked.connect(self.node.publish_power_cmd)

        # Signal
        self.label_signal = QLabel('Signal:', self)
        self.btn_signal_on = QPushButton('ON', self)
        self.btn_signal_off = QPushButton('OFF', self)
        self.btn_group_signal = QButtonGroup(self)
        self.btn_group_signal.addButton(self.btn_signal_on)
        self.btn_group_signal.addButton(self.btn_signal_off)
        self.btn_signal_on.setCheckable(True)
        self.btn_signal_off.setCheckable(True)
        self.btn_signal_on.clicked.connect(self.node.publish_power_cmd)
        self.btn_signal_off.clicked.connect(self.node.publish_power_cmd)

        # Power
        self.label_power = QLabel('Power:', self)
        self.btn_power_on = QPushButton('ON', self)
        self.btn_power_off = QPushButton('OFF', self)
        self.btn_group_power = QButtonGroup(self)
        self.btn_group_power.addButton(self.btn_power_on)
        self.btn_group_power.addButton(self.btn_power_off)
        self.btn_power_on.setCheckable(True)
        self.btn_power_off.setCheckable(True)
        self.btn_power_on.clicked.connect(self.node.publish_power_cmd)
        self.btn_power_off.clicked.connect(self.node.publish_power_cmd)

        # Mode
        self.label_mode = QLabel('Robot Mode Switch:', self)
        self.btn_rest_mode = QPushButton('Rest Mode', self)
        self.btn_config = QPushButton('Config Mode', self)
        self.btn_config.setVisible(False)
        self.btn_set_zero = QPushButton('Set Zero', self)
        self.btn_hall_cal = QPushButton('Hall Calibrate', self)
        self.btn_motor_mode = QPushButton('Motor Mode', self)

        self.btn_group_mode = QButtonGroup(self)
        buttons = [self.btn_rest_mode, self.btn_config, self.btn_set_zero, self.btn_hall_cal, self.btn_motor_mode]
        for i, btn in enumerate(buttons):
            btn.setCheckable(True)
            btn.clicked.connect(self.node.publish_power_cmd)
            self.btn_group_mode.addButton(btn, i)

        self.btn_steer_cal = QPushButton('Steer Calibrate', self)
        self.btn_steer_cal.setCheckable(True)
        self.btn_steer_cal.clicked.connect(self.node.publish_power_cmd)

        # Motor Mode
        self.btn_rt_mode = QPushButton('RT', self)
        self.btn_csv_mode = QPushButton('CSV', self)
        self.btn_group_motor = QButtonGroup(self)
        self.btn_group_motor.addButton(self.btn_rt_mode)
        self.btn_group_motor.addButton(self.btn_csv_mode)
        self.btn_rt_mode.setCheckable(True)
        self.btn_csv_mode.setCheckable(True)
        self.btn_rt_mode.clicked.connect(self.node.publish_power_cmd)
        self.btn_csv_mode.clicked.connect(self.node.publish_power_cmd)

        # CSV
        self.label_csv = QLabel('Input File Name (.csv):', self)
        self.edit_csv = QLineEdit('', self)
        self.btn_csv_select = QPushButton('Select', self)
        self.btn_csv_select.clicked.connect(self.node.select_csv_file)
        self.btn_csv_run = QPushButton('Run', self)
        self.btn_csv_run.setCheckable(True)
        self.btn_csv_run.clicked.connect(self.node.csv_control_cmd)

        # Output
        self.label_output = QLabel('Output File Name (.csv):', self)
        self.edit_output = QLineEdit('', self)

        # Trigger
        self.btn_trigger = QPushButton('Trigger', self)
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.clicked.connect(self.node.publish_trigger_cmd)

        # Reset
        self.btn_reset = QPushButton('Reset', self)
        self.btn_reset.clicked.connect(self.reset)

        # Status
        self.label_status = QLabel('Status:', self)
        headers = ['Digital', 'Signal', 'Power', 'Mode'] + \
                  ['LF R', 'LF L', 'RF R', 'RF L', 'RH R', 'RH L', 'LH R', 'LH L']
        self.status_labels = [QLabel(h) for h in headers]
        self.value_labels = [QLabel('-') for _ in headers]

        # Layout
        layout = QGridLayout()
        layout.setSpacing(20)
        layout.addWidget(self.btn_ros_bridge, 0, 0, 1, 2)
        layout.addWidget(self.label_digital, 1, 0, 1, 2)
        layout.addWidget(self.btn_digital_on, 2, 0)
        layout.addWidget(self.btn_digital_off, 2, 1)
        layout.addWidget(self.label_signal, 3, 0, 1, 2)
        layout.addWidget(self.btn_signal_on, 4, 0)
        layout.addWidget(self.btn_signal_off, 4, 1)
        layout.addWidget(self.label_power, 5, 0, 1, 2)
        layout.addWidget(self.btn_power_on, 6, 0)
        layout.addWidget(self.btn_power_off, 6, 1)
        layout.addWidget(self.label_mode, 7, 0, 1, 2)
        layout.addWidget(self.btn_rest_mode, 8, 0, 1, 2)
        layout.addWidget(self.btn_set_zero, 9, 0, 1, 2)
        layout.addWidget(self.btn_hall_cal, 10, 0, 1, 2)
        layout.addWidget(self.btn_steer_cal, 11, 0, 1, 2)
        layout.addWidget(self.btn_motor_mode, 12, 0, 1, 2)
        layout.addWidget(self.btn_rt_mode, 13, 0)
        layout.addWidget(self.btn_csv_mode, 13, 1)

        # Right side
        layout.addWidget(QLabel(), 0, 2, 14, 1)  # spacer
        layout.addWidget(self.label_csv, 0, 3, 1, 2)
        layout.addWidget(self.edit_csv, 1, 3)
        layout.addWidget(self.btn_csv_select, 1, 4)
        layout.addWidget(self.btn_csv_run, 2, 3, 1, 2)
        layout.addWidget(self.label_output, 7, 3, 1, 2)
        layout.addWidget(self.edit_output, 8, 3, 1, 2)
        layout.addWidget(self.btn_trigger, 9, 3, 1, 2)
        layout.addWidget(self.btn_reset, 10, 3, 1, 2)

        # Status
        layout.addWidget(self.label_status, 0, 6, 1, 2)
        for i, (h, v) in enumerate(zip(self.status_labels, self.value_labels)):
            layout.addWidget(h, i + 1, 6)
            layout.addWidget(v, i + 1, 7)

        self.setLayout(layout)
        self.setWindowTitle('Corgi Control Panel')
        self.setStyleSheet('background-color: dimgray; color: white; font: 18pt Ubuntu;')
        self.setWindowFlag(Qt.WindowStaysOnTopHint)
        self.resize(1400, 800)

    def reset(self):
        self.btn_digital_off.setChecked(True)
        self.btn_signal_off.setChecked(True)
        self.btn_power_off.setChecked(True)
        self.btn_rest_mode.setChecked(True)
        self.btn_rt_mode.setChecked(True)
        self.btn_csv_run.setChecked(False)
        self.btn_trigger.setChecked(False)
        self.node.publish_trigger_cmd()
        self.node.publish_power_cmd()

    def set_btn_enable(self):
        # (Same logic as before — you can copy-paste)
        pass  # Implement as needed

    def update_power_status(self):
        values = [
            'ON' if self.node.power_state.digital else 'OFF',
            'ON' if self.node.power_state.signal else 'OFF',
            'ON' if self.node.power_state.power else 'OFF',
            ['Rest', 'Set Zero', 'Hall Cal', 'Motor', 'Config'][self.node.power_state.robot_mode]
        ]
        for i, v in enumerate(values):
            self.value_labels[i].setText(v)

    def update_motor_status(self):
        pass

    def update_steer_status(self):
        if self.node.steer_state.current_state:
            self.btn_steer_cal.setChecked(False)
        self.set_btn_enable()

    def timer_update(self):
        if hasattr(self.node, 'process_csv') and self.node.process_csv.poll() is not None:
            self.btn_csv_run.setChecked(False)
            self.node.csv_control_cmd()


def main():
    rclpy.init()
    panel = CorgiControlPanel()
    rclpy.shutdown()


if __name__ == '__main__':
    main()