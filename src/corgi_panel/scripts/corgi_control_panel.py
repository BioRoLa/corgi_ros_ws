#!/usr/bin/env python3
import os
import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from corgi_msgs.msg import *
import subprocess
import signal
import numpy as np

GPIO_defined = True
try: import Jetson.GPIO as GPIO 
except: GPIO_defined = False


class CorgiControlPanel(QWidget):
    def __init__(self):
        super(CorgiControlPanel, self).__init__()
        
        if GPIO_defined:
            self.trigger_pin = 16
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.output(self.trigger_pin, GPIO.LOW)
        
        self.init_ui()
        self.init_ros()
        self.reset()


    def init_ui(self):
        ### ROS Bridge Button ###
        self.btn_ros_bridge = QPushButton('Run Bridge', self)
        btn_bridge_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                              QPushButton:checked {background-color: lavender; color: black;}
                              QPushButton:hover:!checked {background-color: silver;}
                              QPushButton:hover:pressed {background-color: gray; border-style: inset;}
                              QPushButton:disabled {background-color: lightgray; color: gray;}'''
        
        self.btn_ros_bridge.setStyleSheet(btn_bridge_style)
        self.btn_ros_bridge.setCheckable(True)
        self.btn_ros_bridge.clicked.connect(self.ros_bridge_cmd)
        
        
        ### Digital Button ###
        self.label_digital = QLabel('Digital:', self)
        self.label_digital.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_digital = QButtonGroup(self)
        self.btn_group_digital.setExclusive(True)
        
        self.btn_digital_on = QPushButton('ON', self)
        self.btn_digital_off = QPushButton('OFF', self)
        
        btn_digital_list = [self.btn_digital_on, self.btn_digital_off]
        btn_digital_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                               QPushButton:checked {background-color: pink;}
                               QPushButton:hover:!checked {background-color: silver;}
                               QPushButton:hover:pressed {background-color: lightcoral; border-style: inset;}
                               QPushButton:disabled {background-color: lightgray; color: gray;}'''

        for btn in btn_digital_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_digital_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_digital.addButton(btn)
            
        self.btn_digital_off.setChecked(True)
        
        
        ### Signal Button ###
        self.label_signal = QLabel('Signal:', self)
        self.label_signal.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_signal = QButtonGroup(self)
        self.btn_group_signal.setExclusive(True)
        
        self.btn_signal_on = QPushButton('ON', self)
        self.btn_signal_off = QPushButton('OFF', self)
        
        btn_signal_list = [self.btn_signal_on, self.btn_signal_off]
        btn_signal_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                              QPushButton:checked {background-color: pink;}
                              QPushButton:hover:!checked {background-color: silver;}
                              QPushButton:hover:pressed {background-color: lightcoral; border-style: inset;}
                              QPushButton:disabled {background-color: lightgray; color: gray;}'''

        for btn in btn_signal_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_signal_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_signal.addButton(btn)
            
        self.btn_signal_off.setChecked(True)
        
        
        ### Power Button ###
        self.label_power = QLabel('Power:', self)
        self.label_power.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_power = QButtonGroup(self)
        self.btn_group_power.setExclusive(True)
        
        self.btn_power_on = QPushButton('ON', self)
        self.btn_power_off = QPushButton('OFF', self)
        
        btn_power_list = [self.btn_power_on, self.btn_power_off]
        btn_power_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                             QPushButton:checked {background-color: pink;}
                             QPushButton:hover:!checked {background-color: silver;}
                             QPushButton:hover:pressed {background-color: lightcoral; border-style: inset;}
                             QPushButton:disabled {background-color: lightgray; color: gray;}'''

        for btn in btn_power_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_power_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_power.addButton(btn)
            
        self.btn_power_off.setChecked(True)
        

        ### Mode Switch Button ###
        self.label_mode = QLabel('Robot Mode Switch:')
        self.label_mode.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_mode = QButtonGroup(self)
        self.btn_group_mode.setExclusive(True)
        
        self.btn_rest_mode  = QPushButton('Rest Mode', self)
        self.btn_config     = QPushButton('Config Mode', self)
        self.btn_config.setVisible(False)
        self.btn_set_zero   = QPushButton('Set Zero', self)
        self.btn_hall_cal   = QPushButton('Hall Calibrate', self)
        self.btn_motor_mode = QPushButton('Motor Mode', self)
        
        btn_mode_list = [self.btn_rest_mode, self.btn_config, self.btn_set_zero, self.btn_hall_cal, self.btn_motor_mode]
        btn_mode_style = '''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                            QPushButton:checked {background-color: palegreen; color: black;}
                            QPushButton:hover:!checked {background-color: silver; color: black;}
                            QPushButton:hover:pressed {background-color: mediumseagreen; border-style: inset;}
                            QPushButton:disabled {background-color: lightgray; color: gray;}'''
        
        for id, btn in enumerate(btn_mode_list):
            btn.setCheckable(True)
            btn.setStyleSheet(btn_mode_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_mode.addButton(btn, id)
        
        self.btn_rest_mode.setChecked(True)
        
        self.btn_steer_cal  = QPushButton('Steer Calibrate', self)
        self.btn_steer_cal.setCheckable(True)
        self.btn_steer_cal.setStyleSheet(btn_mode_style)
        self.btn_steer_cal.clicked.connect(self.publish_power_cmd)
        
        
        ### Motor Mode Selection ###
        self.btn_group_motor = QButtonGroup(self)
        self.btn_group_motor.setExclusive(True)
        
        self.btn_rt_mode    = QPushButton('RT', self)
        self.btn_csv_mode   = QPushButton('CSV', self)

        btn_motor_list = [self.btn_rt_mode, self.btn_csv_mode]
        btn_mode_style = '''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                            QPushButton:checked {background-color: darkgreen; color: white;}
                            QPushButton:hover:!checked {background-color: silver; color: black;}
                            QPushButton:hover:pressed {background-color: mediumseagreen; border-style: inset;}
                            QPushButton:disabled {background-color: lightgray; color: gray;}'''

        for btn in btn_motor_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_mode_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_motor.addButton(btn)
            
        self.btn_rt_mode.setChecked(True)
        
            
        ### Data Output File Name ###
        self.label_output = QLabel('Output File Name (.csv):', self)
        self.label_output.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_output = QLineEdit('', self)
        self.edit_output.setStyleSheet('''background-color: white;''')
        
        
        ### CSV Control Button ###
        self.label_csv = QLabel('Input File Name (.csv):', self)
        self.label_csv.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_csv = QLineEdit('', self)
        self.edit_csv.setStyleSheet('''background-color: white;''')
        
        self.btn_csv_select = QPushButton('Select', self)
        self.btn_csv_select.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                             QPushButton:hover:!checked {background-color: silver; color: black;}
                                             QPushButton:hover:pressed {background-color: gray; border-style: inset;}
                                             QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_csv_select.clicked.connect(self.select_csv_file)
        
        self.btn_csv_run = QPushButton('Run', self)
        self.btn_csv_run.setCheckable(True)
        self.btn_csv_run.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                          QPushButton:checked {background-color: lightgray; color: black;}
                                          QPushButton:hover:!checked {background-color: lightgray; color: black;}
                                          QPushButton:hover:checked {background-color: silver; color: black;}
                                          QPushButton:hover:pressed {background-color: silver; border-style: inset;}
                                          QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_csv_run.clicked.connect(self.csv_control_cmd)
        
        
        ### Trigger Button ###
        self.btn_trigger = QPushButton('Trigger', self)
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                          QPushButton:checked {background-color: skyblue; color: black;}
                                          QPushButton:hover:!checked {background-color: silver; color: black;}
                                          QPushButton:hover:checked {background-color: cornflowerblue; color: black;}
                                          QPushButton:hover:pressed {background-color: royalblue; border-style: inset;}
                                          QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_trigger.clicked.connect(self.publish_trigger_cmd)
        
        
        ### Reset Button ###
        self.btn_reset = QPushButton('Reset', self)
        self.btn_reset.setStyleSheet('''QPushButton {background-color: plum; color: black; text-align: center; border-radius: 5px;}
                                        QPushButton:hover:!checked {background-color: violet; color: black;}
                                        QPushButton:hover:pressed {background-color: darkviolet; border-style: inset;}
                                        QPushButton:disabled {background-color: lightgray; color: gray;}''')
        self.btn_reset.clicked.connect(self.reset)
        
        
        ### Robot Status ###
        self.label_status = QLabel('Status:')
        self.label_status.setStyleSheet('color: white; font-weight: bold;')
        
        power_status_headers = ['Digital', 'Signal', 'Power', 'Mode']
        motor_status_headers = ['LF Motor R', 'LF Motor L', 'RF Motor R', 'RF Motor L',
                                'RH Motor R', 'RH Motor L', 'LH Motor R', 'LH Motor L',]
        
        self.power_status_values = ['OFF', 'OFF', 'OFF', '-']
        self.motor_status_values = ['-', '-', '-', '-', '-', '-', '-', '-']
        
        self.label_power_status_headers = [QLabel(header) for header in power_status_headers]
        self.label_motor_status_headers = [QLabel(header) for header in motor_status_headers]
        self.label_power_status_values = [QLabel(value) for value in self.power_status_values]
        self.label_motor_status_values = [QLabel(value) for value in self.motor_status_values]
        
        label_status_headers = self.label_power_status_headers + self.label_motor_status_headers
        label_status_values = self.label_power_status_values + self.label_motor_status_values
        
        [label.setStyleSheet('color: antiquewhite;') for label in label_status_headers]
        [label.setStyleSheet('color: gold;') for label in label_status_values]
        [label.setFixedWidth(300) for label in label_status_values]
        [label.setAlignment(Qt.AlignCenter) for label in label_status_values]
        
        
        ### Vertical Line ###
        vlines = [QWidget() for i in range(5)]
        for vline in vlines:
            vline.setFixedWidth(5)
            vline.setStyleSheet('background-color: black;')
        
        
        ### Horizontal Line ###
        hlines = [QWidget() for i in range(5)]
        for hline in hlines:
            hline.setFixedHeight(5)
            hline.setStyleSheet('background-color: black;')
        
        
        ### Arrange Elements ###
        layout = QGridLayout()
        layout.setSpacing(20)
        layout.addWidget(self.btn_ros_bridge,  0, 0, 1, 2)
        layout.addWidget(self.label_digital,   1, 0, 1, 2)
        layout.addWidget(self.btn_digital_on,  2, 0, 1, 1)
        layout.addWidget(self.btn_digital_off, 2, 1, 1, 1)
        layout.addWidget(self.label_signal,    3, 0, 1, 2)
        layout.addWidget(self.btn_signal_on,   4, 0, 1, 1)
        layout.addWidget(self.btn_signal_off,  4, 1, 1, 1)
        layout.addWidget(self.label_power,     5, 0, 1, 2)
        layout.addWidget(self.btn_power_on,    6, 0, 1, 1)
        layout.addWidget(self.btn_power_off,   6, 1, 1, 1)
        layout.addWidget(self.label_mode,      7, 0, 1, 2)
        layout.addWidget(self.btn_rest_mode,   8, 0, 1, 2)
        layout.addWidget(self.btn_set_zero,    9, 0, 1, 2)
        layout.addWidget(self.btn_hall_cal,    10,0, 1, 2)
        layout.addWidget(self.btn_steer_cal,   11,0, 1, 2)
        layout.addWidget(self.btn_motor_mode,  12,0, 1, 2)
        layout.addWidget(self.btn_rt_mode,     13,0, 1, 1)
        layout.addWidget(self.btn_csv_mode,    13,1, 1, 1)
        layout.addWidget(vlines[0],            0, 2, 14,1)
        layout.addWidget(self.label_csv,       0, 3, 1, 2)
        layout.addWidget(self.edit_csv,        1, 3, 1, 1)
        layout.addWidget(self.btn_csv_select,  1, 4, 1, 1)
        layout.addWidget(self.btn_csv_run,     2, 3, 1, 2)
        layout.addWidget(self.label_output,    7, 3, 1, 2)
        layout.addWidget(self.edit_output,     8, 3, 1, 2)
        layout.addWidget(self.btn_trigger,     9, 3, 1, 2)
        layout.addWidget(self.btn_reset,       10,3, 1, 2)
        layout.addWidget(vlines[1],            0, 5, 14,1)
        layout.addWidget(self.label_status,    0, 6, 1, 2)
        for i in range(len(label_status_headers)):
            layout.addWidget(label_status_headers[i], i+1, 6, 1, 1)
        for i in range(len(label_status_values)):
            layout.addWidget(label_status_values[i],  i+1, 7, 1, 1)
        

        ### General Setting ###
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(100)
        
        self.setLayout(layout)
        self.setWindowTitle('Corgi Control Panel')
        self.setStyleSheet('''background-color: dimgray; font-family: Ubuntu; font-size: 36px; padding: 5px''')
        self.setWindowFlag(Qt.WindowStaysOnTopHint)
        
        x = (QApplication.desktop().screenGeometry().width() - self.width())
        y = (QApplication.desktop().screenGeometry().height() - self.height())
        self.move(x//3, y//3)
        
        self.show()
        

    def init_ros(self):
        rclpy.init()
        node = rclpy.create_node('corgi_control_panel')
        
        self.power_cmd_pub = node.create_publisher(PowerCmdStamped, queue_size=10, 'power/command')
        self.motor_cmd_pub = node.create_publisher(MotorCmdStamped, queue_size=10, 'motor/command')
        self.trigger_pub = node.create_publisher(TriggerStamped, queue_size=10, 'trigger')
        self.sensor_enable_pub = node.create_publisher(SensorEnableStamped, queue_size=10, 'sensor_enable')
        
        self.power_state_sub = node.create_subscription(PowerStateStamped, 'power/state', self.power_state_cb)
        self.motor_state_sub = node.create_subscription(MotorStateStamped, 'motor/state', self.motor_state_cb)
        self.steer_state_sub = node.create_subscription(SteeringStateStamped, 'steer/state', self.steer_state_cb)
        
        self.power_state = PowerStateStamped()
        self.motor_state = MotorCmdStamped()
        self.steer_state = SteeringStateStamped()
    
    
    def select_csv_file(self):
        file_dialog = QFileDialog(self)
        file_dialog.setWindowTitle('Open CSV File')
        file_dialog.setNameFilter('CSV Files (*.csv)')
        file_dialog.setDirectory(os.path.expanduser('~/corgi_ws/corgi_ros_ws/input_csv/'))
        file_dialog.setStyleSheet('''background-color: white; font-family: Ubuntu; font-size: 24px; padding: 3px''')

        if file_dialog.exec() == QFileDialog.Accepted:
            file_path = file_dialog.selectedFiles()[0]
            file_name = os.path.splitext(os.path.basename(file_path))[0]
            self.edit_csv.setText(file_name)
        else:
            self.edit_csv.setText('')

    
    def ros_bridge_cmd(self):        
        if self.btn_ros_bridge.isChecked():
            self.btn_ros_bridge.setText('Stop Bridge')
            self.process_bridge = subprocess.Popen(['rosrun', 'corgi_ros_bridge', 'corgi_ros_bridge.sh'])
        else:
            self.btn_ros_bridge.setText('Run Bridge')
            if hasattr(self, 'process_bridge'):
                self.process_bridge.send_signal(signal.SIGINT)
                self.process_bridge.wait()
        
        self.set_btn_enable()
        
    
    def csv_control_cmd(self):        
        if self.btn_csv_run.isChecked():
            self.btn_csv_run.setText('Stop')
            self.process_csv = subprocess.Popen(['rosrun', 'corgi_csv_control', 'corgi_csv_control', self.edit_csv.text()])

        else:
            self.btn_csv_run.setText('Run')
            if hasattr(self, 'process_csv'):
                self.process_csv.terminate()
                self.process_csv.wait()
        
        self.set_btn_enable()
        

    def publish_power_cmd(self):
        power_cmd = PowerCmdStamped()
        
        power_cmd.header.seq = 9999
        power_cmd.header.stamp = rospy.Time.now()
        power_cmd.digital = self.btn_digital_on.isChecked()
        power_cmd.signal = self.btn_signal_on.isChecked()
        power_cmd.power = self.btn_power_on.isChecked()
        power_cmd.clean = False
        power_cmd.robot_mode = self.btn_group_mode.checkedId()
        power_cmd.steering_cali = self.btn_steer_cal.isChecked()
        
        self.power_cmd_pub.publish(power_cmd)
        
        if self.sender() != self.btn_motor_mode: self.publish_motor_zero_cmd(kp=0, ki=0, kd=0)
        # elif self.power_state.robot_mode != 4: self.publish_motor_zero_cmd(kp=90, ki=0, kd=1.75)
        
        self.set_btn_enable()
        

    def publish_motor_zero_cmd(self, kp, ki, kd):
        motor_cmd = MotorCmdStamped()
        
        motor_cmd.header.seq = 9999
        motor_cmd.header.stamp = rospy.Time.now()
        
        for cmd in [motor_cmd.module_a, motor_cmd.module_b, motor_cmd.module_c, motor_cmd.module_d]:
            cmd.theta = np.deg2rad(17)
            cmd.beta = 0
            cmd.kp_r = kp
            cmd.kp_l = kp
            cmd.ki_r = ki
            cmd.ki_l = ki
            cmd.kd_r = kd
            cmd.kd_l = kd
            
        self.motor_cmd_pub.publish(motor_cmd)
        
        self.set_btn_enable()
        
    
    def publish_trigger_cmd(self):
        trigger_cmd = TriggerStamped()
        
        trigger_cmd.header.stamp = rospy.Time.now()
        trigger_cmd.enable = self.btn_trigger.isChecked()
        trigger_cmd.output_filename = self.edit_output.text()
        
        self.trigger_pub.publish(trigger_cmd)
        
        if GPIO_defined: GPIO.output(self.trigger_pin, GPIO.HIGH if self.btn_trigger.isChecked() else GPIO.LOW)
        
        self.set_btn_enable()
        
    
    def set_btn_enable(self):
        self.btn_ros_bridge.setEnabled(self.btn_digital_off.isChecked())
        self.btn_digital_on.setEnabled(self.btn_ros_bridge.isChecked())
        self.btn_digital_off.setEnabled(self.btn_signal_off.isChecked())
        self.btn_signal_on.setEnabled(self.btn_digital_on.isChecked())
        self.btn_signal_off.setEnabled(self.btn_power_off.isChecked())
        self.btn_power_on.setEnabled(self.btn_signal_on.isChecked())
        self.btn_power_off.setEnabled(self.btn_rest_mode.isChecked())
        self.btn_rest_mode.setEnabled(not self.btn_csv_run.isChecked())
        self.btn_set_zero.setEnabled(self.btn_digital_on.isChecked() and self.btn_power_on.isChecked() and not self.btn_motor_mode.isChecked())
        self.btn_hall_cal.setEnabled(self.power_state.robot_mode in [1, 2])
        self.btn_steer_cal.setEnabled(self.power_state.robot_mode == 3 and not self.steer_state.current_state)
        self.btn_motor_mode.setEnabled(self.power_state.robot_mode in [1, 3])
        self.btn_rt_mode.setEnabled(self.btn_motor_mode.isChecked() and not self.btn_csv_run.isChecked())
        self.btn_csv_mode.setEnabled(self.btn_motor_mode.isChecked())
        self.label_csv.setEnabled(self.btn_motor_mode.isChecked() and self.btn_csv_mode.isChecked())
        self.edit_csv.setEnabled(self.label_csv.isEnabled())
        self.btn_csv_select.setEnabled(self.label_csv.isEnabled())
        self.btn_csv_run.setEnabled(self.label_csv.isEnabled())
        
        
    def reset(self):
        self.btn_digital_off.setChecked(True)
        self.btn_signal_off.setChecked(True)
        self.btn_power_off.setChecked(True)
        self.btn_rest_mode.setChecked(True)
        self.btn_rt_mode.setChecked(True)
        self.btn_csv_run.setChecked(False)
        self.btn_trigger.setChecked(False)
        
        self.csv_control_cmd()
        self.publish_trigger_cmd()
        self.publish_power_cmd()
    
    
    def power_state_cb(self, state):
        if (self.power_state.digital, self.power_state.power, self.power_state.signal, self.power_state.robot_mode) != (state.digital, state.power, state.signal, state.robot_mode):
            self.power_state = state
            self.update_power_status()
        else:
            self.power_state = state
            
        
    def motor_state_cb(self, state):
        self.motor_state = state
        
        
    def steer_state_cb(self, state):
        self.steer_state = state
        self.update_steer_status()
    
    def update_power_status(self):
        self.power_status_values[0] = 'ON' if self.power_state.digital else 'OFF'
        self.power_status_values[1] = 'ON' if self.power_state.signal else 'OFF'
        self.power_status_values[2] = 'ON' if self.power_state.power else 'OFF'
        self.power_status_values[3] = ['Rest Mode', 'Set Zero', 'Hall Calibrate', 'Motor Mode', 'Config Mode'][self.power_state.robot_mode]
        
        for i in range(len(self.power_status_values)):
            self.label_power_status_values[i].setText(self.power_status_values[i])

        if self.btn_hall_cal.isChecked() and self.power_state.robot_mode == 3:
            self.btn_motor_mode.setEnabled(True)
            self.btn_motor_mode.setChecked(True)
        
        self.set_btn_enable()
        

    def update_motor_status(self):
        self.set_btn_enable()
        
    
    def update_steer_status(self):
        if self.steer_state.current_state:
            self.btn_steer_cal.setChecked(False)
        self.set_btn_enable()

    
    def timer_update(self):
        if hasattr(self, 'process_csv') and self.process_csv.poll() is not None:
            self.btn_csv_run.setChecked(False)
            self.csv_control_cmd()
        
    
    def closeEvent(self, event):
        self.power_state_sub.unregister()
        self.motor_state_sub.unregister()
        super(CorgiControlPanel, self).closeEvent(event)
        
        self.reset()
        self.btn_ros_bridge.setChecked(False)
        self.ros_bridge_cmd()
        
        try: subprocess.run(['rosnode', 'kill', 'corgi_data_recorder'], check=True)
        except subprocess.CalledProcessError as e: print(f'Failed to kill corgi_data_recorder: {e}')
        
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())
