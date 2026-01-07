#!/usr/bin/env python3
import os
import sys
import threading
import subprocess
import signal
import numpy as np
from datetime import datetime
from enum import IntEnum

import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QFont, QColor, QPalette
from PyQt5.QtCore import pyqtSignal

from corgi_msgs.msg import (
    MotorCmdStamped, MotorStateStamped, PowerCmdStamped, PowerStateStamped,
    RobotCmdStamped, RobotStateStamped, TriggerStamped, LogStamped
)

GPIO_defined = True
try: import Jetson.GPIO as GPIO 
except: GPIO_defined = False

class ROBOTMODE(IntEnum):
    SYSTEM_ON = 0
    INIT = 1
    IDLE = 2
    STANDBY = 3
    MOTORCONFIG = 4

class LOGLEVEL(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4

STYLESHEET = """
QWidget {
    background-color: #2b2b2b;
    color: #ffffff;
    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
    font-size: 14px;
}
QGroupBox {
    border: 1px solid #555;
    border-radius: 5px;
    margin-top: 20px;
    font-weight: bold;
    color: #ccc;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 3px;
}
QPushButton {
    background-color: #404040;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 8px;
    min-height: 25px;
}
QPushButton:hover { background-color: #505050; }
QPushButton:pressed { background-color: #2d2d2d; }
QPushButton:checked { background-color: #3a6ea5; color: white; border: 1px solid #5a9ed5; }
QPushButton:disabled { background-color: #333; color: #777; border: 1px solid #444; }

QPushButton#EstopBtn { background-color: #d32f2f; font-weight: bold; font-size: 18px; border: none; }
QPushButton#EstopBtn:hover { background-color: #b71c1c; }
QPushButton#EstopBtn:pressed { background-color: #8e0000; }

QPushButton#SystemOnBtn:checked { background-color: #2e7d32; } /* Green for ON */
QPushButton#ConfigBtn:checked { background-color: #f9a825; color: black; } /* Yellow for Config */

QLabel#HeaderLabel { font-size: 18px; font-weight: bold; color: #eee; }
QLabel#StatusLabel { font-size: 24px; font-weight: bold; color: #4fc3f7; }
QLabel#MotorLabel { font-size: 12px; color: #aaa; }
QLineEdit { background-color: #202020; border: 1px solid #555; color: white; padding: 5px; border-radius: 3px; }
QTextEdit { background-color: #1e1e1e; border: 1px solid #444; color: #00e676; font-family: 'Consolas', monospace; }
QLabel#PowerBadge { background-color: #222; border: 1px solid #555; border-radius: 4px; padding: 4px 8px; color: #eee; }
"""

class CorgiControlPanel(QWidget):
    power_state_signal = pyqtSignal(object)
    robot_state_signal = pyqtSignal(object)
    motor_state_signal = pyqtSignal(object)
    log_state_signal = pyqtSignal(object)

    def __init__(self):
        super(CorgiControlPanel, self).__init__()
        
        if GPIO_defined:
            self.trigger_pin = 16
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.output(self.trigger_pin, GPIO.LOW)

        self.init_ui()
        self.init_ros()
        
        # Connect signals
        self.power_state_signal.connect(self._handle_power_state_update)
        self.robot_state_signal.connect(self._handle_robot_state_update)
        self.motor_state_signal.connect(self._handle_motor_state_update)
        self.log_state_signal.connect(self._handle_log_update)
        
        # Robot command sequencing and pending state tracking
        self._robot_cmd_seq = 0
        self._pending_robot_mode = None  # type: int | None
        self._last_confirmed_mode = None  # Track last confirmed mode for error recovery
        self.process_recorder = None  # Track data recorder process
        self.process_imu = None  # Track IMU process
        self.process_set_zero = None  # Track set_zero process

        self.reset()

    def init_ui(self):
        self.setStyleSheet(STYLESHEET)
        
        # 主佈局
        main_v_layout = QVBoxLayout()
        main_v_layout.setSpacing(10)
        main_v_layout.setContentsMargins(15, 15, 15, 15)
        
        # --- Top Bar ---
        top_bar = QHBoxLayout()

        # Power summary (top-left)
        power_box = QHBoxLayout()
        power_box.setSpacing(8)
        self.lbl_voltage = QLabel('--.- V')
        self.lbl_voltage.setObjectName('PowerBadge')
        self.lbl_soc = QLabel('-- %')
        self.lbl_soc.setObjectName('PowerBadge')
        self.lbl_current = QLabel('-.-- A')
        self.lbl_current.setObjectName('PowerBadge')
        self.lbl_power = QLabel('--.- W')
        self.lbl_power.setObjectName('PowerBadge')
        power_box.addWidget(self.lbl_voltage)
        power_box.addWidget(self.lbl_soc)
        power_box.addWidget(self.lbl_current)
        power_box.addWidget(self.lbl_power)
        top_bar.addLayout(power_box)

        self.btn_estop = QPushButton('EMERGENCY STOP')
        self.btn_estop.setObjectName("EstopBtn")
        self.btn_estop.setMinimumWidth(200)
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self.e_stop_cmd)
        
        top_bar.addStretch(1) # 讓 E-Stop 靠右
        top_bar.addWidget(self.btn_estop)
        main_v_layout.addLayout(top_bar)
        
        # --- Middle Section ---
        middle_layout = QHBoxLayout()
        sidebar = QVBoxLayout()
        sidebar.setSpacing(15)
        
        # 1. ROS Bridge
        self.btn_ros_bridge = QPushButton('Run ROS Bridge')
        self.btn_ros_bridge.setCheckable(True)
        self.btn_ros_bridge.clicked.connect(self.ros_bridge_cmd)
        sidebar.addWidget(self.btn_ros_bridge)
        
        # 2. IMU & Set Zero
        self.btn_imu = QPushButton('IMU')
        self.btn_imu.setCheckable(True)
        self.btn_imu.clicked.connect(self.imu_cmd)
        sidebar.addWidget(self.btn_imu)

        self.btn_set_zero = QPushButton('Set Zero')
        self.btn_set_zero.clicked.connect(self.set_zero_cmd)
        sidebar.addWidget(self.btn_set_zero)

        # 3. FSM Control + Current Mode (Merged)
        grp_fsm = QGroupBox("FSM Indicator")
        grp_fsm_layout = QVBoxLayout()
        
        # Current Mode Display
        mode_container = QFrame()
        mode_container.setStyleSheet("background-color: #222; border-radius: 5px; margin-bottom: 5px;")
        mode_h_layout = QHBoxLayout(mode_container)
        mode_h_layout.setContentsMargins(5, 5, 5, 5)
        
        lbl_mode_title = QLabel("Current mode:")
        lbl_mode_title.setStyleSheet("color: #888; font-size: 12px;")
        self.label_robot_mode_value = QLabel("---")
        self.label_robot_mode_value.setObjectName("StatusLabel")
        self.label_robot_mode_value.setAlignment(Qt.AlignCenter)
        self.label_robot_mode_value.setStyleSheet("color: #bdbdbd; font-weight: bold; font-size: 20px;")
        
        mode_h_layout.addWidget(lbl_mode_title)
        mode_h_layout.addWidget(self.label_robot_mode_value)
        grp_fsm_layout.addWidget(mode_container)

        # FSM Buttons
        self.btn_system = QPushButton('Set to REST')
        self.btn_system.setObjectName("SystemOnBtn")
        self.btn_system.setCheckable(True)
        self.btn_system.clicked.connect(self.system_cmd)
        
        self.btn_idle = QPushButton('Set to IDLE')
        self.btn_idle.clicked.connect(self.set_idle_mode)
        
        self.btn_standby = QPushButton('Set STANDBY')
        self.btn_standby.clicked.connect(self.set_standby_mode)
        
        self.btn_motorconfig = QPushButton('Set to CONFIG')
        self.btn_motorconfig.setObjectName("ConfigBtn")
        self.btn_motorconfig.clicked.connect(self.set_motorconfig_mode)
        
        grp_fsm_layout.addWidget(self.btn_system)
        grp_fsm_layout.addWidget(self.btn_idle)
        grp_fsm_layout.addWidget(self.btn_standby)
        grp_fsm_layout.addWidget(self.btn_motorconfig)
        grp_fsm.setLayout(grp_fsm_layout)
        sidebar.addWidget(grp_fsm)

        # 4. Recorder
        grp_rec = QGroupBox("Data Recorder")
        grp_rec_layout = QVBoxLayout()
        self.edit_output = QLineEdit()
        self.edit_output.setPlaceholderText("File Name (.csv)")
        self.edit_output.returnPressed.connect(self.start_recording_from_input)
        self.btn_trigger = QPushButton('Start Trigger')
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.clicked.connect(self.publish_trigger_cmd)
        
        grp_rec_layout.addWidget(self.edit_output)
        grp_rec_layout.addWidget(self.btn_trigger)
        grp_rec.setLayout(grp_rec_layout)
        sidebar.addWidget(grp_rec)
        
        sidebar.addStretch(1)
        
        # Right Monitor Area (Only Motor Grid Now)
        monitor_layout = QVBoxLayout()
        
        grid_motors = QGridLayout()
        self.leg_labels = {}
        self.motor_labels = {}
        legs = [('LF', 0, 0, ['L', 'R']), ('RF', 0, 1, ['L', 'R']), ('LH', 1, 0, ['L', 'R']), ('RH', 1, 1, ['L', 'R'])]
        
        for leg_name, r, c, motors in legs:
            leg_group = QGroupBox(leg_name)
            leg_layout = QVBoxLayout()
            for m_key in motors:
                lbl = QLabel(f"{m_key}: --")
                lbl.setObjectName("MotorLabel")
                leg_layout.addWidget(lbl)
                self.motor_labels[m_key] = lbl
            leg_group.setLayout(leg_layout)
            grid_motors.addWidget(leg_group, r, c)

        monitor_layout.addLayout(grid_motors)
        monitor_layout.addStretch(1)

        middle_layout.addLayout(sidebar, 1)
        middle_layout.addLayout(monitor_layout, 3)
        main_v_layout.addLayout(middle_layout)
        
        log_group = QGroupBox("System Log")
        log_layout_inner = QVBoxLayout()
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        self.text_log.setMaximumHeight(150)
        log_layout_inner.addWidget(self.text_log)
        log_group.setLayout(log_layout_inner)
        main_v_layout.addWidget(log_group)

        self.setLayout(main_v_layout)
        self.setWindowTitle('Corgi Control Panel')
        self.resize(1024, 768)
        
        # 初始化按鈕狀態：只有 ros_bridge 可用
        self.btn_estop.setEnabled(False)
        self.btn_imu.setEnabled(False)
        self.btn_set_zero.setEnabled(False)
        self.btn_system.setEnabled(False)
        self.btn_idle.setEnabled(False)
        self.btn_standby.setEnabled(False)
        self.btn_motorconfig.setEnabled(False)
        self.btn_trigger.setEnabled(False)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(100)
        self.show()

    def init_ros(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('corgi_control_panel')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._executor_thread.start()

        self.power_cmd_pub = self.node.create_publisher(PowerCmdStamped, 'power/command', 10)
        self.robot_cmd_pub = self.node.create_publisher(RobotCmdStamped, 'robot/command', 10)
        self.trigger_pub = self.node.create_publisher(TriggerStamped, 'trigger', 10)

        self.power_state_sub = self.node.create_subscription(PowerStateStamped, 'power/state', self.power_state_cb, 10)
        self.robot_state_sub = self.node.create_subscription(RobotStateStamped, 'robot/state', self.robot_state_cb, 10)
        self.motor_state_sub = self.node.create_subscription(MotorStateStamped, 'motor/state', self.motor_state_cb, 10)
        self.log_sub = self.node.create_subscription(LogStamped, 'log', self.log_cb, 10)

        self.power_state = PowerStateStamped()
        self.robot_state = RobotStateStamped()
        self.motor_state = MotorStateStamped()
        self.log_state = LogStamped()
        self.add_log('[SYSTEM] Control Panel Initialized', 'INFO')

    def ros_bridge_cmd(self):        
        if self.btn_ros_bridge.isChecked():
            self.btn_ros_bridge.setText('Stop ROS Bridge')
            try:
                if hasattr(self, 'process_bridge') and self.process_bridge is not None and self.process_bridge.poll() is None:
                    self.add_log('Bridge already running; skip start', 'WARN')
                    return
                # Launch the ROS2 bridge executable directly
                self.process_bridge = subprocess.Popen(['ros2', 'run', 'corgi_ros_bridge', 'corgi_ros_bridge'])
                self.add_log('ROS Bridge Started (ros2 run corgi_ros_bridge corgi_ros_bridge)', 'SYSTEM')
            except Exception as e:
                self.add_log(f'Failed to start ROS Bridge: {e}', 'ERROR')
        else:
            self.btn_ros_bridge.setText('Run ROS Bridge')
            if hasattr(self, 'process_bridge') and self.process_bridge is not None:
                self.btn_ros_bridge.setEnabled(False)  # avoid double clicks while stopping
                # Stop in background to avoid freezing UI
                def _stop():
                    try:
                        self.process_bridge.send_signal(signal.SIGINT)
                        ret = self.process_bridge.wait(timeout=2.0)
                        self.add_log(f'ROS Bridge Stopped (code {ret})', 'SYSTEM')
                    except subprocess.TimeoutExpired:
                        self.add_log('ROS Bridge did not exit, terminating...', 'WARN')
                        self.process_bridge.terminate()
                        try:
                            ret = self.process_bridge.wait(timeout=2.0)
                            self.add_log(f'ROS Bridge Terminated (code {ret})', 'SYSTEM')
                        except subprocess.TimeoutExpired:
                            self.add_log('ROS Bridge still alive, killing...', 'ERROR')
                            self.process_bridge.kill()
                            self.process_bridge.wait()
                    finally:
                        self.process_bridge = None
                        self.btn_ros_bridge.setEnabled(True)
                threading.Thread(target=_stop, daemon=True).start()
            else:
                self.add_log('ROS Bridge Stopped', 'SYSTEM')
        self.set_btn_enable()

    def imu_cmd(self):
        if self.btn_imu.isChecked():
            self.btn_imu.setText('Stop IMU')
            try:
                if hasattr(self, 'process_imu') and self.process_imu is not None and self.process_imu.poll() is None:
                    self.add_log('IMU already running; skip start', 'WARN')
                    return
                # Launch the IMU node using ros2 run
                self.process_imu = subprocess.Popen(['ros2', 'run', 'corgi_imu', 'imu_node'])
                self.add_log('IMU Started (ros2 run corgi_imu imu_node)', 'SYSTEM')
            except Exception as e:
                self.add_log(f'Failed to start IMU: {e}', 'ERROR')
        else:
            self.btn_imu.setText('IMU')
            if hasattr(self, 'process_imu') and self.process_imu is not None:
                self.btn_imu.setEnabled(False)  # avoid double clicks while stopping
                # Stop in background to avoid freezing UI
                def _stop():
                    try:
                        self.process_imu.send_signal(signal.SIGINT)
                        ret = self.process_imu.wait(timeout=2.0)
                        self.add_log(f'IMU Stopped (code {ret})', 'SYSTEM')
                    except subprocess.TimeoutExpired:
                        self.add_log('IMU did not exit, terminating...', 'WARN')
                        self.process_imu.terminate()
                        try:
                            ret = self.process_imu.wait(timeout=2.0)
                            self.add_log(f'IMU Terminated (code {ret})', 'SYSTEM')
                        except subprocess.TimeoutExpired:
                            self.add_log('IMU still alive, killing...', 'ERROR')
                            self.process_imu.kill()
                            self.process_imu.wait()
                    finally:
                        self.process_imu = None
                        self.btn_imu.setEnabled(True)
                threading.Thread(target=_stop, daemon=True).start()
            else:
                self.add_log('IMU Stopped', 'SYSTEM')

    def set_zero_cmd(self):
        """Run set_zero command to reset motor reference zero points"""
        # Disable button immediately to prevent double clicks
        self.btn_set_zero.setEnabled(False)
        self.btn_set_zero.setText('Setting Zero...')
        
        try:
            # Launch set_zero as a background process
            self.process_set_zero = subprocess.Popen(['ros2', 'run', 'corgi_set_zero', 'set_zero'])
            self.add_log('Set Zero Started (ros2 run corgi_set_zero set_zero)', 'SYSTEM')
        except Exception as e:
            self.add_log(f'Failed to start set_zero: {e}', 'ERROR')
            self.btn_set_zero.setEnabled(True)
            self.btn_set_zero.setText('Set Zero')

    def e_stop_cmd(self):
        self.add_log('EMERGENCY STOP ACTIVATED!', 'FATAL')
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(ROBOTMODE.IDLE)  # [2] 強制回 IDLE
        self.robot_cmd_pub.publish(robot_cmd)
        self._robot_cmd_seq += 1
        self._pending_robot_mode = int(ROBOTMODE.IDLE)
        self.set_btn_enable()
    
    # 使用 Enum 發送命令
    def system_cmd(self): self._pub_robot_mode(ROBOTMODE.SYSTEM_ON)
    def set_idle_mode(self): self._pub_robot_mode(ROBOTMODE.IDLE)
    def set_standby_mode(self): self._pub_robot_mode(ROBOTMODE.STANDBY)
    def set_motorconfig_mode(self): self._pub_robot_mode(ROBOTMODE.MOTORCONFIG)
    
    def _pub_robot_mode(self, mode):
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(mode)
        self.robot_cmd_pub.publish(robot_cmd)
        self._robot_cmd_seq += 1
        self._pending_robot_mode = int(mode)
        self.add_log(f'Sent Robot Mode Command: {mode.name} ({mode.value}), seq={self._robot_cmd_seq}', 'SYSTEM')
        # Disable buttons until state matches the requested mode
        self.set_btn_enable()

    def start_recording_from_input(self):
        """Start recording when user presses Enter in the filename input field"""
        filename = self.edit_output.text().strip()
        if filename and not self.btn_trigger.isChecked():
            self.btn_trigger.setChecked(True)
            self.publish_trigger_cmd()
        elif not filename:
            self.add_log('Please enter a filename before recording', 'WARN')
        else:
            self.add_log('Recording already in progress', 'WARN')

    def publish_trigger_cmd(self):
        trigger_cmd = TriggerStamped()
        
        trigger_cmd.header.stamp = self.node.get_clock().now().to_msg()
        trigger_cmd.enable = self.btn_trigger.isChecked()
        trigger_cmd.output_filename = self.edit_output.text().strip()
        
        self.trigger_pub.publish(trigger_cmd)
        
        if GPIO_defined: 
            GPIO.output(self.trigger_pin, GPIO.HIGH if self.btn_trigger.isChecked() else GPIO.LOW)
        
        if self.btn_trigger.isChecked():
            self.add_log(f'Trigger enabled: {trigger_cmd.output_filename if trigger_cmd.output_filename else "no filename"}', 'INFO')
        else:
            self.add_log('Trigger stopped', 'INFO')
        
        self.set_btn_enable()

    def set_btn_enable(self):
        bridge_on = self.btn_ros_bridge.isChecked()
        
        # E-Stop, IMU, Set Zero：ROS Bridge 啟動後始終可用
        self.btn_estop.setEnabled(bridge_on)
        self.btn_imu.setEnabled(bridge_on)
        self.btn_set_zero.setEnabled(bridge_on)
        self.btn_trigger.setEnabled(bridge_on)
        
        # 使用 Enum 判斷當前模式
        current = self.robot_state.robot_mode if hasattr(self.robot_state, 'robot_mode') else -1
        
        # 定義拓樸邏輯:
        # 0 <=> 2  (SYSTEM_ON <=> IDLE)
        # 0 <=> 4  (SYSTEM_ON <=> MOTORCONFIG)
        # 2 <=> 3  (IDLE <=> STANDBY)
        # 2 <=> 4  (IDLE <=> MOTORCONFIG)
        
        if not bridge_on:
            # ROS Bridge 未啟動：所有FSM按鈕禁用
            self.btn_system.setEnabled(False)
            self.btn_idle.setEnabled(False)
            self.btn_standby.setEnabled(False)
            self.btn_motorconfig.setEnabled(False)
        else:
            # ROS Bridge 已啟動
            if current == -1:
                # 尚未收到狀態：啟用 Idle 和 Config 作為初始選項
                self.btn_system.setEnabled(False)
                self.btn_idle.setEnabled(True)
                self.btn_standby.setEnabled(False)
                self.btn_motorconfig.setEnabled(True)
            else:
                # 根據當前模式決定可用的轉換
                # System ON (0): 可去 2(IDLE) 或 4(CONFIG)
                # IDLE (2): 可去 0(SYSTEM_ON), 3(STANDBY), 4(CONFIG)
                # STANDBY (3): 可去 2(IDLE)
                # CONFIG (4): 可去 0(SYSTEM_ON), 2(IDLE)
                
                # System ON Button: 從 2, 4 可以進入；從 0 可以退出(去2)
                self.btn_system.setEnabled(current in [ROBOTMODE.IDLE, ROBOTMODE.MOTORCONFIG])
                
                # Idle Button: 從 0, 3, 4 可以進入
                self.btn_idle.setEnabled(current in [ROBOTMODE.SYSTEM_ON, ROBOTMODE.STANDBY, ROBOTMODE.MOTORCONFIG])
                
                # Standby Button: 只能從 2(IDLE) 進入
                self.btn_standby.setEnabled(current == ROBOTMODE.IDLE)
                
                # Config Button: 從 0, 2 可以進入
                self.btn_motorconfig.setEnabled(current in [ROBOTMODE.SYSTEM_ON, ROBOTMODE.IDLE])

    def reset(self):
        self.btn_system.setChecked(False)
        self.btn_trigger.setChecked(False)
        self.publish_trigger_cmd()

    def power_state_cb(self, state): self.power_state_signal.emit(state)
    def robot_state_cb(self, state): self.robot_state_signal.emit(state)
    def motor_state_cb(self, state): self.motor_state_signal.emit(state)
    def log_cb(self, log_msg): self.log_state_signal.emit(log_msg)

    def _handle_power_state_update(self, state):
        self.power_state = state
        # Update power badges
        try:
            v_total = float(getattr(state, 'v_0', 0.0))
        except Exception:
            v_total = 0.0

        try:
            i_total = float(getattr(state, 'i_1', 0.0))
        except Exception:
            i_total = 0.0
        # i_total = 0.0
        # for idx in range(1, 12):
        #     val = getattr(state, f'i_{idx}', 0.0)
        #     try:
        #         i_total += float(val)
        #     except Exception:
        #         pass
        soc = self._soc_from_voltage(v_total)
        self.lbl_voltage.setText(f"{v_total:.1f} V")
        self.lbl_soc.setText(f"{soc:.0f} %")
        self.lbl_current.setText(f"{i_total:.2f} A")
        power = v_total * i_total
        self.lbl_power.setText(f"{power:.1f} W")
        self.set_btn_enable()

    def _soc_from_voltage(self, v_total: float) -> float:
        V_MIN = 42.0  # 3.5V * 12
        V_MAX = 50.4  # 4.2V * 12
        if V_MAX <= V_MIN:
            return 0.0
        soc = (v_total - V_MIN) / (V_MAX - V_MIN) * 100.0
        if soc > 100.0:
            soc = 100.0
        if soc < 0.0:
            soc = 0.0
        return soc

    def _handle_log_update(self, log_msg):
        """Handle incoming log messages from lower-level systems"""
        self.log_state = log_msg
        
        # Extract log information
        level = log_msg.level
        node_name = log_msg.node_name if hasattr(log_msg, 'node_name') else 'unknown'
        message = log_msg.message if hasattr(log_msg, 'message') else ''
        
        # Convert timestamp
        if hasattr(log_msg.header, 'stamp'):
            stamp = log_msg.header.stamp
            timestamp = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9).strftime('%Y-%m-%d %H:%M:%S.%f')
        else:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        
        # Map level to name and color
        level_map = {
            LOGLEVEL.DEBUG: ('DEBUG', '#2196f3'),  # BLUE
            LOGLEVEL.INFO: ('INFO ', '#00e676'),   # GREEN
            LOGLEVEL.WARN: ('WARN ', '#ffea00'),   # YELLOW
            LOGLEVEL.ERROR: ('ERROR', '#ff5252'),  # RED
            LOGLEVEL.FATAL: ('FATAL', '#d32f2f'),  # BOLD RED
        }
        
        level_name, color = level_map.get(level, ('UNKNOWN', '#ffffff'))
        
        # Format and display log with color
        log_html = f'<span style="color:#888;">[{timestamp}]</span> '
        log_html += f'<span style="color:{color}; font-weight:bold;">[{level_name}]</span> '
        log_html += f'<span style="color:#aaa;">[{node_name}]</span> '
        log_html += f'<span style="color:#ddd;">{message}</span>'
        
        self.text_log.append(log_html)
        self.text_log.verticalScrollBar().setValue(self.text_log.verticalScrollBar().maximum())
        
        # Check if set_zero has completed
        if node_name == 'corgi_set_zero' and 'Set Zero Completed' in message:
            self._on_set_zero_completed()
        
        # Handle ERROR and FATAL: clear pending mode as lower system reverted
        if level in [LOGLEVEL.ERROR, LOGLEVEL.FATAL]:
            if self._pending_robot_mode is not None:
                reverted_mode = ROBOTMODE(self._pending_robot_mode).name if self._pending_robot_mode in ROBOTMODE.__members__.values() else str(self._pending_robot_mode)
                self.add_log(f'[SYSTEM] Command to {reverted_mode} failed - system reverted', 'WARN')
                self._pending_robot_mode = None
                self.set_btn_enable()
    
    def _handle_robot_state_update(self, state):
        self.robot_state = state
        current_mode = int(state.robot_mode)
        
        # Clear pending command once state matches
        if self._pending_robot_mode is not None and current_mode == int(self._pending_robot_mode):
            self.add_log(f'[SYSTEM] Robot mode reached: {ROBOTMODE(self._pending_robot_mode).name} ({self._pending_robot_mode})', 'INFO')
            self._pending_robot_mode = None
            
            # Launch config panel when entering MOTORCONFIG mode
            if current_mode == ROBOTMODE.MOTORCONFIG:
                self.launch_config_panel()
        
        # Update last confirmed mode
        self._last_confirmed_mode = current_mode
        
        # 使用 Enum 更新 UI 顯示
        try:
            mode_enum = ROBOTMODE(state.robot_mode)
            mode_text = mode_enum.name
        except ValueError:
            mode_text = "---"

        self.label_robot_mode_value.setText(mode_text)
        
        if state.robot_mode == ROBOTMODE.SYSTEM_ON: color = "#00e676"
        elif state.robot_mode == ROBOTMODE.IDLE: color = "#2979ff"
        elif state.robot_mode == ROBOTMODE.MOTORCONFIG: color = "#ffea00"
        else: color = "#bdbdbd"
        self.label_robot_mode_value.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 20px;")
        self.set_btn_enable()

    def _handle_motor_state_update(self, state):
        self.motor_state = state
        if not hasattr(state, 'module_a'): return
        modules = [state.module_a, state.module_b, state.module_c, state.module_d]
        for module_idx, module in enumerate(modules):
            if not hasattr(module, 'motor_mode'): continue
            for motor_idx, side in enumerate(['L', 'R']):
                motor_num = module_idx * 2 + motor_idx + 1
                key = f'M{motor_num}'
                if key in self.motor_labels:
                    pos = 0.0
                    if hasattr(module, 'position') and len(module.position) > motor_idx:
                        pos = np.degrees(module.position[motor_idx])
                    temp = 0 
                    if hasattr(module, 'temperature') and len(module.temperature) > motor_idx:
                         temp = module.temperature[motor_idx]
                    self.motor_labels[key].setText(f"{key}: {pos:.1f}° | {temp}°C")
                    if temp > 60: self.motor_labels[key].setStyleSheet("color: #ff5252; font-weight: bold;")
                    else: self.motor_labels[key].setStyleSheet("color: #aaa;")

    def launch_config_panel(self):
        """Launch the config panel when entering MOTORCONFIG mode"""
        try:
            # Check if config panel is already running
            if hasattr(self, 'process_config') and self.process_config is not None and self.process_config.poll() is None:
                self.add_log('Config Panel already running', 'WARN')
                return
            
            # Get the path to the config panel script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_panel_path = os.path.join(script_dir, 'corgi_config_panel_dev.py')
            
            # Launch the config panel
            self.process_config = subprocess.Popen(['python3', config_panel_path])
            self.add_log('Config Panel launched', 'SYSTEM')
        except Exception as e:
            self.add_log(f'Failed to launch Config Panel: {e}', 'ERROR')

    def _on_set_zero_completed(self):
        """Handle set_zero completion"""
        if hasattr(self, 'process_set_zero') and self.process_set_zero is not None:
            self.process_set_zero.wait(timeout=1.0)
            self.process_set_zero = None
        
        # Re-enable button after completion
        self.btn_set_zero.setEnabled(True)
        self.btn_set_zero.setText('Set Zero')
        self.add_log('Motor zero points set successfully', 'SYSTEM')

    def add_log(self, message, level='INFO'):
        """Add log message with optional level for color coding"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        
        # Color map for internal logs (matching lower system)
        color_map = {
            'DEBUG': '#2196f3',
            'INFO': '#00e676',
            'WARN': '#ffea00',
            'ERROR': '#ff5252',
            'FATAL': '#d32f2f',
            'SYSTEM': '#bb86fc',  # Purple for system messages
        }
        color = color_map.get(level, '#ffffff')
        
        # Format level to be 5 chars like lower system (e.g., 'INFO ')
        level_padded = f'{level:5s}'
        
        log_html = f'<span style="color:#888;">[{timestamp}]</span> '
        log_html += f'<span style="color:{color}; font-weight:bold;">[{level_padded}]</span> '
        log_html += f'<span style="color:#aaa;">[orin]</span> '
        log_html += f'<span style="color:#ddd;">{message}</span>'
        
        self.text_log.append(log_html)
        self.text_log.verticalScrollBar().setValue(self.text_log.verticalScrollBar().maximum())

    def timer_update(self): pass
    def closeEvent(self, event):
        try: self.node.destroy_subscription(self.power_state_sub)
        except: pass
        try: self.node.destroy_subscription(self.robot_state_sub)
        except: pass
        try: self.node.destroy_subscription(self.log_sub)
        except: pass
        self.reset()
        if hasattr(self, 'process_bridge') and self.process_bridge is not None:
            try:
                self.process_bridge.send_signal(signal.SIGINT)
                self.process_bridge.wait(timeout=1.0)
            except:
                pass
        if hasattr(self, 'process_imu') and self.process_imu is not None:
            try:
                self.process_imu.send_signal(signal.SIGINT)
                self.process_imu.wait(timeout=1.0)
            except:
                pass
        if hasattr(self, 'process_config') and self.process_config is not None:
            try:
                self.process_config.terminate()
                self.process_config.wait(timeout=1.0)
            except:
                pass
        if hasattr(self, 'process_recorder') and self.process_recorder is not None:
            try:
                self.process_recorder.send_signal(signal.SIGINT)
                self.process_recorder.wait(timeout=1.0)
            except:
                pass
        if hasattr(self, 'process_set_zero') and self.process_set_zero is not None:
            try:
                self.process_set_zero.send_signal(signal.SIGINT)
                self.process_set_zero.wait(timeout=1.0)
            except:
                pass
        try: rclpy.try_shutdown()
        except: pass
        super(CorgiControlPanel, self).closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())