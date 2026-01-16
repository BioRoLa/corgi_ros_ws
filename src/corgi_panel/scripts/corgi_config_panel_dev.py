#!/usr/bin/env python3
import os
import sys
import threading
import time
from enum import IntEnum
from collections import deque
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QGroupBox, QLineEdit, 
                             QTabWidget, QFormLayout, QMessageBox, QTextEdit, 
                             QProgressBar, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject

from corgi_msgs.msg import ConfigStamped, RobotCmdStamped
from motor_config import MotorParameterRegistry, ConfigType

# --- Definitions ---

class Module(IntEnum):
    MODULE_A = 0
    MODULE_B = 1
    MODULE_C = 2
    MODULE_D = 3

class Motor(IntEnum):
    MOTOR_R = 0  # Right motor (Belt)
    MOTOR_L = 1  # Left motor (Direct)

class ConfigMode(IntEnum):
    READ = 0
    WRITE = 1

class RobotMode(IntEnum):
    SYSTEM_ON = 0
    INIT = 1
    IDLE = 2
    STANDBY = 3
    MOTORCONFIG = 4

# 1. Error Codes Definition
class ErrorCode(IntEnum):
    CODE_CONFIG_SUCCESS = 0
    CODE_INVALID_VALUE = 1
    CODE_READ_ONLY = 2
    CODE_INVALID_ADDR = 3
    CODE_INVALID_CMD = 4
    INVALIDE_MODULE_INDEX = 5
    INVALIDE_MOTOR_INDEX = 6
    INVALIDE_SEQ = 7

# Log Levels (Aligned with Control Panel)
class LOGLEVEL(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4

# --- ROS Worker ---

class RosWorker(QObject):
    msg_received = pyqtSignal(ConfigStamped)

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub = None
        self.sub = None
        self.robot_cmd_pub = None

    def start_ros(self):
        rclpy.init()
        self.node = rclpy.create_node('corgi_config_panel')
        self.pub = self.node.create_publisher(ConfigStamped, 'config/command', 10)
        self.sub = self.node.create_subscription(ConfigStamped, 'config/state', self.msg_cb, 10)
        self.robot_cmd_pub = self.node.create_publisher(RobotCmdStamped, 'robot/command', 10)
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        thread = threading.Thread(target=self.executor.spin, daemon=True)
        thread.start()

    def msg_cb(self, msg):
        self.msg_received.emit(msg)

    def send_cmd(self, msg):
        if self.pub:
            self.pub.publish(msg)
    
    def send_robot_cmd(self, msg):
        if self.robot_cmd_pub:
            self.robot_cmd_pub.publish(msg)

# --- UI Components ---

class ParameterTab(QWidget):
    request_write = pyqtSignal(str, object)

    def __init__(self, param_registry):
        super().__init__()
        self.registry = param_registry
        self.inputs = {}
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        order = ["System", "Control", "Limits", "Motor", "Reserved"]
        sorted_groups = sorted(self.registry.get_all_groups(), key=lambda x: order.index(x) if x in order else 99)

        for group in sorted_groups:
            page = QWidget()
            form = QFormLayout(page)
            form.setLabelAlignment(Qt.AlignRight)
            
            for param in self.registry.get_group(group):
                name = param.name
                inp = QLineEdit()
                inp.setPlaceholderText("Wait...")
                inp.setEnabled(False)
                
                if not param.writable:
                    inp.setStyleSheet("background-color: #333; color: #aaa; border: 1px solid #444;")
                else:
                    inp.returnPressed.connect(lambda n=name: self.handle_enter(n))
                    inp.setStyleSheet("background-color: #2a2a2a; color: #fff; border: 1px solid #555;")
                
                self.inputs[name] = inp
                label_text = f"{param.description} ({name})" if param.description else name
                if param.unit:
                    label_text += f" [{param.unit}]"
                form.addRow(label_text, inp)
            
            self.tabs.addTab(page, group)
        layout.addWidget(self.tabs)

    def handle_enter(self, name):
        val = self.inputs[name].text()
        self.request_write.emit(name, val)

    def update_field_from_motor(self, name, value):
        if name in self.inputs:
            inp = self.inputs[name]
            inp.setText(str(value))
            param = self.registry.get_by_name(name)
            if param and param.writable:
                inp.setEnabled(True)
                inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")

    def set_all_enabled(self, enabled):
        for name, inp in self.inputs.items():
            param = self.registry.get_by_name(name)
            if param and param.writable:
                inp.setEnabled(enabled)
                if not enabled:
                     inp.setStyleSheet("background-color: #2a2a2a; color: #aaa; border: 1px solid #555;")
                else:
                     inp.setStyleSheet("background-color: #404040; color: #fff; border: 1px solid #555;")

# --- Main Window ---

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        self.registry = MotorParameterRegistry()
        self.worker = RosWorker()
        self.seq_counter = 0
        self.robot_cmd_seq = 0
        
        self.current_module = None
        self.current_motor = None
        
        self.load_queue = deque()
        self.pending_write = None
        self.retry_count = 0
        self.max_retries = 3
        self.successful_reads = 0
        
        # 5. Log Filter Init
        self.min_log_level = LOGLEVEL.DEBUG
        
        self.init_ui()
        self.init_ros()
        
        self.tx_timer = QTimer()
        self.tx_timer.setSingleShot(True)
        self.tx_timer.timeout.connect(self.handle_timeout)

    def init_ui(self):
        self.setWindowTitle("Corgi Motor Configurator")
        self.resize(1200, 800)
        # Stylesheet matching control panel dark theme
        self.setStyleSheet("""
            QWidget { background-color: #2b2b2b; color: #fff; font-family: 'Segoe UI', 'Ubuntu'; font-size: 14px; }
            QLineEdit { padding: 4px; border-radius: 3px; background-color: #202020; border: 1px solid #555; }
            QLineEdit:focus { border: 1px solid #2196F3; }
            QComboBox { background-color: #404040; padding: 5px; border: 1px solid #555; }
            QGroupBox { border: 1px solid #555; margin-top: 20px; font-weight: bold; color: #ccc; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
            QTextEdit { font-family: 'Consolas', monospace; font-size: 12px; background-color: #1e1e1e; border: 1px solid #444; }
            QPushButton { background-color: #404040; border: 1px solid #555; padding: 5px; border-radius: 4px; }
            QPushButton:hover { background-color: #505050; }
        """)

        main_layout = QHBoxLayout(self)

        # Left Panel
        left_panel = QVBoxLayout()
        sel_group = QGroupBox("Target")
        sel_layout = QFormLayout(sel_group)
        
        self.cb_module = QComboBox()
        self.cb_module.addItems(["- Select -", "Module A", "Module B", "Module C", "Module D"])
        self.cb_module.currentIndexChanged.connect(self.on_selection_change)
        
        self.cb_motor = QComboBox()
        self.cb_motor.addItems(["- Select -", "Motor R (Belt)", "Motor L (Direct)"])
        self.cb_motor.setEnabled(False)
        self.cb_motor.currentIndexChanged.connect(self.on_motor_selected)
        
        sel_layout.addRow("Module:", self.cb_module)
        sel_layout.addRow("Motor:", self.cb_motor)
        left_panel.addWidget(sel_group)
        
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout(action_group)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setEnabled(False)
        self.btn_refresh.clicked.connect(self.refresh_current_motor)
        self.btn_refresh.setStyleSheet("QPushButton { background-color: #3a6ea5; color: white; border: none; } QPushButton:hover { background-color: #4a7eb5; }")
        
        self.btn_rest = QPushButton("REST (System On)")
        self.btn_rest.clicked.connect(self.send_rest_command)
        self.btn_rest.setStyleSheet("QPushButton { background-color: #2e7d32; color: white; border: none; } QPushButton:hover { background-color: #3e8d42; }")
        
        action_layout.addWidget(self.btn_refresh)
        action_layout.addWidget(self.btn_rest)
        left_panel.addWidget(action_group)
        
        # Log View
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        
        # Log Filter
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Min Level:"))
        self.cb_log_level = QComboBox()
        self.cb_log_level.addItems(['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'])
        self.cb_log_level.currentIndexChanged.connect(self.on_log_level_changed)
        filter_layout.addWidget(self.cb_log_level)
        filter_layout.addStretch(1)
        log_layout.addLayout(filter_layout)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        left_panel.addWidget(log_group, 1)
        
        main_layout.addLayout(left_panel, 1)

        # Right Panel
        right_panel = QVBoxLayout()
        self.status_label = QLabel("Please select a motor.")
        self.status_label.setStyleSheet("color: #aaa; font-style: italic; font-size: 16px;")
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setStyleSheet("QProgressBar { border: 1px solid #555; border-radius: 5px; text-align: center; } QProgressBar::chunk { background-color: #00e676; }")
        
        self.param_widget = ParameterTab(self.registry)
        self.param_widget.request_write.connect(self.start_write_transaction)
        
        right_panel.addWidget(self.status_label)
        right_panel.addWidget(self.progress)
        right_panel.addWidget(self.param_widget)
        
        main_layout.addLayout(right_panel, 3)

    def init_ros(self):
        self.worker.msg_received.connect(self.handle_ros_msg)
        self.worker.start_ros()
        self.add_log("System Ready.", LOGLEVEL.INFO, source="system")

    # --- 3. Log System (Aligned with Control Panel) ---
    def add_log(self, message, level=LOGLEVEL.INFO, source="system"):
        if level < self.min_log_level:
            return

        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        # Color mapping same as control panel
        level_map = {
            LOGLEVEL.DEBUG: ('DEBUG', '#2196f3'),  # BLUE
            LOGLEVEL.INFO:  ('INFO ', '#00e676'),  # GREEN
            LOGLEVEL.WARN:  ('WARN ', '#ffea00'),  # YELLOW
            LOGLEVEL.ERROR: ('ERROR', '#ff5252'),  # RED
            LOGLEVEL.FATAL: ('FATAL', '#d32f2f'),  # BOLD RED
        }
        
        level_name, color = level_map.get(level, ('UNK  ', '#ffffff'))

        # Source styling
        source_str = f"[{source}]"
        
        log_html = f'<span style="color:#888;">[{timestamp}]</span> '
        log_html += f'<span style="color:{color}; font-weight:bold;">[{level_name}]</span> '
        log_html += f'<span style="color:#aaa;">{source_str}</span> '
        log_html += f'<span style="color:#ddd;">{message}</span>'
        
        self.log_view.append(log_html)
        # Auto scroll
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def on_log_level_changed(self, index):
        self.min_log_level = LOGLEVEL(index)
        self.add_log(f"Log filter set to {LOGLEVEL(index).name}", LOGLEVEL.INFO, "system")

    # --- Selection Logic ---
    def on_selection_change(self):
        idx = self.cb_module.currentIndex()
        if idx > 0:
            self.current_module = idx - 1
            self.cb_motor.setEnabled(True)
        else:
            self.current_module = None
            self.cb_motor.setEnabled(False)
            self.cb_motor.setCurrentIndex(0)

    def on_motor_selected(self):
        idx = self.cb_motor.currentIndex()
        if idx <= 0 or self.current_module is None:
            return
        self.current_motor = idx - 1
        self.start_load_sequence()
    
    def refresh_current_motor(self):
        if self.current_module is not None and self.current_motor is not None:
            self.add_log("Refreshing motor parameters...", LOGLEVEL.INFO, "user")
            self.start_load_sequence()
        else:
            self.add_log("No motor selected to refresh", LOGLEVEL.WARN, "user")

    # --- Read Sequence ---
    def start_load_sequence(self):
        self.load_queue.clear()
        self.successful_reads = 0
        
        self.param_widget.set_all_enabled(False)
        self.cb_module.setEnabled(False)
        self.cb_motor.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.progress.setVisible(True)
        self.progress.setValue(0)
        self.status_label.setText("Reading parameters...")
        
        for addr in range(8):
            self.load_queue.append((ConfigType.INT, addr))
        for addr in range(30):
            self.load_queue.append((ConfigType.FLOAT, addr))
            
        self.total_load_items = len(self.load_queue)
        self.process_load_queue()

    def process_load_queue(self):
        if not self.load_queue:
            self.finish_loading()
            return

        c_type, addr = self.load_queue[0]
        self.retry_count = 0 
        
        done = self.total_load_items - len(self.load_queue)
        self.progress.setValue(int((done / self.total_load_items) * 100))

        # We set current_seq in send_config_cmd
        self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
        self.tx_timer.start(3000)

    def finish_loading(self):
        if self.successful_reads == 0:
            self.status_label.setText("Connection Failed")
            self.status_label.setStyleSheet("color: #ff5252; font-weight: bold;")
            self.add_log("Failed to read parameters.", LOGLEVEL.ERROR, "system")
        else:
            total_params = self.total_load_items
            if self.successful_reads < total_params:
                self.status_label.setText(f"Partial: {self.successful_reads}/{total_params}")
                self.status_label.setStyleSheet("color: #ffea00; font-weight: bold;")
                self.add_log(f"Partial success ({self.successful_reads}/{total_params})", LOGLEVEL.WARN, "system")
            else:
                self.status_label.setText(f"Connected: M{self.current_module}-M{self.current_motor}")
                self.status_label.setStyleSheet("color: #00e676; font-weight: bold;")
                self.add_log("Read complete.", LOGLEVEL.INFO, "system")
        
        self.progress.setVisible(False)
        self.cb_module.setEnabled(True)
        self.cb_motor.setEnabled(True)
        self.btn_refresh.setEnabled(True)

    # --- Write Transaction ---
    def start_write_transaction(self, name, value_str):
        param = self.registry.get_by_name(name)
        if not param: return

        try:
            if param.data_type == ConfigType.INT:
                val = int(float(value_str))
            else:
                val = float(value_str)
            
            if val < param.min_value or val > param.max_value:
                QMessageBox.warning(self, "Limit Error", f"Value out of range ({param.min_value} ~ {param.max_value})")
                return
        except ValueError:
            QMessageBox.warning(self, "Format Error", "Invalid number format.")
            return

        self.param_widget.set_all_enabled(False)
        self.status_label.setText(f"Writing {name}...")
        
        # [UPDATED] Simplified pending write structure
        self.pending_write = {
            'name': name,
            'target_val': val,
            'param': param,
            'seq_sent': 0  # To store the sequence ID we sent
        }

        self.add_log(f"Writing {name} = {val}", LOGLEVEL.INFO, "orin")
        
        # Send Write Request
        sent_seq = self.send_config_cmd(ConfigMode.WRITE, param.data_type, param.address, val)
        self.pending_write['seq_sent'] = sent_seq
        self.tx_timer.start(3000)

    # --- Communication Core ---
    def send_config_cmd(self, mode, c_type, addr, val):
        self.seq_counter = (self.seq_counter + 1) % 65535 # Prevent overflow
        
        msg = ConfigStamped()
        msg.header.seq = self.seq_counter
        msg.header.stamp = self.worker.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.transmit = True
        msg.module = int(self.current_module)
        msg.motor = int(self.current_motor)
        msg.mode = int(mode)
        msg.type = int(c_type)
        msg.address = int(addr)
        msg.error_code = 0
        
        if c_type == ConfigType.INT:
            msg.value_i = int(val)
            msg.value_f = 0.0
        else:
            msg.value_i = 0
            msg.value_f = float(val)
            
        self.worker.send_cmd(msg)
        
        # Log command
        mode_str = "READ" if mode == ConfigMode.READ else "WRITE"
        self.add_log(f"CMD {mode_str} T:{c_type} A:{addr} SEQ:{self.seq_counter}", LOGLEVEL.DEBUG, "orin")
        
        return self.seq_counter

    def handle_ros_msg(self, msg):
        # --- [FIX] Filter mismatching sequence IDs ---
        # This prevents processing junk messages with seq=0 or delayed responses
        if msg.header.seq != self.seq_counter:
            return

        # At this point, we are sure the message corresponds to the latest request
        error_code = msg.error_code
        error_name = ErrorCode(error_code).name if error_code in ErrorCode.__members__.values() else f"ERR_{error_code}"

        # Debug log for valid incoming message
        self.add_log(f"REPLY SEQ:{msg.header.seq} ERR:{error_code}({error_name}) V:{msg.value_f:.2f}", LOGLEVEL.DEBUG, "fpga_driver")

        # --- Priority 1: Handle Read Queue (Scanning) ---
        if self.load_queue:
            req_type, req_addr = self.load_queue[0]
            
            # Since we filtered seq above, we just check Type and Address match
            if msg.type == int(req_type) and msg.address == req_addr:
                self.tx_timer.stop()
                self.load_queue.popleft()
                
                # Check for error
                if error_code != ErrorCode.CODE_CONFIG_SUCCESS:
                    self.add_log(f"Read Error on {req_type.name}:{req_addr}: {error_name}", LOGLEVEL.ERROR, "fpga_driver")
                    # Continue to next item even if error occurs
                else:
                    # Success
                    param = self.registry.get_by_type_and_address(req_type, req_addr)
                    if param:
                        val = msg.value_i if req_type == ConfigType.INT else msg.value_f
                        self.param_widget.update_field_from_motor(param.name, val)
                        self.successful_reads += 1
                
                # Process next
                self.process_load_queue()
            return

        # --- Priority 2: Handle Write Transaction ---
        if self.pending_write:
            # We already confirmed seq match, so we pass True
            self._handle_write_logic(msg, is_seq_match=True, error_code=error_code)

    def _handle_write_logic(self, msg, is_seq_match, error_code):
        # [UPDATED] Simplified write logic: Check Seq -> Check Err -> Check Value match
        tx = self.pending_write
        param = tx['param']
        
        # 1. Basic check: Ensure msg targets the current parameter
        if ConfigType(msg.type) != param.data_type or msg.address != param.address:
            return 
            
        # 2. Sequence check (passed from caller)
        if not is_seq_match:
            return

        # 3. Check Error Code
        if error_code != ErrorCode.CODE_CONFIG_SUCCESS:
            error_name = ErrorCode(error_code).name if error_code in ErrorCode.__members__.values() else f"ERR_{error_code}"
            self.add_log(f"Write Failed: {error_name}", LOGLEVEL.ERROR, "fpga_driver")
            self.end_transaction(success=False, msg=f"Write Error: {error_name}")
            return

        # 4. Check if echoed value matches target value
        received_val = msg.value_i if param.data_type == ConfigType.INT else msg.value_f
        target_val = tx['target_val']
        
        is_match = False
        if param.data_type == ConfigType.INT:
            is_match = (received_val == int(target_val))
        else:
            # Floating point tolerance check
            is_match = abs(received_val - float(target_val)) < 0.001
        
        if is_match:
            self.add_log(f"Write Success (Ack): {param.name} -> {received_val}", LOGLEVEL.INFO, "system")
            self.param_widget.update_field_from_motor(param.name, received_val)
            self.end_transaction(success=True)
        else:
            self.add_log(f"Write Mismatch! Sent: {target_val}, Echo: {received_val}", LOGLEVEL.ERROR, "system")
            self.end_transaction(success=False, msg=f"Mismatch: Sent {target_val} != Got {received_val}")

    def end_transaction(self, success, msg=""):
        self.pending_write = None
        self.tx_timer.stop()
        if not success:
            QMessageBox.critical(self, "Write Failed", msg)
            self.status_label.setText("Write Failed")
        else:
            self.status_label.setText("Write Successful")
        self.param_widget.set_all_enabled(True)

    def handle_timeout(self):
        if self.load_queue:
            # For read loop, timeout acts like an error -> Log and Skip
            c_type, addr = self.load_queue[0]
            self.add_log(f"Timeout reading {c_type.name}:{addr}. Skipping.", LOGLEVEL.WARN, "system")
            
            self.load_queue.popleft()
            self.process_load_queue()
            
        elif self.pending_write:
            self.add_log("Write operation timed out.", LOGLEVEL.ERROR, "system")
            self.end_transaction(success=False, msg="Motor did not respond (Timeout).")
    
    def send_rest_command(self):
        self.robot_cmd_seq += 1
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self.robot_cmd_seq
        robot_cmd.header.stamp = self.worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(RobotMode.SYSTEM_ON)
        
        self.worker.send_robot_cmd(robot_cmd)
        self.add_log("Sent REST (SYSTEM_ON)", LOGLEVEL.INFO, "orin")
        self.close()
    
    def closeEvent(self, event):
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())