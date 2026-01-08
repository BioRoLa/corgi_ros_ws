#!/usr/bin/env python3
import os
import sys
import threading
import time
from enum import IntEnum
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QGroupBox, QLineEdit, 
                             QTabWidget, QFormLayout, QMessageBox, QTextEdit, 
                             QProgressBar, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject

from corgi_msgs.msg import ConfigStamped

from corgi_msgs.msg import RobotCmdStamped

from motor_config import MotorParameterRegistry, ConfigType

# Module enumeration (A, B, C, D correspond to the 4 motor modules)
class Module(IntEnum):
    MODULE_A = 0
    MODULE_B = 1
    MODULE_C = 2
    MODULE_D = 3

# Motor enumeration (R: Belt drive, L: Direct drive)
class Motor(IntEnum):
    MOTOR_R = 0  # Right motor (Belt)
    MOTOR_L = 1  # Left motor (Direct)

# Configuration command mode
class ConfigMode(IntEnum):
    READ = 0   # Read parameter from motor
    WRITE = 1  # Write parameter to motor

# Robot FSM states
class RobotMode(IntEnum):
    SYSTEM_ON = 0    # System power on, motors enabled
    INIT = 1         # Initialization state
    IDLE = 2         # Idle state, motors disabled
    STANDBY = 3      # Standby state, ready for motion
    MOTORCONFIG = 4  # Motor configuration mode

# ROS Worker: Handles ROS2 communication in a separate thread
class RosWorker(QObject):
    msg_received = pyqtSignal(ConfigStamped)  # Signal emitted when config message received

    def __init__(self):
        super().__init__()
        self.node = None              # ROS2 node
        self.pub = None               # Config command publisher
        self.sub = None               # Config state subscriber
        self.robot_cmd_pub = None     # Robot command publisher (for REST)

    def start_ros(self):
        """Initialize ROS2 node and start spinning in background thread"""
        rclpy.init()
        self.node = rclpy.create_node('corgi_config_panel')

        # Publisher for motor configuration commands
        self.pub = self.node.create_publisher(ConfigStamped, 'config/command', 10)
        # Subscriber for motor configuration state feedback
        self.sub = self.node.create_subscription(ConfigStamped, 'config/state', self.msg_cb, 10)
        # Publisher for robot mode commands
        self.robot_cmd_pub = self.node.create_publisher(RobotCmdStamped, 'robot/command', 10)
        
        # Start executor in daemon thread
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        thread = threading.Thread(target=self.executor.spin, daemon=True)
        thread.start()

    def msg_cb(self, msg):
        """Callback for config state messages - emit signal to main thread"""
        self.msg_received.emit(msg)

    def send_cmd(self, msg):
        """Publish config command message"""
        if self.pub:
            self.pub.publish(msg)
    
    def send_robot_cmd(self, msg):
        """Publish robot mode command message"""
        if self.robot_cmd_pub:
            self.robot_cmd_pub.publish(msg)

# Parameter Tab Widget: Displays motor parameters in categorized tabs
class ParameterTab(QWidget):
    request_write = pyqtSignal(str, object)  # Signal: (parameter_name, value)

    def __init__(self, param_registry):
        super().__init__()
        self.registry = param_registry  # Motor parameter registry
        self.inputs = {}                # Dict: parameter_name -> QLineEdit widget
        self.init_ui()

    def init_ui(self):
        """Initialize UI with tabbed parameter groups"""
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()

        # Define tab order for better organization
        order = ["System", "Control", "Limits", "Motor", "Reserved"]
        sorted_groups = sorted(self.registry.get_all_groups(), key=lambda x: order.index(x) if x in order else 99)

        # Create a tab for each parameter group
        for group in sorted_groups:
            page = QWidget()
            form = QFormLayout(page)
            form.setLabelAlignment(Qt.AlignRight)
            
            # Add parameters to the form
            for param in self.registry.get_group(group):
                name = param.name

                inp = QLineEdit()
                inp.setPlaceholderText("Wait...")  # Show waiting state
                inp.setEnabled(False)  # Locked until value is read from motor
                
                if not param.writable:
                    # Read-only parameter: gray background
                    inp.setStyleSheet("background-color: #333; color: #aaa; border: 1px solid #444;")
                else:
                    # Writable parameter: press Enter to write to motor
                    inp.returnPressed.connect(lambda n=name: self.handle_enter(n))
                    # Initially locked (dark), unlocks when data received
                    inp.setStyleSheet("background-color: #2a2a2a; color: #fff; border: 1px solid #555;")
                
                self.inputs[name] = inp
                
                # Create label with description and unit
                label_text = f"{param.description} ({name})" if param.description else name
                if param.unit:
                    label_text += f" [{param.unit}]"
                
                form.addRow(label_text, inp)
            
            self.tabs.addTab(page, group)
        layout.addWidget(self.tabs)

    def handle_enter(self, name):
        """Handle Enter key press - emit write request signal"""
        val = self.inputs[name].text()
        self.request_write.emit(name, val)

    def update_field_from_motor(self, name, value):
        """Update field with value read from motor and unlock if writable"""
        if name in self.inputs:
            inp = self.inputs[name]
            inp.setText(str(value))
            
            param = self.registry.get_by_name(name)
            if param and param.writable:
                # Unlock writable parameters after successful read
                inp.setEnabled(True)
                inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")

    def set_all_enabled(self, enabled):
        """Enable/disable all writable parameter inputs"""
        for name, inp in self.inputs.items():
            param = self.registry.get_by_name(name)
            # Only affect writable parameters (read-only stay disabled)
            if param and param.writable:
                inp.setEnabled(enabled)
                if not enabled:
                     # Locked state: dark gray
                     inp.setStyleSheet("background-color: #2a2a2a; color: #aaa; border: 1px solid #555;")
                else:
                     # Unlocked state: lighter gray
                     inp.setStyleSheet("background-color: #404040; color: #fff; border: 1px solid #555;")

# --- Main Window: Motor Configuration Panel ---
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        # Parameter registry and ROS worker
        self.registry = MotorParameterRegistry()  # Defines all motor parameters
        self.worker = RosWorker()                 # Handles ROS communication
        self.seq_counter = 0                      # Config message sequence counter
        self.robot_cmd_seq = 0                    # Robot command sequence counter
        
        # Current selection state
        self.current_module = None  # Selected module (0-3)
        self.current_motor = None   # Selected motor (0-1)
        
        # Read queue and transaction management
        self.load_queue = deque()       # Queue of (ConfigType, Address) to read
        self.pending_write = None       # Current write transaction info
        self.retry_count = 0            # Current retry attempt counter
        self.max_retries = 3            # Max retries before skipping
        self.successful_reads = 0       # Count of successfully read parameters
        
        self.init_ui()
        self.init_ros()
        
        # Timer for detecting communication timeouts
        self.tx_timer = QTimer()
        self.tx_timer.setSingleShot(True)
        self.tx_timer.timeout.connect(self.handle_timeout)

    def init_ui(self):
        self.setWindowTitle("Corgi Motor Configurator")
        self.resize(1000, 700)
        self.setStyleSheet("""
            QWidget { background-color: #2b2b2b; color: #fff; font-family: 'Segoe UI'; font-size: 14px; }
            QLineEdit { padding: 4px; border-radius: 3px; }
            QLineEdit:focus { border: 1px solid #2196F3; }
            QComboBox { background-color: #404040; padding: 5px; }
            QGroupBox { border: 1px solid #555; margin-top: 20px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
            QTextEdit { font-family: 'Consolas'; font-size: 12px; }
        """)

        main_layout = QHBoxLayout(self)

        # Left Panel: Selection & Log
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
        
        # Action buttons
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout(action_group)
        
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setEnabled(False)
        self.btn_refresh.clicked.connect(self.refresh_current_motor)
        self.btn_refresh.setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }")
        
        self.btn_rest = QPushButton("REST")
        self.btn_rest.clicked.connect(self.send_rest_command)
        self.btn_rest.setStyleSheet("QPushButton { background-color: #2e7d32; color: white; font-weight: bold; }")
        
        action_layout.addWidget(self.btn_refresh)
        action_layout.addWidget(self.btn_rest)
        left_panel.addWidget(action_group)
        
        # Log View
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        left_panel.addWidget(log_group, 1) # stretch
        
        main_layout.addLayout(left_panel, 1)

        # Right Panel: Parameters
        right_panel = QVBoxLayout()
        self.status_label = QLabel("Please select a motor.")
        self.status_label.setStyleSheet("color: #aaa; font-style: italic;")
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        
        self.param_widget = ParameterTab(self.registry)
        self.param_widget.request_write.connect(self.start_write_transaction)
        
        right_panel.addWidget(self.status_label)
        right_panel.addWidget(self.progress)
        right_panel.addWidget(self.param_widget)
        
        main_layout.addLayout(right_panel, 3)

    def init_ros(self):
        """Initialize ROS worker and connect signals"""
        self.worker.msg_received.connect(self.handle_ros_msg)
        self.worker.start_ros()
        self.log("System Ready.", "INFO")

    def log(self, msg, level="INFO"):
        """Add colored log entry with timestamp"""
        color = "#00e676" if level in ["INFO", "SUCCESS"] else "#ff5252" if level == "ERROR" else "#2196F3"
        timestamp = time.strftime("%H:%M:%S")
        self.log_view.append(f'<span style="color:#888">[{timestamp}]</span> <span style="color:{color}"><b>[{level}]</b></span> {msg}')

    # --- Selection Logic ---
    def on_selection_change(self):
        """Handle module selection change - enable/disable motor dropdown"""
        idx = self.cb_module.currentIndex()
        if idx > 0:
            self.current_module = idx - 1
            self.cb_motor.setEnabled(True)
        else:
            self.current_module = None
            self.cb_motor.setEnabled(False)
            self.cb_motor.setCurrentIndex(0)

    def on_motor_selected(self):
        """Handle motor selection - start reading all parameters"""
        idx = self.cb_motor.currentIndex()
        if idx <= 0 or self.current_module is None:
            return

        self.current_motor = idx - 1
        self.start_load_sequence()
    
    def refresh_current_motor(self):
        """Re-read all parameters for the currently selected motor"""
        if self.current_module is not None and self.current_motor is not None:
            self.log("Refreshing motor parameters...", "INFO")
            self.start_load_sequence()
        else:
            self.log("No motor selected to refresh", "WARN")

    # --- Read All Parameters Sequence (Scan) ---
    def start_load_sequence(self):
        """
        Start sequential parameter read process:
        1. INT parameters (Type 0): Address 0~7
        2. FLOAT parameters (Type 1): Address 0~29
        """
        self.load_queue.clear()
        self.successful_reads = 0  # Reset successful read counter
        
        # Lock UI and show progress bar
        self.param_widget.set_all_enabled(False)
        self.cb_module.setEnabled(False)
        self.cb_motor.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.progress.setVisible(True)
        self.progress.setValue(0)
        self.status_label.setText("Reading parameters from motor...")
        
        # Build read queue: all INT addresses, then all FLOAT addresses
        for addr in range(8):
            self.load_queue.append((ConfigType.INT, addr))
        for addr in range(30):
            self.load_queue.append((ConfigType.FLOAT, addr))
            
        self.total_load_items = len(self.load_queue)
        self.process_load_queue()

    def process_load_queue(self):
        """Process next item in read queue or finish if empty"""
        if not self.load_queue:
            # All reads complete
            self.finish_loading()
            return

        c_type, addr = self.load_queue[0]
        self.retry_count = 0  # Reset retry counter for new request
        
        # Update progress bar
        done = self.total_load_items - len(self.load_queue)
        self.progress.setValue(int((done / self.total_load_items) * 100))

        # Send read command (value_f=0, value_i=0 for read mode)
        self.log(f"Reading Type={c_type.name} Addr={addr}", "DEBUG")
        self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
        
        # Set 500ms timeout for response
        self.tx_timer.start(500)

    def finish_loading(self):
        """Complete read sequence - evaluate connection status and unlock UI"""
        if self.successful_reads == 0:
            # No parameters were successfully read - connection failed
            self.status_label.setText(f"Failed to connect to Module {Module(self.current_module).name}, Motor {Motor(self.current_motor).name}")
            self.status_label.setStyleSheet("color: #ff5252; font-weight: bold;")
            self.log("Failed to read any parameters. Cannot connect to motor.", "ERROR")
        else:
            # At least some parameters were read
            total_params = self.total_load_items
            if self.successful_reads < total_params:
                # Partial success - some reads failed
                self.status_label.setText(f"Partial read: {self.successful_reads}/{total_params} params (Module {Module(self.current_module).name}, Motor {Motor(self.current_motor).name})")
                self.status_label.setStyleSheet("color: #ffea00; font-weight: bold;")
                self.log(f"Partial success: {self.successful_reads}/{total_params} parameters read. Use Refresh to retry.", "WARN")
            else:
                # Complete success - all parameters read
                self.status_label.setText(f"Connected: Module {Module(self.current_module).name}, Motor {Motor(self.current_motor).name}")
                self.status_label.setStyleSheet("color: #00e676; font-weight: bold;")
                self.log("All parameters read complete.", "SUCCESS")
        
        # Unlock UI
        self.progress.setVisible(False)
        self.cb_module.setEnabled(True)
        self.cb_motor.setEnabled(True)
        self.btn_refresh.setEnabled(True)

    # --- Write Transaction: Write -> Verify Pattern ---
    def start_write_transaction(self, name, value_str):
        """Start write transaction: validate, write, then verify"""
        # 1. Check parameter definition
        param = self.registry.get_by_name(name)
        if not param:
            return

        # 2. Validate value range
        try:
            if param.data_type == ConfigType.INT:
                val = int(float(value_str))
            else:
                val = float(value_str)
            
            if val < param.min_value or val > param.max_value:
                QMessageBox.warning(self, "Limit Error", f"Value {val} is out of range ({param.min_value} ~ {param.max_value})")
                return
        except ValueError:
            QMessageBox.warning(self, "Format Error", "Invalid number format.")
            return

        # 3. Lock UI during transaction
        self.param_widget.set_all_enabled(False)
        self.status_label.setText(f"Writing {name}...")
        
        # 4. Create transaction state machine
        self.pending_write = {
            'name': name,
            'target_val': val,
            'param': param,
            'state': 'WAIT_WRITE_ACK'  # First state: waiting for write acknowledgment
        }

        # 5. Send WRITE command
        self.log(f"Writing {name} = {val}", "INFO")
        self.send_config_cmd(ConfigMode.WRITE, param.data_type, param.address, val)
        self.tx_timer.start(500)  # Allow more time for write operation

    def handle_write_step(self, msg):
        """State machine for write transaction: WRITE_ACK -> READ_VERIFY"""
        tx = self.pending_write
        param = tx['param']
        
        # Verify this message is for the current write operation
        if ConfigType(msg.type) != param.data_type or msg.address != param.address:
            return 

        if tx['state'] == 'WAIT_WRITE_ACK':
            # Received write acknowledgment, now verify with read
            self.log(f"Write Ack received. Verifying...", "DEBUG")
            tx['state'] = 'WAIT_READ_VERIFY'
            
            # Send READ command to verify the written value
            self.send_config_cmd(ConfigMode.READ, param.data_type, param.address, 0)
            self.tx_timer.start(500)
            
        elif tx['state'] == 'WAIT_READ_VERIFY':
            # Received read response, compare with target value
            received_val = msg.value_i if param.data_type == ConfigType.INT else msg.value_f
            target_val = tx['target_val']
            
            # Check if values match (with tolerance for floats)
            is_match = False
            if param.data_type == ConfigType.INT:
                is_match = (received_val == int(target_val))
            else:
                is_match = abs(received_val - float(target_val)) < 0.001
            
            if is_match:
                # Write successful and verified
                self.log(f"Write Success: {param.name} confirmed as {received_val}", "SUCCESS")
                self.param_widget.update_field_from_motor(param.name, received_val)
                self.end_transaction(success=True)
            else:
                # Verification failed - value mismatch
                self.log(f"Verification Failed! Expected {target_val}, got {received_val}", "ERROR")
                QMessageBox.critical(self, "Write Failed", f"Verification failed.\nTarget: {target_val}\nActual: {received_val}")
                self.end_transaction(success=False)

    def end_transaction(self, success):
        """Clean up and unlock UI after transaction completes"""
        self.pending_write = None
        self.tx_timer.stop()
        self.status_label.setText("Ready")
        self.param_widget.set_all_enabled(True)

    # --- Communication Core ---
    def send_config_cmd(self, mode, c_type, addr, val):
        """Build and send config command message"""
        self.seq_counter += 1
        
        msg = ConfigStamped()
        msg.header.seq = self.seq_counter
        msg.header.stamp = self.worker.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.transmit = True  # Flag to transmit to motor controller
        msg.module = int(self.current_module)
        msg.motor = int(self.current_motor)
        msg.mode = int(mode)  # READ or WRITE
        msg.type = int(c_type)  # INT or FLOAT
        msg.address = int(addr)  # Parameter address
        msg.error_code = 0
        
        # Set value based on data type
        if c_type == ConfigType.INT:
            msg.value_i = int(val)
            msg.value_f = 0.0
        else:
            msg.value_i = 0
            msg.value_f = float(val)
            
        self.worker.send_cmd(msg)

    def handle_ros_msg(self, msg):
        """Main ROS message callback - route to read queue or write transaction"""
        
        # Debug: Log all incoming messages
        self.log(f"Received: seq={msg.header.seq} (expect {self.seq_counter}), type={msg.type}, addr={msg.address}, val_i={msg.value_i}, val_f={msg.value_f}", "DEBUG")
        
        # Priority 1: Handle read queue (scanning parameters)
        if self.load_queue:
            req_type, req_addr = self.load_queue[0]
            
            # Check if Type/Addr match (relaxed SEQ check for reliability)
            if msg.type == int(req_type) and msg.address == req_addr:
               
                self.tx_timer.stop()  # Cancel timeout
                self.load_queue.popleft()  # Remove completed task
                
                # Lookup parameter and update UI
                param = self.registry.get_by_type_and_address(req_type, req_addr)
                if param:
                    val = msg.value_i if req_type == ConfigType.INT else msg.value_f
                    self.param_widget.update_field_from_motor(param.name, val)
                    self.log(f"✓ Read {param.name} = {val}", "SUCCESS")
                    self.successful_reads += 1  # Track successful reads
                else:
                    self.log(f"Warning: No param for Type={req_type} Addr={req_addr}", "WARN")
                
                # Process next item in queue
                self.process_load_queue()
            return

        # Priority 2: Handle write transaction
        if self.pending_write:
            # Check if this message is for current write operation
            param = self.pending_write['param']
            if msg.type == param.data_type and msg.address == param.address:
                self.tx_timer.stop()
                self.handle_write_step(msg)
            return

    def handle_timeout(self):
        """Handle communication timeout - retry or skip"""
        if self.load_queue:
            # Read timeout - retry or skip
            if self.retry_count < self.max_retries:
                # Retry the current read
                self.retry_count += 1
                c_type, addr = self.load_queue[0]
                self.log(f"Timeout reading Type {c_type.name} Addr {addr}. Retry {self.retry_count}/{self.max_retries}", "WARN")
                # Resend current request
                self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
                self.tx_timer.start(500)
            else:
                # Max retries exceeded - skip this parameter
                skipped = self.load_queue.popleft()
                self.log(f"Failed to read Type {skipped[0].name} Addr {skipped[1]} after {self.max_retries} retries. Skipping.", "ERROR")
                self.retry_count = 0
                self.process_load_queue()
            
        elif self.pending_write:
            # Write timeout - abort transaction
            self.log("Write operation timed out.", "ERROR")
            QMessageBox.critical(self, "Timeout", "Motor did not respond.")
            self.end_transaction(success=False)
    
    def send_rest_command(self):
        """Send REST command (Robot Mode = SYSTEM_ON) and close window"""
        self.robot_cmd_seq += 1
        
        # Build robot command message
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self.robot_cmd_seq
        robot_cmd.header.stamp = self.worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(RobotMode.SYSTEM_ON)  # Request SYSTEM_ON state
        
        self.worker.send_robot_cmd(robot_cmd)
        self.log("Sent REST command (Robot Mode = SYSTEM_ON)", "INFO")
        
        # Close config panel after sending command
        self.close()
    
    def closeEvent(self, event):
        """Handle window close event - allow reopening from control panel"""
        self.log("Config Panel closing...", "INFO")
        event.accept()  # Accept close event

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())