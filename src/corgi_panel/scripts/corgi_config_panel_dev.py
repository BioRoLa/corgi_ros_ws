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
                             QProgressBar)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject

from corgi_msgs.msg import ConfigStamped

from motor_config import MotorParameterRegistry, ConfigType

# --- Enums (Matching Proto) ---
class Module(IntEnum):
    MODULE_A = 0
    MODULE_B = 1
    MODULE_C = 2
    MODULE_D = 3

class Motor(IntEnum):
    MOTOR_R = 0
    MOTOR_L = 1

class ConfigMode(IntEnum):
    READ = 0
    WRITE = 1

# --- ROS Worker ---
class RosWorker(QObject):
    msg_received = pyqtSignal(ConfigStamped)

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub = None
        self.sub = None

    def start_ros(self):
        rclpy.init()
        self.node = rclpy.create_node('corgi_config_panel')
        
        # Pub: config/command, Sub: config/state
        self.pub = self.node.create_publisher(ConfigStamped, 'config/command', 10)
        self.sub = self.node.create_subscription(ConfigStamped, 'config/state', self.msg_cb, 10)
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        thread = threading.Thread(target=self.executor.spin, daemon=True)
        thread.start()

    def msg_cb(self, msg):
        self.msg_received.emit(msg)

    def send_cmd(self, msg):
        if self.pub:
            self.pub.publish(msg)

# --- UI Tab Component ---
class ParameterTab(QWidget):
    request_write = pyqtSignal(str, object) # name, value

    def __init__(self, param_registry):
        super().__init__()
        self.registry = param_registry
        self.inputs = {} # name -> QLineEdit
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        # 排列順序
        order = ["System", "Control", "Limits", "Motor", "Reserved"]
        sorted_groups = sorted(self.registry.get_all_groups(), key=lambda x: order.index(x) if x in order else 99)

        for group in sorted_groups:
            page = QWidget()
            form = QFormLayout(page)
            form.setLabelAlignment(Qt.AlignRight)
            
            for param in self.registry.get_group(group):
                name = param.name
                
                # 建立輸入框
                inp = QLineEdit()
                inp.setPlaceholderText("Wait...") # 初始顯示等待
                inp.setEnabled(False) # 初始鎖定，直到讀取到數值
                
                # 樣式設定
                if not param.writable:
                    # 唯讀參數 (灰色)
                    inp.setStyleSheet("background-color: #333; color: #aaa; border: 1px solid #444;")
                else:
                    # 可寫參數 (按下 Enter 觸發寫入)
                    inp.returnPressed.connect(lambda n=name: self.handle_enter(n))
                    # 初始狀態也是鎖定 (深色)，直到收到數據才解鎖
                    inp.setStyleSheet("background-color: #2a2a2a; color: #fff; border: 1px solid #555;")
                
                self.inputs[name] = inp
                
                # Label
                label_text = f"{param.description} ({name})" if param.description else name
                if param.unit:
                    label_text += f" [{param.unit}]"
                
                form.addRow(label_text, inp)
            
            self.tabs.addTab(page, group)
        layout.addWidget(self.tabs)

    def handle_enter(self, name):
        # 取得使用者輸入的值並發出請求
        val = self.inputs[name].text()
        self.request_write.emit(name, val)

    def update_field_from_motor(self, name, value):
        """當從馬達收到數值時呼叫"""
        if name in self.inputs:
            inp = self.inputs[name]
            inp.setText(str(value))
            
            # 只有當該參數定義為可寫，才解鎖讓使用者修改
            param = self.registry.get_by_name(name)
            if param and param.writable:
                inp.setEnabled(True)
                inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")

    def set_all_enabled(self, enabled):
        """
        全域鎖定/解鎖。
        在讀取或寫入交易過程中，鎖定所有欄位防止干擾。
        """
        for name, inp in self.inputs.items():
            param = self.registry.get_by_name(name)
            # 只有原本就是可寫的參數才受此控制
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
        
        # State
        self.current_module = None
        self.current_motor = None
        
        # Queue & Transaction
        self.load_queue = deque() # 存放 (Type, Address)
        self.pending_write = None # 存放正在進行的寫入交易資訊
        self.retry_count = 0 # 重試計數器
        self.max_retries = 3 # 最大重試次數
        
        self.init_ui()
        self.init_ros()
        
        # Timer 用於超時重試
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
        self.cb_motor.addItems(["- Select -", "Motor R", "Motor L"])
        self.cb_motor.setEnabled(False)
        self.cb_motor.currentIndexChanged.connect(self.on_motor_selected)
        
        sel_layout.addRow("Module:", self.cb_module)
        sel_layout.addRow("Motor:", self.cb_motor)
        left_panel.addWidget(sel_group)
        
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
        self.worker.msg_received.connect(self.handle_ros_msg)
        self.worker.start_ros()
        self.log("System Ready.", "SYSTEM")

    def log(self, msg, level="INFO"):
        color = "#00e676" if level in ["INFO", "SUCCESS"] else "#ff5252" if level == "ERROR" else "#2196F3"
        timestamp = time.strftime("%H:%M:%S")
        self.log_view.append(f'<span style="color:#888">[{timestamp}]</span> <span style="color:{color}"><b>[{level}]</b></span> {msg}')

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

    # --- Read All Sequence (Scan) ---
    def start_load_sequence(self):
        """
        開始讀取流程：
        1. INT (Type 0): Address 0~7
        2. FLOAT (Type 1): Address 0~29
        """
        self.load_queue.clear()
        
        # 鎖定 UI，顯示進度條
        self.param_widget.set_all_enabled(False)
        self.cb_module.setEnabled(False)
        self.cb_motor.setEnabled(False)
        self.progress.setVisible(True)
        self.progress.setValue(0)
        self.status_label.setText("Reading parameters from motor...")
        
        # 建立讀取清單
        for addr in range(8):
            self.load_queue.append((ConfigType.INT, addr))
        for addr in range(30):
            self.load_queue.append((ConfigType.FLOAT, addr))
            
        self.total_load_items = len(self.load_queue)
        self.process_load_queue()

    def process_load_queue(self):
        if not self.load_queue:
            # 全部讀取完成
            self.finish_loading()
            return

        c_type, addr = self.load_queue[0]
        self.retry_count = 0  # 重置重試計數
        
        # 更新進度條
        done = self.total_load_items - len(self.load_queue)
        self.progress.setValue(int((done / self.total_load_items) * 100))

        # 發送讀取命令 (value_f=0, value_i=0)
        self.log(f"Reading Type={c_type.name} Addr={addr}", "DEBUG")
        self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
        
        # 設定 500ms 超時 (給更長時間)
        self.tx_timer.start(500)

    def finish_loading(self):
        self.status_label.setText(f"Connected: Module {Module(self.current_module).name}, Motor {Motor(self.current_motor).name}")
        self.status_label.setStyleSheet("color: #00e676; font-weight: bold;")
        self.progress.setVisible(False)
        self.cb_module.setEnabled(True)
        self.cb_motor.setEnabled(True)
        self.log("All parameters read complete.", "SUCCESS")

    # --- Write Transaction (Write -> Verify) ---
    def start_write_transaction(self, name, value_str):
        # 1. 檢查參數定義
        param = self.registry.get_by_name(name)
        if not param:
            return

        # 2. 驗證數值範圍
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

        # 3. 鎖定所有 UI
        self.param_widget.set_all_enabled(False)
        self.status_label.setText(f"Writing {name}...")
        
        # 4. 建立交易狀態
        self.pending_write = {
            'name': name,
            'target_val': val,
            'param': param,
            'state': 'WAIT_WRITE_ACK'
        }

        # 5. 發送 WRITE 指令
        self.log(f"Writing {name} = {val}", "INFO")
        self.send_config_cmd(ConfigMode.WRITE, param.data_type, param.address, val)
        self.tx_timer.start(500) # 寫入給多一點時間

    def handle_write_step(self, msg):
        """處理寫入流程的狀態機"""
        tx = self.pending_write
        param = tx['param']
        
        # 檢查是否為當前操作的參數回傳
        if ConfigType(msg.type) != param.data_type or msg.address != param.address:
            return 

        if tx['state'] == 'WAIT_WRITE_ACK':
            # 收到 Write 的 Echo，發送 Read 進行 Double Check
            self.log(f"Write Ack received. Verifying...", "DEBUG")
            tx['state'] = 'WAIT_READ_VERIFY'
            
            # 發送 Read 指令
            self.send_config_cmd(ConfigMode.READ, param.data_type, param.address, 0)
            self.tx_timer.start(500)
            
        elif tx['state'] == 'WAIT_READ_VERIFY':
            # 收到 Read 的結果，比對數值
            received_val = msg.value_i if param.data_type == ConfigType.INT else msg.value_f
            target_val = tx['target_val']
            
            is_match = False
            if param.data_type == ConfigType.INT:
                is_match = (received_val == int(target_val))
            else:
                is_match = abs(received_val - float(target_val)) < 0.001
            
            if is_match:
                self.log(f"Write Success: {param.name} confirmed as {received_val}", "SUCCESS")
                self.param_widget.update_field_from_motor(param.name, received_val)
                self.end_transaction(success=True)
            else:
                self.log(f"Verification Failed! Expected {target_val}, got {received_val}", "ERROR")
                QMessageBox.critical(self, "Write Failed", f"Verification failed.\nTarget: {target_val}\nActual: {received_val}")
                self.end_transaction(success=False)

    def end_transaction(self, success):
        self.pending_write = None
        self.tx_timer.stop()
        self.status_label.setText("Ready")
        self.param_widget.set_all_enabled(True)

    # --- Communication Core ---
    def send_config_cmd(self, mode, c_type, addr, val):
        self.seq_counter += 1
        
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

    def handle_ros_msg(self, msg):
        """ROS 回調主入口"""
        
        # Debug: Log all incoming messages
        self.log(f"Received: seq={msg.header.seq} (expect {self.seq_counter}), type={msg.type}, addr={msg.address}, val_i={msg.value_i}, val_f={msg.value_f}", "DEBUG")
        
        # 1. 處理讀取隊列 (Scanning)
        if self.load_queue:
            req_type, req_addr = self.load_queue[0]
            
            # 檢查 Type/Addr 是否匹配 (放寬 SEQ 檢查，只要 Type/Addr 對就接受)
            if msg.type == int(req_type) and msg.address == req_addr:
               
                self.tx_timer.stop()
                self.load_queue.popleft() # 移除已完成的任務
                
                # 查表更新 UI
                param = self.registry.get_by_type_and_address(req_type, req_addr)
                if param:
                    val = msg.value_i if req_type == ConfigType.INT else msg.value_f
                    self.param_widget.update_field_from_motor(param.name, val)
                    self.log(f"✓ Read {param.name} = {val}", "SUCCESS")
                else:
                    self.log(f"Warning: No param for Type={req_type} Addr={req_addr}", "WARN")
                
                # 繼續下一個
                self.process_load_queue()
            return

        # 2. 處理寫入交易 (Write Transaction)
        if self.pending_write:
            # Check if this message matches our pending write operation
            param = self.pending_write['param']
            if msg.type == param.data_type and msg.address == param.address:
                self.tx_timer.stop()
                self.handle_write_step(msg)
            return

    def handle_timeout(self):
        if self.load_queue:
            # 讀取超時，重試或跳過
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                c_type, addr = self.load_queue[0]
                self.log(f"Timeout reading Type {c_type.name} Addr {addr}. Retry {self.retry_count}/{self.max_retries}", "WARN")
                # 重新發送當前請求
                self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
                self.tx_timer.start(500)  # 給更長的超時時間
            else:
                # 超過重試次數，跳過
                skipped = self.load_queue.popleft()
                self.log(f"Failed to read Type {skipped[0].name} Addr {skipped[1]} after {self.max_retries} retries. Skipping.", "ERROR")
                self.retry_count = 0
                self.process_load_queue()
            
        elif self.pending_write:
            self.log("Write operation timed out.", "ERROR")
            QMessageBox.critical(self, "Timeout", "Motor did not respond.")
            self.end_transaction(success=False)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())