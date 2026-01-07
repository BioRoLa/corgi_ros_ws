#!/usr/bin/env python3
import os
import sys
import threading
import time
import yaml
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
from std_msgs.msg import Header

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

class ConfigType(IntEnum):
    INT = 0
    FLOAT = 1

# --- Parameter Manager ---
class ParameterManager:
    """
    僅負責查詢參數定義 (Address, Type, Limits, Writable)。
    完全不處理數值初始化 (Init)，數值來源僅限於 Motor。
    """
    def __init__(self, yaml_file='motor_parameters.yaml'):
        self.params_by_name = {}
        self.params_by_addr = {} # Key: (type, address)
        self.groups = {}
        self.load_yaml(yaml_file)

    def load_yaml(self, yaml_file):
        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                
            for p in data.get('parameters', []):
                # 判定型態
                c_type = ConfigType.INT if p['DataType'] == 'int' else ConfigType.FLOAT
                addr = p['Address']
                name = p['Variable']
                
                param_obj = {
                    'name': name,
                    'addr': addr,
                    'type': c_type,
                    'group': p.get('Group', 'Unassigned'),
                    # 處理 Min/Max，若為空字串則設為無限大
                    'min': float(p.get('Min')) if p.get('Min') != "" else -float('inf'),
                    'max': float(p.get('Max')) if p.get('Max') != "" else float('inf'),
                    'desc': p.get('Desc', ''),
                    'unit': p.get('Unit', ''),
                    'writable': bool(p.get('Writeable', 0))
                }
                
                self.params_by_name[name] = param_obj
                self.params_by_addr[(c_type, addr)] = param_obj
                
                if param_obj['group'] not in self.groups:
                    self.groups[param_obj['group']] = []
                self.groups[param_obj['group']].append(param_obj)
                
        except Exception as e:
            print(f"Error loading YAML: {e}")

    def get_by_addr(self, c_type, addr):
        return self.params_by_addr.get((c_type, addr))

    def get_by_name(self, name):
        return self.params_by_name.get(name)

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

    def __init__(self, param_manager):
        super().__init__()
        self.pm = param_manager
        self.inputs = {} # name -> QLineEdit
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        # 排列順序
        order = ["System", "Control", "Limits", "Motor", "Reserved"]
        sorted_groups = sorted(self.pm.groups.keys(), key=lambda x: order.index(x) if x in order else 99)

        for group in sorted_groups:
            page = QWidget()
            form = QFormLayout(page)
            form.setLabelAlignment(Qt.AlignRight)
            
            for param in self.pm.groups[group]:
                name = param['name']
                
                # 建立輸入框
                inp = QLineEdit()
                inp.setPlaceholderText("Wait...") # 初始顯示等待
                inp.setEnabled(False) # 初始鎖定，直到讀取到數值
                
                # 樣式設定
                if not param['writable']:
                    # 唯讀參數 (灰色)
                    inp.setStyleSheet("background-color: #333; color: #aaa; border: 1px solid #444;")
                else:
                    # 可寫參數 (按下 Enter 觸發寫入)
                    inp.returnPressed.connect(lambda n=name: self.handle_enter(n))
                    # 初始狀態也是鎖定 (深色)，直到收到數據才解鎖
                    inp.setStyleSheet("background-color: #2a2a2a; color: #fff; border: 1px solid #555;")
                
                self.inputs[name] = inp
                
                # Label
                label_text = f"{param['desc']} ({name})"
                if param['unit']:
                    label_text += f" [{param['unit']}]"
                
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
            
            # 只有當該參數在 YAML 定義為可寫，才解鎖讓使用者修改
            param_def = self.pm.get_by_name(name)
            if param_def and param_def['writable']:
                inp.setEnabled(True)
                inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")

    def set_all_enabled(self, enabled):
        """
        全域鎖定/解鎖。
        在讀取或寫入交易過程中，鎖定所有欄位防止干擾。
        """
        for name, inp in self.inputs.items():
            param_def = self.pm.get_by_name(name)
            # 只有原本就是可寫的參數才受此控制
            if param_def and param_def['writable']:
                inp.setEnabled(enabled)
                if not enabled:
                     inp.setStyleSheet("background-color: #2a2a2a; color: #aaa; border: 1px solid #555;")
                else:
                     inp.setStyleSheet("background-color: #404040; color: #fff; border: 1px solid #555;")

# --- Main Window ---
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        # 檢查 yaml 是否存在
        if not os.path.exists('motor_parameters.yaml'):
            QMessageBox.critical(self, "Error", "motor_parameters.yaml not found!")
            sys.exit(1)

        self.pm = ParameterManager('motor_parameters.yaml')
        self.worker = RosWorker()
        self.seq_counter = 0
        
        # State
        self.current_module = None
        self.current_motor = None
        
        # Queue & Transaction
        self.load_queue = deque() # 存放 (Type, Address)
        self.pending_write = None # 存放正在進行的寫入交易資訊
        
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
        
        self.param_widget = ParameterTab(self.pm)
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
        
        # 更新進度條
        done = self.total_load_items - len(self.load_queue)
        self.progress.setValue(int((done / self.total_load_items) * 100))

        # 發送讀取命令 (value_f=0, value_i=0)
        self.send_config_cmd(ConfigMode.READ, c_type, addr, 0)
        
        # 設定 200ms 超時
        self.tx_timer.start(200)

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
        param_def = self.pm.get_by_name(name)
        if not param_def:
            return

        # 2. 驗證數值範圍 (依照 YAML 定義的 Min/Max)
        try:
            if param_def['type'] == ConfigType.INT:
                val = int(float(value_str))
            else:
                val = float(value_str)
            
            if val < param_def['min'] or val > param_def['max']:
                QMessageBox.warning(self, "Limit Error", f"Value {val} is out of range ({param_def['min']} ~ {param_def['max']})")
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
            'def': param_def,
            'state': 'WAIT_WRITE_ACK'
        }

        # 5. 發送 WRITE 指令
        self.log(f"Writing {name} = {val}", "INFO")
        self.send_config_cmd(ConfigMode.WRITE, param_def['type'], param_def['addr'], val)
        self.tx_timer.start(500) # 寫入給多一點時間

    def handle_write_step(self, msg):
        """處理寫入流程的狀態機"""
        tx = self.pending_write
        param_def = tx['def']
        
        # 檢查是否為當前操作的參數回傳
        if ConfigType(msg.type) != param_def['type'] or msg.address != param_def['addr']:
            return 

        if tx['state'] == 'WAIT_WRITE_ACK':
            # 收到 Write 的 Echo，發送 Read 進行 Double Check
            self.log(f"Write Ack received. Verifying...", "DEBUG")
            tx['state'] = 'WAIT_READ_VERIFY'
            
            # 發送 Read 指令
            self.send_config_cmd(ConfigMode.READ, param_def['type'], param_def['addr'], 0)
            self.tx_timer.start(500)
            
        elif tx['state'] == 'WAIT_READ_VERIFY':
            # 收到 Read 的結果，比對數值
            received_val = msg.value_i if param_def['type'] == ConfigType.INT else msg.value_f
            target_val = tx['target_val']
            
            is_match = False
            if param_def['type'] == ConfigType.INT:
                is_match = (received_val == int(target_val))
            else:
                is_match = abs(received_val - float(target_val)) < 0.001
            
            if is_match:
                self.log(f"Write Success: {param_def['name']} confirmed as {received_val}", "SUCCESS")
                self.param_widget.update_field_from_motor(param_def['name'], received_val)
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
        msg.header = Header()
        msg.header.seq = self.seq_counter
        msg.header.stamp = self.worker.node.get_clock().now().to_msg()
        
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
        
        # 1. 處理讀取隊列 (Scanning)
        if self.load_queue:
            req_type, req_addr = self.load_queue[0]
            
            # 檢查 SEQ 以及 Type/Addr 是否匹配
            if msg.header.seq == self.seq_counter and \
               msg.type == int(req_type) and \
               msg.address == req_addr:
               
                self.tx_timer.stop()
                self.load_queue.popleft() # 移除已完成的任務
                
                # 查表 YAML 更新 UI
                param_def = self.pm.get_by_addr(ConfigType(msg.type), msg.address)
                if param_def:
                    val = msg.value_i if msg.type == int(ConfigType.INT) else msg.value_f
                    self.param_widget.update_field_from_motor(param_def['name'], val)
                
                # 繼續下一個
                self.process_load_queue()
            return

        # 2. 處理寫入交易 (Write Transaction)
        if self.pending_write:
            if msg.header.seq == self.seq_counter:
                self.tx_timer.stop()
                self.handle_write_step(msg)
            return

    def handle_timeout(self):
        if self.load_queue:
            # 讀取超時，跳過該參數
            skipped = self.load_queue.popleft()
            self.log(f"Timeout reading Type {skipped[0].name} Addr {skipped[1]}. Skipping.", "WARN")
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