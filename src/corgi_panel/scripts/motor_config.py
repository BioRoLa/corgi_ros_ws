#!/usr/bin/env python3
"""
Motor parameter configuration
Defines all motor parameters with their properties
"""
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class ConfigType(IntEnum):
    INT = 0
    FLOAT = 1


@dataclass
class MotorParameter:
    """Definition of a single motor parameter"""
    name: str
    address: int
    data_type: ConfigType
    group: str
    writable: bool
    min_value: float
    max_value: float
    description: str = ""
    unit: str = ""
    init_value: Optional[float] = None


class MotorParameterRegistry:
    """Central registry of all motor parameters"""
    
    def __init__(self):
        self.params_by_name = {}
        self.params_by_addr = {}  # Key: (type, address)
        self.groups = {}
        self._init_parameters()
    
    def _init_parameters(self):
        """Initialize all motor parameters"""
        params = [
            # System Group
            MotorParameter("CAN_ID", 1, ConfigType.INT, "System", True, 0, 127, 
                          "CAN bus ID of the motor", "", 1),
            MotorParameter("CAN_MASTER", 2, ConfigType.INT, "System", True, 0, 127,
                          "CAN bus Master ID", "", 0),
            MotorParameter("CAN_TIMEOUT", 3, ConfigType.INT, "System", True, 0, 2147483647,
                          "CAN timeout period (0 = none)", "(cycles)", 0),
            MotorParameter("R_PHASE", 13, ConfigType.FLOAT, "System", False, 0, 2147483647,
                          "", "", None),
            
            # Control Group
            MotorParameter("I_BW", 2, ConfigType.FLOAT, "Control", True, 100, 2000,
                          "Current bandwidth", "(Hz)", 1000),
            MotorParameter("KP_MAX", 25, ConfigType.FLOAT, "Control", True, 0, 1000,
                          "Maximum Position P-gain value", "", 500),
            MotorParameter("KI_MAX", 26, ConfigType.FLOAT, "Control", True, 0, 10,
                          "Maximum Position Integral I-gain value", "", 0),
            MotorParameter("KD_MAX", 27, ConfigType.FLOAT, "Control", True, 0, 5,
                          "Maximum velocity D-gain value", "", 5),
            
            # Limits Group
            MotorParameter("I_MAX", 3, ConfigType.FLOAT, "Limits", True, 0.0, 75.0,
                          "Maximum current", "(A)", 40.0),
            MotorParameter("I_FW_MAX", 6, ConfigType.FLOAT, "Limits", True, 0.0, 33.0,
                          "Maximum flux weakening current", "(A)", 0.0),
            MotorParameter("P_MIN", 19, ConfigType.FLOAT, "Limits", True, -2147483647.0, 0.0,
                          "Minimum position setpoint", "(rad)", 0.0),
            MotorParameter("P_MAX", 20, ConfigType.FLOAT, "Limits", True, 0.0, 2147483647.0,
                          "Maximum position setpoint", "(rad)", 6.283),
            MotorParameter("V_MIN", 21, ConfigType.FLOAT, "Limits", True, -2147483647.0, 0.0,
                          "Minimum velocity setpoint", "(rad/s)", -45.0),
            MotorParameter("V_MAX", 22, ConfigType.FLOAT, "Limits", True, 0.0, 2147483647.0,
                          "Maximum velocity setpoint", "(rad/s)", 45.0),
            MotorParameter("T_MIN", 23, ConfigType.FLOAT, "Limits", True, -2147483647.0, 0.0,
                          "Minimum torque setpoint", "(Nm)", -20.0),
            MotorParameter("T_MAX", 24, ConfigType.FLOAT, "Limits", True, 0.0, 2147483647.0,
                          "Maximum torque setpoint", "(Nm)", 20.0),
            MotorParameter("TEMP_MAX", 8, ConfigType.FLOAT, "Limits", False, float('-inf'), float('inf'),
                          "", "", 125),
            MotorParameter("I_MAX_CONT", 9, ConfigType.FLOAT, "Limits", False, float('-inf'), float('inf'),
                          "Continuous maximum current", "(A)", 14),
            
            # Motor Group
            MotorParameter("PPAIRS", 10, ConfigType.FLOAT, "Motor", False, float('-inf'), float('inf'),
                          "", "", 21),
            MotorParameter("KT", 14, ConfigType.FLOAT, "Motor", True, 0.0001, 2147483647.0,
                          "Motor torque constant", "(Nm/A)", 0.08),
            MotorParameter("GR", 17, ConfigType.FLOAT, "Motor", True, 0.001, 2147483647.0,
                          "Gear ratio", "", 6.0),
            MotorParameter("I_CAL", 18, ConfigType.FLOAT, "Motor", True, 0.0, 20.0,
                          "Calibration current", "(A)", 5.0),
            MotorParameter("HALL_CAL_OFFSET", 28, ConfigType.FLOAT, "Motor", True, 0, 143,
                          "Hall calibration offset", "", 0),
            MotorParameter("HALL_CAL_SPEED", 29, ConfigType.FLOAT, "Motor", True, 0.0, 10.0,
                          "Hall calibration speed", "(rad/s)", 0.2),
            MotorParameter("PHASE_ORDER", 0, ConfigType.INT, "Motor", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("M_ZERO", 4, ConfigType.INT, "Motor", False, float('-inf'), float('inf'),
                          "", "", 0),
            MotorParameter("E_ZERO", 5, ConfigType.INT, "Motor", False, float('-inf'), float('inf'),
                          "", "", 0),
            MotorParameter("HALL_CAL_DIR", 6, ConfigType.INT, "Motor", True, -1, 1,
                          "Hall calibration direction", "", 1),
            MotorParameter("ENCODER_LUT", 7, ConfigType.INT, "Motor", False, float('-inf'), float('inf'),
                          "", "", None),
            
            # Reserved Group
            MotorParameter("Reserved0", 0, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("Reserved1", 1, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("THETA_MIN", 4, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("THETA_MAX", 5, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("R_NOMINAL", 7, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", 0),
            MotorParameter("Reserved11", 11, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("Reserved12", 12, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("R_TH", 15, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
            MotorParameter("C_TH", 16, ConfigType.FLOAT, "Reserved", False, float('-inf'), float('inf'),
                          "", "", None),
        ]
        
        for param in params:
            self.params_by_name[param.name] = param
            key = (param.data_type, param.address)
            self.params_by_addr[key] = param
            
            if param.group not in self.groups:
                self.groups[param.group] = []
            self.groups[param.group].append(param)
    
    def get_by_name(self, name: str) -> Optional[MotorParameter]:
        """Get parameter by name"""
        return self.params_by_name.get(name)
    
    def get_by_addr(self, config_type: ConfigType, address: int) -> Optional[MotorParameter]:
        """Get parameter by type and address"""
        return self.params_by_addr.get((config_type, address))
    
    def get_by_type_and_address(self, config_type: ConfigType, address: int) -> Optional[MotorParameter]:
        """Get parameter by type and address (alias for get_by_addr)"""
        return self.get_by_addr(config_type, address)
    
    def get_group(self, group_name: str) -> list:
        """Get all parameters in a group"""
        return self.groups.get(group_name, [])
    
    def get_all_groups(self) -> list:
        """Get all group names"""
        return list(self.groups.keys())
