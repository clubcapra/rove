from enum import Enum
from functools import wraps
from typing import Callable, TypeVar
from odrive_can.enums import *
from odrive_can.utils import *

class CmdID(Enum):
	Get_Version = 0x000
	Heartbeat = 0x001
	Estop = 0x002
	Get_Error = 0x003
	RxSdo = 0x004
	TxSdo = 0x005
	Address = 0x006
	Set_Axis_State = 0x007
	Get_Encoder_Estimates = 0x009
	Set_Controller_Mode = 0x00b
	Set_Input_Pos = 0x00c
	Set_Input_Vel = 0x00d
	Set_Input_Torque = 0x00e
	Set_Limits = 0x00f
	Set_Traj_Vel_Limit = 0x011
	Set_Traj_Accel_Limits = 0x012
	Set_Traj_Inertia = 0x013
	Get_Iq = 0x014
	Get_Temperature = 0x015
	Reboot = 0x016
	Get_Bus_Voltage_Current = 0x017
	Clear_Errors = 0x018
	Set_Absolute_Position = 0x019
	Set_Pos_Gain = 0x01a
	Set_Vel_Gains = 0x01b
	Get_Torques = 0x01c
	Get_Powers = 0x01d
	Enter_DFU_Mode = 0x01f


CommandT = TypeVar('CommandT', Callable)


