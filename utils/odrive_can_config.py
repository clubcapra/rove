"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""

import asyncio
from pathlib import Path
import threading
from typing import Optional
import can
import struct
import odrive_can
import odrive_can.examples
import odrive_can.linear_model
import odrive.config
import odrive_can_api
import odrive_can_api.dict_editor

basepath = Path(__file__).parent / 'config'

sample_data = {
    "name": "Alice",
    "age": 25,
    "location": {"city": "Wonderland", "country": "Fantasy"},
    "hobbies": ["reading", "chess", "exploring"]
}

editor = odrive_can_api.dict_editor.DictionaryEditor(sample_data, basepath=basepath)
editor.run()


node_id = 0 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
            break

# Set velocity to 1.0 turns/s
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', 1.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False
))

# Print encoder feedback
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")