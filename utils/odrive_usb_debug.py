import odrive
import time
import math
from odrive.enums import *


odrv0 = odrive.find_any()
odrv = odrv0

# odrv.axis0.controller.config.spinout_electrical_power_threshold = 1000
# odrv.axis0.controller.config.spinout_mechanical_power_threshold = -1000
# odrv.axis0.config.can.error_msg_rate_ms = 100
# odrv.axis0.config.can.node_id = 21
# odrv.axis0.config.enable_watchdog = True

print(odrv.axis0.config.can.node_id)
print(odrv.axis0.config.enable_watchdog)
print(odrv.axis0.config.watchdog_timeout)
print(odrive.utils.dump_errors(odrv0))
print(odrv.axis0.disarm_reason)
print(odrv.axis0.active_errors)

# odrv.axis0.config.can.node_id = 0
# odrv.save_configuration()
