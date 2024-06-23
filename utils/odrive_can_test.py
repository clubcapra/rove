import asyncio
from odrive_can import ODriveCAN, CanMsg
from odrive_can.tools import UDP_Client

AXIS_ID = 0
INTERFACE = "can0"
SETPOINT = 50

udp = UDP_Client()  # send data to UDP server for plotting

def feedback_callback_fcn(msg: CanMsg, caller: ODriveCAN):
    """called on position estimate"""
    print(msg)
    udp.send(msg.data)

async def main():
    """connect to odrive"""
    try:
        drv = ODriveCAN(axis_id=AXIS_ID, interface=INTERFACE)

        # set up callback (optional)
        drv.feedback_callback = feedback_callback_fcn

        # start
        await drv.start()

        # check errors (raises exception if any)
        await drv.check_errors()  # Ensure this is awaited

        # set controller mode
        await drv.set_controller_mode("POSITION_CONTROL", "POS_FILTER")

        # reset encoder
        await drv.set_linear_count(0)

        # set axis state
        await drv.set_axis_state("CLOSED_LOOP_CONTROL")

        # set position gain
        await drv.set_pos_gain(3.0)

        for _ in range(2):
            # setpoint
            await drv.set_input_pos(SETPOINT)
            await asyncio.sleep(5.0)
            await drv.set_input_pos(-SETPOINT)
            await asyncio.sleep(5.0)

        await drv.set_input_pos(0.0)
    except Exception as e:
        print(f"Error in main function: {e}")

asyncio.run(main())
