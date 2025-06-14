from time import sleep
import pytest
from rove_control_board.control_board_bridge import ServoMock

def test_move_pos():
    motor = ServoMock(500)
    assert motor.actual_position == 0
    motor.move_pos(400)
    for _ in range(100):
        sleep(0.01)
        motor.update()
    assert motor.actual_position == 400
    
