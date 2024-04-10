from typing import Callable, Optional
from rove_control_board.capra_micro_comm import BinaryData, SerialCommandManager, Void, CommandManager

manager = SerialCommandManager()

class Vector2D(BinaryData):
    def __init__(self, x:float=0, y:float=0):
        super().__init__('ff', x=x, y=y)

class State(BinaryData):
    def __init__(self, state:bool=False):
        super().__init__('B', state=state)

class Status(BinaryData):
    def __init__(self, statusCode:int=0):
        super().__init__('I', statusCode=statusCode)

@manager.binaryFunction(Void, Status)
def ledOn():
    print("on")

@manager.binaryFunction(Void, Status)
def ledOff():
    print("off")

@manager.binaryFunction(State, Status)
def setLedState(state:State):
    print('on' if state.state else 'off')

@ledOn.postCall
@ledOff.postCall
@setLedState.postCall
def ledStatus(status:Status):
    print(f"code: {status.statusCode}")


@manager.binaryFunction(State, State)
def loopback(state:State) -> State:
    pass

@loopback.preCall
def preLoop(state:State):
    print(f"Sending {state.state}")
    
@loopback.postCall
def postLoop(state:State):
    print(f"Recieving {state.state}")