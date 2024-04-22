from enum import Enum, Flag
from typing import NoReturn
import capra_micro_comm_py as comm
from rove_control_board.canutils import CanBusCommandManager

# manager = comm.SerialCommandManager()
manager = CanBusCommandManager()

@manager.enum('I')
class StatusCode(Enum):
    STNone = 0
    
@manager.enum('I')
class ErrorCode(Enum):
    ERNone =            0b00000000
    ERAdapterNotInit =  0b00000001
    ERServoXNACK =      0b00000010
    ERServoYNACK =      0b00000100

@manager.enum("I")
class ServoControlMode(Enum):
    SCMNone = 0
    SCMPosition = 1
    SCMSpeed = 2

@manager.struct('ii')
class Vector2D(comm.BinaryData):
    def __init__(self, x:int=0, y:int=0):
        super().__init__(x=x, y=y)
        self.x:int
        self.y:int

@manager.struct('BBB')
class RGB(comm.BinaryData):
    def __init__(self, r:int=0,g:int=0,b:int=0):
        super().__init__(r=r,g=g,b=b)
        self.r:int
        self.g:int
        self.b:int
        
@manager.struct('_II')
class Report(comm.BinaryData):
    def __init__(self, pos:Vector2D=Vector2D(), statusCode:int=0, errorCode:int=0):
        super().__init__(pos=pos, statusCode=statusCode, errorCode=errorCode)
        self.pos:Vector2D
        self.statusCode:int
        self.errorCode:int

@manager.struct('ff')
class Bounds(comm.BinaryData):
    def __init__(self, lower:float=0, upper:float=0):
        super().__init__(lower=lower, upper=upper)
        self.lower:float
        self.upper:float
    
@manager.command(Vector2D, comm.Bool_)
def setServoPosition(pos:Vector2D) -> comm.Bool_:
    pass

@manager.command(comm.Void, Vector2D)
def getServoPosition() -> Vector2D:
    pass

@manager.command(comm.Void, comm.Bool_)
def setServoPositionZero() -> comm.Bool_:
    pass

@manager.command(Vector2D, comm.Bool_)
def setServoSpeed(speed:Vector2D) -> comm.Bool_:
    pass

@manager.command(comm.Void, Vector2D)
def getServoSpeed() -> Vector2D:
    pass

@manager.command(comm.Void, comm.Int)
def getServoPositionX() -> comm.Int:
    pass

@manager.command(comm.Void, comm.Int)
def getServoPositionY() -> comm.Int:
    pass

@manager.command(comm.Void, comm.Int)
def getServoSpeedX() -> comm.Int:
    pass

@manager.command(comm.Void, comm.Int)
def getServoSpeedY() -> comm.Int:
    pass

@manager.command(comm.Byte, comm.Bool_)
def setServoXAcc(acc:comm.Byte) -> comm.Bool_:
    pass

@manager.command(comm.Byte, comm.Bool_)
def setServoYAcc(acc:comm.Byte) -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Byte)
def getServoXAcc() -> comm.Byte:
    pass

@manager.command(comm.Void, comm.Byte)
def getServoYAcc() -> comm.Byte:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setLEDFront(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setLEDBack(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setLEDStrobe(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Bool_)
def getLEDFront() -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Bool_)
def getLEDBack() -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Bool_)
def getLEDStrobe() -> comm.Bool_:
    pass

@manager.command(comm.Void, Report)
def getReport() -> Report:
    pass



