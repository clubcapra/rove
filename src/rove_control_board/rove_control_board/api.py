from enum import Enum, Flag
from typing import NoReturn
import capra_micro_comm_py as comm
from rove_control_board.canutils import CanBusCommandManager, CanBusMockManager

# manager = comm.SerialCommandManager()
manager = CanBusCommandManager()
# manager = CanBusMockManager()

@manager.struct('BBBx')
class RGB(comm.BinaryData):
    def __init__(self, r:int=0,g:int=0,b:int=0):
        super().__init__(r=r,g=g,b=b)
        self.r:int
        self.g:int
        self.b:int
        
@manager.struct('_B')
class RGBLed(comm.BinaryData):
    def __init__(self, rgb:RGB=RGB(), index:int=0):
        super().__init__(rgb=rgb, index=index)
        self.rgb:RGB
        self.index:int
        
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

@manager.command(comm.Void, comm.Bool_)
def getGPIO1() -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setGPIO1(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Bool_)
def getGPIO2() -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setGPIO2(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Void, comm.Bool_)
def getGPIO3() -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setGPIO3(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Int, RGB)
def getRGBLed(index:comm.Int) -> RGB:
    pass

@manager.command(RGBLed, comm.Bool_)
def setRGBLed(led:RGBLed) -> comm.Bool_:
    pass

# from enum import Enum, Flag
# from typing import NoReturn
# import capra_micro_comm_py as comm
# from rove_control_board.canutils import CanBusCommandManager

# # manager = comm.SerialCommandManager()
# manager = CanBusCommandManager()

# @manager.enum('H')
# class StatusCode(Enum):
#     STNone = 0
    
# @manager.enum('H')
# class ErrorCode(Enum):
#     ERNone =            0b00000000
#     ERAdapterNotInit =  0b00000001
#     ERServoXNACK =      0b00000010
#     ERServoYNACK =      0b00000100

# @manager.enum("I")
# class ServoControlMode(Enum):
#     SCMNone = 0
#     SCMPosition = 1
#     SCMSpeed = 2

# @manager.struct('hh')
# class Vector2D(comm.BinaryData):
#     def __init__(self, x:int=0, y:int=0):
#         super().__init__(x=x, y=y)
#         self.x:int
#         self.y:int

# @manager.struct('BBB')
# class RGB(comm.BinaryData):
#     def __init__(self, r:int=0,g:int=0,b:int=0):
#         super().__init__(r=r,g=g,b=b)
#         self.r:int
#         self.g:int
#         self.b:int
        
# @manager.struct('_HH')
# class Report(comm.BinaryData):
#     def __init__(self, pos:Vector2D=Vector2D(), statusCode:int=0, errorCode:int=0):
#         super().__init__(pos=pos, statusCode=statusCode, errorCode=errorCode)
#         self.pos:Vector2D
#         self.statusCode:int
#         self.errorCode:int

# @manager.struct('ff')
# class Bounds(comm.BinaryData):
#     def __init__(self, lower:float=0, upper:float=0):
#         super().__init__(lower=lower, upper=upper)
#         self.lower:float
#         self.upper:float
    
# @manager.command(Vector2D, comm.Bool_)
# def setServoPosition(pos:Vector2D) -> comm.Bool_:
#     pass

# @manager.command(comm.Void, Vector2D)
# def getServoPosition() -> Vector2D:
#     pass

# @manager.command(comm.Void, comm.Bool_)
# def setServoPositionZero() -> comm.Bool_:
#     pass

# @manager.command(Vector2D, comm.Bool_)
# def setServoSpeed(speed:Vector2D) -> comm.Bool_:
#     pass

# @manager.command(comm.Void, Vector2D)
# def getServoSpeed() -> Vector2D:
#     pass

# @manager.command(comm.Void, comm.Short)
# def getServoPositionX() -> comm.Short:
#     pass

# @manager.command(comm.Void, comm.Short)
# def getServoPositionY() -> comm.Short:
#     pass

# @manager.command(comm.Void, comm.Short)
# def getServoSpeedX() -> comm.Short:
#     pass

# @manager.command(comm.Void, comm.Short)
# def getServoSpeedY() -> comm.Short:
#     pass

# @manager.command(comm.Byte, comm.Bool_)
# def setServoXAcc(acc:comm.Byte) -> comm.Bool_:
#     pass

# @manager.command(comm.Byte, comm.Bool_)
# def setServoYAcc(acc:comm.Byte) -> comm.Bool_:
#     pass

# @manager.command(comm.Void, comm.Byte)
# def getServoXAcc() -> comm.Byte:
#     pass

# @manager.command(comm.Void, comm.Byte)
# def getServoYAcc() -> comm.Byte:
#     pass

# @manager.command(comm.Bool_, comm.Bool_)
# def setLEDFront(state:comm.Bool_) -> comm.Bool_:
#     pass

# @manager.command(comm.Bool_, comm.Bool_)
# def setLEDBack(state:comm.Bool_) -> comm.Bool_:
#     pass

# @manager.command(comm.Bool_, comm.Bool_)
# def setLEDStrobe(state:comm.Bool_) -> comm.Bool_:
#     pass

# @manager.command(comm.Void, comm.Bool_)
# def getLEDFront() -> comm.Bool_:
#     pass

# @manager.command(comm.Void, comm.Bool_)
# def getLEDBack() -> comm.Bool_:
#     pass

# @manager.command(comm.Void, comm.Bool_)
# def getLEDStrobe() -> comm.Bool_:
#     pass

# @manager.command(comm.Void, Report)
# def getReport() -> Report:
#     pass


