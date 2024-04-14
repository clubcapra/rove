from typing import NoReturn
import capra_micro_comm_py as comm

manager = comm.SerialCommandManager()

@manager.struct('ff')
class Vector2D(comm.BinaryData):
    def __init__(self, x:float=0, y:float=0):
        super().__init__(x=x, y=y)
        self.x:float
        self.y:float

@manager.struct('H')
class Status(comm.BinaryData):
    def __init__(self, statusCode:int = 0):
        super().__init__(statusCode=statusCode)
        self.statusCode:int
     
@manager.struct('BBB')
class RGB(comm.BinaryData):
    def __init__(self, r:int=0,g:int=0,b:int=0):
        super().__init__(r=r,g=g,b=b)
        self.r:int
        self.g:int
        self.b:int
        
@manager.struct('H_x')
class LED(comm.BinaryData):
    def __init__(self, i:int=0, c:RGB=RGB(0,0,0)):
        super().__init__(i=i, c=c)


@manager.struct('__xx')
class Report(comm.BinaryData):
    def __init__(self, pos:Vector2D=Vector2D(), status:Status=Status()):
        super().__init__(pos=pos, status=status)
        self.pos:Vector2D
        self.status:Status

@manager.struct('ff')
class Bounds(comm.BinaryData):
    def __init__(self, lower:float=0, upper:float=0):
        super().__init__(lower=lower, upper=upper)
        self.lower:float
        self.upper:float
    
@manager.struct('fff_')
class PIDConfig(comm.BinaryData):
    def __init__(self, p:float=1, i:float=0, d:float=0, bounds:Bounds=Bounds()):
        super().__init__(p=p,i=i,d=d, bounds=bounds)
        self.p:float
        self.i:float
        self.d:float
        self.bounds:Bounds

@manager.struct('__')
class Config(comm.BinaryData):
    def __init__(self, horizPID:PIDConfig=PIDConfig(),vertiPID:PIDConfig=PIDConfig()):
        super().__init__(horizPID=horizPID, vertiPID=vertiPID)
        self.horizPID:PIDConfig
        self.vertiPID:PIDConfig
        
@manager.command(LED, comm.Bool_)
def setLedColor(led:LED) -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setFrontLedState(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(comm.Bool_, comm.Bool_)
def setBackLedState(state:comm.Bool_) -> comm.Bool_:
    pass

@manager.command(Vector2D, comm.Bool_)
def setTPVPosition(pos:Vector2D) -> comm.Bool_:
    pass

@manager.command(comm.Void, Vector2D)
def getTPVPosition() -> Vector2D:
    pass

@manager.command(Vector2D, comm.Bool_)
def setTPVSpeed(speed:Vector2D) -> comm.Bool_:
    pass

@manager.command(comm.Void, Report)
def getReport() -> Report:
    pass

@manager.command(Config, comm.Bool_)
def setConfig(config:Config) -> comm.Bool_:
    pass


