from rove_control_board.capra_micro_comm import BinaryData, SerialCommandManager, Void, CommandManager

manager = SerialCommandManager()

@manager.struct('ff')
class Vector2D(BinaryData):
    def __init__(self, x:float=0, y:float=0):
        super().__init__(x=x, y=y)
        self.x:float
        self.y:float

@manager.struct('?')
class State(BinaryData):
    def __init__(self, state:bool=False):
        super().__init__(state=state)
        self.state:bool

@manager.struct('B')
class Status(BinaryData):
    def __init__(self, statusCode:int = 0):
        super().__init__(statusCode=statusCode)
        self.statusCode:int
     
@manager.struct('fff')
class RGB(BinaryData):
    def __init__(self, r:float=0,g:float=0,b:float=0):
        super().__init__(r=r,g=g,b=b)
        self.r:float
        self.g:float
        self.b:float

@manager.command(Void, Void)
def ledOn():
    print("on")

@manager.command(Void, Void)
def ledOff():
    print("off")

