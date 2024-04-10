from __future__ import annotations
import base64
import functools
from io import IOBase, RawIOBase
import struct
from textwrap import dedent
from typing import Callable, Generic, List, NoReturn, Type, TypeVar, Union

from serial import Serial

BinaryType = Union[int, float, bool]

TYPE_MAP = {
    'x' : 'euint8_t',
    'c' : 'eint8_t',
    'b' : 'eint8_t',
    'B' : 'euint8_t',
    '?' : 'eboolean_t',
    'h' : 'eint16_t',
    'H' : 'euint16_t',
    'i' : 'eint32_t',
    'I' : 'euint32_t',
    'l' : 'eint32_t',
    'L' : 'euint32_t',
    'q' : 'eint64_t',
    'Q' : 'euint64_t',
    'f' : 'efloat_t',
}

class BinaryData:
    def __init__(self, fmt:str, **values):
        self._fmt = fmt
        self._keys = list(values.keys())
        self.__dict__.update(values)
    
    def unpack(self, buff:bytes):
        for k, v in zip(self._keys, struct.unpack('<'+self._fmt, buff)):
            self.__dict__[k] = v
    
    @property
    def values(self):
        v = []
        for k in self._keys:
            v.append(self.__dict__[k])
        
        return v

    def __len__(self):
        return struct.calcsize(self._fmt)

    def parse(self):
        i = 0
        pad = 0
        s = f'struct {self.__class__.__name__}' + '\n{\n'
        for f in self._fmt:
            l = '    ' + TYPE_MAP[f] + ' '
            if f == 'x':
                n = f'pad{pad}'
                pad += 1
            else:
                n = self._keys[i]
            
            l += n + ';\n'
            
            if f != 'x':
                i+=1
            s += l
        s += '};\n'
        s += f'static_assert(sizeof({self.__class__.__name__}) == {len(self)});'
        return s
            
            

R = TypeVar('R', bound=BinaryData)
P = TypeVar('P', bound=BinaryData)

class CommandHook(Generic[P, R]):
    def __init__(self, parent:CommandManager, id:int, func:Callable[[P], R], paramType:Type[P], returnType:Type[R]):
        self.parent = parent
        self.id = id
        self.paramType = paramType
        self.returnType = returnType
        self._call = func
        # self.__call__ = functools.update_wrapper(self.__call__, func)
        self._pre_call = None
        self._post_call = None
        
    def __call__(self, p:P=None) -> R:
        # Handle Void parameter
        if self.paramType == Void:
            cb = lambda c, pp: c()
            p = Void()
        else:
            cb = lambda c, pp: c(pp)
        
        # Call pre call if provided
        if self._pre_call:
            cb(self._pre_call, p)
        
        # Encode data
        param = struct.pack('<B'+p._fmt, self.id, *p.values)
        print(param)
        with self.parent as stream:
            stream:RawIOBase
            # Send data
            stream.write(param)
            
            # Call base call
            cb(self._call, p)
            
            # Read data
            if self.returnType != Void:
                res = stream.read(len(self.returnType()))
                print(bin(int.from_bytes(res, 'little')))
        
        # Handle Void return
        r = None
        if self.returnType != Void:
            # Decode data if not Void
            r = self.returnType()
            r.unpack(res)
        
        # Call post call if provided
        if self._post_call:
            if self.returnType == Void:
                self._post_call()
            else:
                self._post_call(r)
        return r
    
    def __pre_call__(self, func:Callable[[P], NoReturn]) -> Callable[[P], NoReturn]:
        self._pre_call = func
        return self._pre_call

    def preCall(self, func:Callable[[P], NoReturn]) -> Callable[[P], NoReturn]:
        return self.__pre_call__(func)
    
    def __post_call__(self, func:Callable[[R], NoReturn]) -> Callable[[R], NoReturn]:
        self._post_call = func
        return self._post_call
    
    def postCall(self, func:Callable[[R], NoReturn]) -> Callable[[R], NoReturn]:
        return self.__post_call__(func)
    
    def __len__(self):
        return struct.calcsize('<B'+self.paramType()._fmt)
        
    def parse(self):
        r = self.returnType.__name__
        p = self.paramType.__name__
        f = self._call.__name__
        
        return f'{r} {f}({p});'

    def lstParse(self):
        r = self.returnType.__name__
        p = self.paramType.__name__
        f = self._call.__name__
        
        return f'new Function<{r}, {p}>(&{f})'

    def assertParse(self):
        return f'static_assert((sizeof({self.paramType.__name__})+1) == {len(self)});'

class Base64(RawIOBase):
    def __init__(self, stream:IOBase):
        super().__init__()
        self.stream = stream
    
    def read(self, size: int = -1) -> bytes | None:
        l = self.stream.readline().rstrip(b'\n')
        print(l)
        return base64.decodebytes(l)

    def write(self, b) -> int | None:
        line = base64.encodebytes(b)
        print(line)
        return self.stream.write(line+b'\n')
    
    def flush(self) -> None:
        return self.stream.flush()
    
class CommandManager:
    def __init__(self):
        self._id = 0
        self._stream = None
        self._commands:List[CommandHook] = []
        
    def _genID(self):
        res = self._id
        self._id += 1
        return res
        
    def binaryFunction(self, paramType:Type[P], returnType:Type[R]):
        id = self._genID()
        def predicate(func:Callable[[P], R]) -> Union[Callable[[P], R], CommandHook[P,R]]:
            res = CommandHook(self, id, func, paramType, returnType)
            self._commands.append(res)
            return functools.update_wrapper(res, func)
        
        return predicate
        
        
    def __enter__(self) -> IOBase: ...
    def __exit__(self, exc_type, exc_val, exc_tb): ...
    
    def buildAPI(self) -> str:
        res = dedent("""\
            #pragma once
            /* This was generated by capra_micro_comm.
            * DO NOT EDIT
            */
            #include <capra_comm.h>
            
            """)
        maxSize = 0
        for cls in BinaryData.__subclasses__():
            inst = cls()
            res += inst.parse() + '\n\n'
            maxSize = max(maxSize, len(inst))
        lst = "BaseFunction_ptr commands[] = {\n"
        for cmd in self._commands:
            res += cmd.parse() + '\n'
            res += cmd.assertParse() + '\n\n'
            lst += f'    {cmd.lstParse()},\n'
        res += lst + '};\n'
        res += f"#define COMMANDS_COUNT {len(self._commands)}\n"
        res += f"#define MAX_DECODED_SIZE {maxSize+1}\n"
        
        encoded = base64.encodebytes(bytes(range(maxSize+1)))
        res += f"#define MAX_ENCODED_SIZE {len(encoded)}\n"
        return res


class SerialCommandManager(CommandManager):
    def __init__(self, port:str = None, baud:int=9600):
        super().__init__()
        self._stream = Serial()
        self._encoder = Base64(self._stream)
        self._stream.port = port
        self._stream.baudrate = baud

    @property
    def port(self) -> str:
        return self._stream.port
    
    @port.setter
    def port(self, p:str):
        self._stream.port = p
    
    @property
    def baud(self) -> int:
        return self._stream.baudrate
    
    @port.setter
    def baud(self, b:int):
        self._stream.baudrate = b
    
    def __enter__(self) -> IOBase:
        if not self._stream.is_open:
            self._stream.open()
        return self._encoder
        
    def __exit__(self, *args, **kwargs):
        self._stream.flush()
        
    
        
class Void(BinaryData):
    def __init__(self):
        BinaryData.__init__(self, 'x')

    