from __future__ import annotations
import base64
from ctypes import ArgumentError
from enum import Enum
import functools
from io import IOBase, RawIOBase
import struct
from textwrap import dedent
from typing import Callable, ForwardRef, Generic, List, NoReturn, Type, TypeVar, Union, ClassVar

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

SIZE_MAP = {
    'x' : 1,
    'c' : 1,
    'b' : 1,
    'B' : 1,
    '?' : 1,
    'h' : 2,
    'H' : 2,
    'i' : 4,
    'I' : 4,
    'l' : 4,
    'L' : 4,
    'q' : 8,
    'Q' : 8,
    'f' : 4,
}

class BinaryData:
    # _fmt = ''
    # _keys:List[str] = []
    # _values:List[Union[BinaryData, BinaryData]] = []
    # _len = 0
    def __init__(self, **values):
        # Set instance attributes
        self._keys = values.keys()
        self._values = values.values()
        self.__dict__.update(values)
        
    def __len__(self):
        # return struct.calcsize('<'+self._fmt)
        return self._len
    
    def unpack(self, buff:bytes):
        # Unpack data
        b = struct.unpack('<'+self._fmt, buff)
        # Buffer index
        idx = 0
        # Format index
        fidx = 0
        for k, v in zip(self.__class__._keys, self.__class__._values):
            if isinstance(v, (int, float, bool)):
                # Set value
                self.__dict__[k] = b[fidx]
                
                # Find size in bytes and move buffer index accordingly
                idx+= SIZE_MAP[self.__class__._fmt[fidx]]
                
                # Base type taking one format space
                fidx+=1
            elif isinstance(v, BinaryData):
                # Subclass of BinaryData
                # Instanciate
                inst = v.__class__()
                
                # Unpack
                inst.unpack(buff[idx:idx+len(inst)])
                
                # Assign value
                self.__dict__[k] = inst
                
                # Move buffer index by the byte length
                idx+=len(inst)
                
                # Move the format index by the values count
                fidx+=len(v.__class__._values)
            elif isinstance(v, Enum) and hasattr(v.__class__, '_fmt'):
                # Enum that contains formatting inromation
                self.__dict__[k] = v.__class__(b[fidx])
                
                # Find size in bytes and move buffer index accordingly
                idx+= SIZE_MAP[self.__class__._fmt[fidx]]
                
                # Base type taking one format space
                fidx+=1
                
    @property
    def values(self):
        v = []
        for k in self._keys:
            v.append(self.__dict__[k])
        
        return v

    def __len__(self):
        return struct.calcsize(self._fmt)

    def parseCls(self):
        cls = self.__class__
        i = 0
        pad = 0
        s = f'struct {cls.__name__}' + '\n{\n'
        for f in cls._fmt:
            l = '    ' + TYPE_MAP[f] + ' '
            if f == 'x':
                n = f'pad{pad}'
                pad += 1
            else:
                n = cls._keys[i]
            
            l += n + ';\n'
            
            if f != 'x':
                i+=1
            s += l
        s += '};\n'
        s += f'static_assert(sizeof({cls.__name__}) == {self._len});'
        return s
            
B = TypeVar('B', bound=Type[BinaryData])
E = TypeVar('E', bound=Type[Enum])
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
        # print(param)
        with self.parent as stream:
            stream:RawIOBase
            # Send data
            stream.write(param)
            
            # Call base call
            cb(self._call, p)
            
            # Read data
            if self.returnType != Void:
                res = stream.read(len(self.returnType()))
                # print(bin(int.from_bytes(res, 'little')))
        
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
        # print(l)
        return base64.decodebytes(l)

    def write(self, b) -> int | None:
        line = base64.encodebytes(b)
        # print(line)
        return self.stream.write(line+b'\n')
    
    def flush(self) -> None:
        return self.stream.flush()
    

    

class CommandManager:
    _basetypes:List[BinaryData] = []
    def __init__(self):
        self._id = 0
        self._stream = None
        self._commands:List[CommandHook] = []
        self._structs:List[Type[BinaryData]] = []
        self._enums:List[Type[Enum]] = []
        self._structs = list(CommandManager._basetypes)
        
    def _genID(self):
        res = self._id
        self._id += 1
        return res
        
    def command(self, paramType:Type[P], returnType:Type[R]):
        id = self._genID()
        def predicate(func:Callable[[P], R]) -> Union[Callable[[P], R], CommandHook[P,R]]:
            res = CommandHook(self, id, func, paramType, returnType)
            self._commands.append(res)
            return functools.update_wrapper(res, func)
        
        return predicate

    def struct(self, fmt:str):
        return _addtype(fmt, self._structs)
    
    def enum(self, fmt:str = 'I'):
        def pred(cls:E):
            def parse():
                s = f'enum class {cls.__name__} : {TYPE_MAP[fmt]}\n'
                s += '{\n'
                for k, v in cls.__members__.items():
                    s += f'    {k} = {v.value},\n'
                s += '};'
                return s
            cls.parse = parse
            cls._fmt = fmt
            self._enums.append(cls)
            return cls
        return pred
        
    def __enter__(self) -> IOBase: ...
    def __exit__(self, exc_type, exc_val, exc_tb): ...
    
    def buildAPI(self) -> str:
        res = dedent("""\
            #pragma once
            /* This was generated by capra_micro_comm.
            * DO NOT EDIT
            */
            #include <capra_comm.h>
            
            // --- ENUMS ---\n""")
        for e in self._enums:
            res += e.parse() + '\n\n'
            
        res += "// --- STRUCTS ---\n"
        maxSize = 0
        for cls in self._structs:
            v = cls()
            res += v.parseCls() + '\n\n'
            maxSize = max(maxSize, len(v))
        
        lst = "BaseFunction_ptr commands[] = {\n"
        res += "// --- COMMANDS ---\n"
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

class StructError(Exception):
    def __init__(self):
        super().__init__("No empty constructors found. Make sure to have an __init__() with no parameters.")
    

def _addtype(fmt:str, lst:List[BinaryData]):
    def pred(cls:B):
        # Setup class attributes
        cls._fmt = ''
        try:
            inst = cls()
        except TypeError as e:
            raise StructError()
        
        global fidx
        fidx = 0
        def pad():
            global fidx
            while fidx < len(fmt) and fmt[fidx] == 'x':
                cls._fmt += 'x'
                fidx+=1
        pad()
        for v in inst._values:
            if isinstance(v, (int, float, bool)):
                cls._fmt += fmt[fidx]
            elif isinstance(v, BinaryData):
                # Subclass of BinaryData
                cls._fmt += v._fmt
            elif isinstance(v, Enum) and hasattr(v.__class__, '_fmt'):
                # Enum that contains formatting information
                cls._fmt += v.__class__._fmt
            fidx+=1
            pad()
        cls._keys = list(inst._keys)
        cls._values = list(inst.values)
        cls._len = len(inst)
        lst.append(cls)
        
        return cls
    return pred

def _basetype(fmt:str):
    return _addtype(fmt, CommandManager._basetypes)
    

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
        
@_basetype('x')
class Void(BinaryData):
    def __init__(self):
        super().__init__()
    
@_basetype('?')
class Bool(BinaryData):
    def __init__(self, b:bool=False):
        super().__init__(b=b)
        self.b:bool
        
@_basetype('B')
class Byte(BinaryData):
    def __init__(self, b:int=0):
        super().__init__(b=b)
        self.b:int
        
@_basetype('h')
class Short(BinaryData):
    def __init__(self, s:int=0):
        super().__init__(s=s)
        self.s:int
        
@_basetype('H')
class UShort(BinaryData):
    def __init__(self, s:int=0):
        super().__init__(s=s)
        self.s:int
        
@_basetype('i')
class Int(BinaryData):
    def __init__(self, i:int=0):
        super().__init__(i=i)
        self.i:int
        
@_basetype('I')
class UInt(BinaryData):
    def __init__(self, i:int=0):
        super().__init__(i=i)
        self.i:int
        
@_basetype('q')
class Long(BinaryData):
    def __init__(self, l:int=0):
        super().__init__(l=l)
        self.l:int
        
@_basetype('Q')
class ULong(BinaryData):
    def __init__(self, l:int=0):
        super().__init__(l=l)
        self.l:int
        
@_basetype('f')
class Float(BinaryData):
    def __init__(self, f:float=0):
        super().__init__(f=f)
        self.f:float
        