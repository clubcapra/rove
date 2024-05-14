from io import IOBase, RawIOBase
from math import isclose
import threading
import time
from types import TracebackType
from typing import Iterable, Literal, TypeAlias, Union
import capra_micro_comm_py as comm
import can
import collections

Interface:TypeAlias = Literal[
    "kvaser",
    "socketcan",
    "serial",
    "pcan",
    "usb2can",
    "ixxat",
    "nican",
    "iscan",
    "virtual",
    "udp_multicast",
    "neovi",
    "vector",
    "slcan",
    "robotell",
    "canalystii",
    "systec",
    "seeedstudio",
    "cantact",
    "gs_usb",
    "nixnet",
    "neousys",
    "etas",
    "socketcand",
]

class benchmark:
    def __init__(self, msg:str):
        self.msg = msg
        self.start = 0
        self.end = 0
    
    def __enter__(self):
        self.start = time.time()
        return self
    
    def __exit__(self, _, __, ___):
        self.end = time.time()
        print(f"{self.msg} took {self.end - self.start} s")

class CanBusStream(RawIOBase, can.Listener):
    def __init__(self, bus:can.BusABC, maxLenth:int, notifier:can.Notifier, timeout:float=1, remoteID:int=0x103, localID:int=0x446):
        RawIOBase.__init__(self)
        can.Listener.__init__(self)
        
        self._bus = bus
        self._notifier = notifier
        self._timeout = timeout
        self._remoteID = remoteID
        self._localID = localID
        self._readqueue = collections.deque(maxlen=maxLenth)
        self._writequeue = collections.deque(maxlen=maxLenth)
        self._notifier.add_listener(self)
        self._trig = threading.Event()
        self._readSize = 0
        
    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_error_frame:
            print(f"Error: {msg}")
        if msg.arbitration_id == self._localID:
            for b in msg.data:
                self._readqueue.append(b)
            self._trig.set()
        
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type: type[BaseException] | None, exc_val: BaseException | None, exc_tb: TracebackType | None) -> None:
        self.flush()
    
    def writable(self) -> bool:
        return True
    
    def readable(self) -> bool:
        return True
    
    def seekable(self) -> bool:
        return False
    
    def tell(self) -> int:
        if len(self._readqueue) == 0:
            return -1
        return self._readqueue[0]
    
    # def read(self) -> int:
    #     tmt = time.time() + self._timeout
    #     while time.time() < tmt and len(self._readqueue) < 1:
    #         time.sleep(0.01)
    #     if len(self._readqueue) < 1:
    #         return -1
    #     return self._readqueue.popleft()
    
    def read(self, size: int = -1) -> bytes | None:
        # with benchmark("read"):
        if size == -1:
            size = len(self._readqueue)
        
        # size = min(size, len(self._readqueue))
        tmt = time.time() + self._timeout
        
        while time.time() < tmt and len(self._readqueue) < size:
            time.sleep(0.0001)
            
        if len(self._readqueue) < size:
            raise TimeoutError("Read timed out")
            # with benchmark("pop"):
        return bytes([self._readqueue.popleft() for _ in range(size)])
            
    def write(self, b: bytes) -> int | None:
        res = 0
        for bb in b:
            if len(self._writequeue) < self._writequeue.maxlen:
                res += 1
            else:
                break
            self._writequeue.append(bb)
        return res
            
    def flush(self) -> None:
        
        # with benchmark("flush"):
        while len(self._writequeue) > 0:
            block = bytes([self._writequeue.popleft() for l in range(min(len(self._writequeue), 8))])
            msg = can.Message(time.time()/1000, self._remoteID, dlc=len(block), data=block, is_extended_id=False, check=True)
            start = time.time()
            self._bus.send(msg, self._timeout)
            
            if isclose(time.time()-start, self._timeout):
                raise TimeoutError("Write timed out")
            
    def stop(self) -> None:
        self.flush()
        self._notifier.remove_listener(self)
        self.close()
        del self._readqueue
        
class CanBusCommandManager(comm.CommandManager):
    def __init__(self):
        super().__init__()
        self._stream = None
        self._channel = None
        self._interface:Interface = None
        self._bus:can.BusABC = None
        self._notifier:can.Notifier = None
        self._inUse = False
        self._bitrate = None
        self._remoteID = 0x103
        self._localID = 0x446
        self._timeout = 1
        
    @property
    def bitrate(self) -> int:
        return self._bitrate
    
    @bitrate.setter
    def bitrate(self, rate:int):
        self._bitrate = rate
        
    @property
    def channel(self) -> Union[None, str]:
        return self._channel
    
    @channel.setter
    def channel(self, chan: Union[None, str]):
        self._channel = chan
        
    @property
    def interface(self) -> Interface:
        return self._interface

    @interface.setter
    def interface(self, inter:Interface):
        self._interface = inter
        
    @property
    def notifier(self) -> can.Notifier:
        return self._notifier
    
    @notifier.setter
    def notifier(self, noti:can.Notifier):
        self._notifier = noti
    
    @property
    def remoteID(self) -> int:
        return self._remoteID
    
    @remoteID.setter
    def remoteID(self, id:int):
        if self._stream is not None:
            self._stream._remoteID = id
        self._remoteID = id
        
    @property
    def timeout(self) -> float:
        return self._timeout
    
    @timeout.setter
    def timeout(self, timeout:float):
        if self._stream is not None:
            self._stream._timeout = timeout
        self._timeout = timeout
        
    @property
    def localID(self) -> int:
        return self._localID
    
    @localID.setter
    def localID(self, id:int):
        if self._stream is not None:
            self._stream._localID = id
        self._localID = id
        
    def _default(self):
        if self._bitrate is None:
            self._bitrate = 250000
        if self._bus is None:
            self._bus = can.Bus(self._channel, self._interface, bitrate=self._bitrate)
        if self._notifier is None:
            self._notifier = can.Notifier(self._bus, [])
        
        
    def _maxSize(self):
        return max([c._len for c in self._structs]) + 1
        
    def __enter__(self) -> IOBase:
        if self._stream is None:
            self._default()
            self._stream = CanBusStream(self._bus, self._maxSize() * 2, self._notifier, self._timeout, self._remoteID, self._localID)
        return self._stream.__enter__()
        
    def __exit__(self, *args, **kwargs):
        self._stream.__exit__(*args, **kwargs)
        