from io import IOBase, RawIOBase
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

class CanBusStream(RawIOBase, can.Listener):
    def __init__(self, bus:can.BusABC, maxLenth:int, notifier:can.Notifier, timeout:float=1, remoteID:int=0x446):
        RawIOBase.__init__(self)
        can.Listener.__init__(self)
        
        self._bus = bus
        self._notifier = notifier
        self._timeout = timeout
        self._remoteID = remoteID
        self._readqueue = collections.deque(maxlen=maxLenth)
        self._writequeue = collections.deque(maxlen=maxLenth)
        self._notifier.add_listener(self)
        
    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == self._remoteID:
            for b in msg.data:
                self._readqueue.append(b)
        
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
    
    def read(self, size: int = -1) -> bytes | None:
        
        if size == -1:
            size = len(self._readqueue)
        size = min(size, len(self._readqueue))
        
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
        
        while len(self._writequeue) > 0:
            block = bytes([self._writequeue.popleft() for l in range(min(len(self._writequeue, 8)))])
            msg = can.Message(time.time(), self._remoteID, dlc=len(block))
            self._bus.send(msg, self._timeout)
            
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
        
    def _default(self):
        if self._bus is None:
            self._bus = can.Bus(self._channel, self._interface)
        if self._notifier is None:
            self._notifier = can.Notifier(self._bus, [])
        
    def _maxSize(self):
        return max([c._len for c in self._structs]) + 1
        
    def __enter__(self) -> IOBase:
        if self._stream is None:
            self._default()
            self._stream = CanBusStream(self._bus, self._maxSize() * 2, self._notifier)
        return self._stream.__enter__()
        
    def __exit__(self, *args, **kwargs):
        self._stream.__exit__(*args, **kwargs)
        