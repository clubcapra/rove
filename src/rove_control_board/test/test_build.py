import os
from pathlib import Path
from typing import Type
import pytest
import capra_micro_comm_py as ucomm

def test_baseAPI():
    manager = ucomm.CommandManager()
    api, _ = manager.generateAPI()
    for s in manager._structs:
        inst = s()
        assert api.find(s.__name__) != -1
        for k in inst._keys:
            assert api.find(k) != -1
        
def test_commands():
    manager = ucomm.CommandManager()
    
    @manager.command(ucomm.Void, ucomm.Void)
    def funcVoid():
        pass
    
    @manager.command(ucomm.Bool_, ucomm.Bool_)
    def funcBool(p:ucomm.Bool_) -> ucomm.Bool_:
        pass

    @manager.command(ucomm.Byte, ucomm.Byte)
    def funcByte(p:ucomm.Byte) -> ucomm.Byte:
        pass

    @manager.command(ucomm.Short, ucomm.Short)
    def funcShort(p:ucomm.Short) -> ucomm.Short:
        pass

    @manager.command(ucomm.UShort, ucomm.UShort)
    def funcUShort(p:ucomm.UShort) -> ucomm.UShort:
        pass

    @manager.command(ucomm.Int, ucomm.Int)
    def funcInt(p:ucomm.Int) -> ucomm.Int:
        pass

    @manager.command(ucomm.UInt, ucomm.UInt)
    def funcUInt(p:ucomm.UInt) -> ucomm.UInt:
        pass

    @manager.command(ucomm.Long, ucomm.Long)
    def funcLong(p:ucomm.Long) -> ucomm.Long:
        pass

    @manager.command(ucomm.ULong, ucomm.ULong)
    def funcULong(p:ucomm.ULong) -> ucomm.ULong:
        pass

    @manager.command(ucomm.Float, ucomm.Float)
    def funcFloat(p:ucomm.Float) -> ucomm.Float:
        pass

    
    api, _ = manager.generateAPI()
    for c in manager._commands:
        found = False
        c:ucomm.CommandHook[ucomm.BinaryData, ucomm.BinaryData]
        p = c.paramType.__name__
        r = c.returnType.__name__
            
        for l in api.splitlines():
            if c._call.__name__ not in l:
                continue
            if l.startswith(r) and l.endswith(f'({p});'):
                found = True
                break
        assert found

        
def test_api():
    from rove_control_board.api import manager as man
    man.generateAPI()
    
def test_apiBuild():
    from rove_control_board.api import manager as man
    d = Path(__file__).parent / 'temp'
    os.system(f'rm -rf {str(d)}')
    man.testAPI(d)
    os.system(f'rm -rf {str(d)}')


