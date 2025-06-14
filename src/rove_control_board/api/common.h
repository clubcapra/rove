#pragma once

#include <inttypes.h>

#ifndef CMD_BUFF_SIZE
#define CMD_BUFF_SIZE 64
#endif // CMD_BUFF_SIZE

// Define types with known lengths for encoding
#ifdef ARDUINO
using euint8_t      = uint8_t;
using eint8_t       = int8_t;
using euint16_t     = uint16_t;
using eint16_t      = int16_t;
using euint32_t     = uint32_t;
using eint32_t      = int32_t;
using euint64_t     = uint64_t;
using eint64_t      = int64_t;
using efloat_t      = float;
using eboolean_t    = bool;

#include <Arduino.h>

class EmptyStream : public Stream
{
public:
    int available() override { return 0; }
    int read() override { return 0; }
    int peek() override { return 0; }
    void flush() override { }
    size_t write(uint8_t) override { return 0; }
    void begin(int) {};
    operator bool() {return true;}
};

#ifndef SAM
    extern EmptyStream debug = EmptyStream();
    extern Serial_& comm = Serial
#else
    extern Serial_& debug;
    extern UARTClass& comm;
#endif

#define DebugVar(var) { debug.print(#var); debug.print(": "); debug.print(var); }
#define DebugVarln(var) { debug.print(#var); debug.print(": "); debug.println(var); }
#define Debug(...) debug.print(__VA_ARGS__)
#define Debugln(...) debug.println(__VA_ARGS__)


inline void debugByte(uint8_t b)
{
    debug.print(b, 2);
    debug.print('|');
}

inline void debugBytes(uint8_t* b, size_t count)
{
    for (uint8_t* bb = b; bb < b + count; ++bb)
    {
        debug.print(*bb, 2);
        debug.print('|');
    }
    debug.println();
    for (uint8_t* bb = b; bb < b + count; ++bb)
    {
        debug.print(*bb, 16);
        debug.print('|');
    }
    debug.println();
}
#else 
using euint8_t      = uint8_t;
using eint8_t       = int8_t;
using euint16_t     = uint16_t;
using eint16_t      = int16_t;
using euint32_t     = uint32_t;
using eint32_t      = int32_t;
using euint64_t     = uint64_t;
using eint64_t      = int64_t;
using efloat_t      = float;
using eboolean_t    = bool;


#define DebugVar(var)
#define DebugVarln(var)
#define Debug(...)
#define Debugln(...)
#define debugByte(b)
#define debugBytes(b, c)
#endif // ARDUINO



// Verify type lengths
static_assert(sizeof(uint8_t)       == 1);

static_assert(sizeof(euint8_t)      == 1);
static_assert(sizeof(eint8_t)       == 1);
static_assert(sizeof(euint16_t)     == 2);
static_assert(sizeof(eint16_t)      == 2);
static_assert(sizeof(euint32_t)     == 4);
static_assert(sizeof(eint32_t)      == 4);
static_assert(sizeof(euint64_t)     == 8);
static_assert(sizeof(eint64_t)      == 8);
static_assert(sizeof(efloat_t)      == 4);
static_assert(sizeof(eboolean_t)    == 1);


