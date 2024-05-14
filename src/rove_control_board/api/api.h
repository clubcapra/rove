#pragma once
/* This was generated by capra_micro_comm.
* DO NOT EDIT
*/

#include <capra_comm.h>

enum StatusCode : euint16_t
{
    STNotInitialized = 0,
    STInitialized = 1,
    STConfigured = 2,
};

enum ErrorCode : euint16_t
{
    ERNone = 0,
    ERAdapterNotInit = 1,
    ERServoNACK = 2,
    ERServoXNACK = 3,
    ERServoYNACK = 4,
    ERWinchLocked = 5,
    ERRXBuffOverflow = 6,
    ERRTXBufferOvervlow = 7,
};

enum ServoControlMode : euint16_t
{
    SCMNone = 0,
    SCMPosition = 1,
    SCMSpeed = 2,
};

enum WinchMode : euint16_t
{
    WMFreeWheel = 0,
    WMBrake = 1,
    WMReverse = 2,
    WMForward = 3,
};




// --- STRUCTS ---
struct Void
{
    euint8_t pad0;
};
static_assert(sizeof(Void) == 1);

struct Bool_
{
    eboolean_t b;
};
static_assert(sizeof(Bool_) == 1);

struct Byte
{
    euint8_t b;
};
static_assert(sizeof(Byte) == 1);

struct Short
{
    eint16_t s;
};
static_assert(sizeof(Short) == 2);

struct UShort
{
    euint16_t s;
};
static_assert(sizeof(UShort) == 2);

struct Int
{
    eint32_t i;
};
static_assert(sizeof(Int) == 4);

struct UInt
{
    euint32_t i;
};
static_assert(sizeof(UInt) == 4);

struct Long
{
    eint64_t l;
};
static_assert(sizeof(Long) == 8);

struct ULong
{
    euint64_t l;
};
static_assert(sizeof(ULong) == 8);

struct Float
{
    efloat_t f;
};
static_assert(sizeof(Float) == 4);

struct Vector2D
{
    eint16_t x;
    eint16_t y;
};
static_assert(sizeof(Vector2D) == 4);

struct RGB
{
    euint8_t r;
    euint8_t g;
    euint8_t b;
    euint8_t pad0;
};
static_assert(sizeof(RGB) == 4);

struct RGBLed
{
    RGB rgb;
    euint8_t index;
};
static_assert(sizeof(RGBLed) == 5);

struct Report
{
    Vector2D pos;
    euint16_t statusCode;
    euint16_t errorCode;
};
static_assert(sizeof(Report) == 8);

struct Bounds
{
    euint16_t lower;
    euint16_t upper;
};
static_assert(sizeof(Bounds) == 4);

struct Configuration
{
    Bounds xBounds;
    Bounds yBounds;
};
static_assert(sizeof(Configuration) == 8);

// --- COMMANDS ---
Int ping(Int);
static_assert((sizeof(Int)+1) == 5);

ULong hashCheck(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setServoPosition(Vector2D);
static_assert((sizeof(Vector2D)+1) == 5);

Vector2D getServoPosition(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setServoPositionZero(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setServoSpeed(Vector2D);
static_assert((sizeof(Vector2D)+1) == 5);

Vector2D getServoSpeed(Void);
static_assert((sizeof(Void)+1) == 2);

Short getServoPositionX(Void);
static_assert((sizeof(Void)+1) == 2);

Short getServoPositionY(Void);
static_assert((sizeof(Void)+1) == 2);

Short getServoSpeedX(Void);
static_assert((sizeof(Void)+1) == 2);

Short getServoSpeedY(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setServoXAcc(Byte);
static_assert((sizeof(Byte)+1) == 2);

Bool_ setServoYAcc(Byte);
static_assert((sizeof(Byte)+1) == 2);

Byte getServoXAcc(Void);
static_assert((sizeof(Void)+1) == 2);

Byte getServoYAcc(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setLEDFront(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

Bool_ setLEDBack(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

Bool_ setLEDStrobe(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

Bool_ getLEDFront(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ getLEDBack(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ getLEDStrobe(Void);
static_assert((sizeof(Void)+1) == 2);

Report getReport(Void);
static_assert((sizeof(Void)+1) == 2);

UShort getWinchMode(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setWinchMode(UShort);
static_assert((sizeof(UShort)+1) == 3);

Bool_ getWinchLock(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setWinchLock(Void);
static_assert((sizeof(Void)+1) == 2);

UShort getServoControlMode(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setServoControlMode(UShort);
static_assert((sizeof(UShort)+1) == 3);

Bool_ getGPIO1(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setGPIO1(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

Bool_ getGPIO2(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setGPIO2(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

Bool_ getGPIO3(Void);
static_assert((sizeof(Void)+1) == 2);

Bool_ setGPIO3(Bool_);
static_assert((sizeof(Bool_)+1) == 2);

RGB getRGBLed(Int);
static_assert((sizeof(Int)+1) == 5);

Bool_ setRGBLed(RGBLed);
static_assert((sizeof(RGBLed)+1) == 6);

Bool_ configure(Configuration);
static_assert((sizeof(Configuration)+1) == 9);

static BaseFunction_ptr commands[] = {
    new Function<Int, Int>(&ping),
    new Function<ULong, Void>(&hashCheck),
    new Function<Bool_, Vector2D>(&setServoPosition),
    new Function<Vector2D, Void>(&getServoPosition),
    new Function<Bool_, Void>(&setServoPositionZero),
    new Function<Bool_, Vector2D>(&setServoSpeed),
    new Function<Vector2D, Void>(&getServoSpeed),
    new Function<Short, Void>(&getServoPositionX),
    new Function<Short, Void>(&getServoPositionY),
    new Function<Short, Void>(&getServoSpeedX),
    new Function<Short, Void>(&getServoSpeedY),
    new Function<Bool_, Byte>(&setServoXAcc),
    new Function<Bool_, Byte>(&setServoYAcc),
    new Function<Byte, Void>(&getServoXAcc),
    new Function<Byte, Void>(&getServoYAcc),
    new Function<Bool_, Bool_>(&setLEDFront),
    new Function<Bool_, Bool_>(&setLEDBack),
    new Function<Bool_, Bool_>(&setLEDStrobe),
    new Function<Bool_, Void>(&getLEDFront),
    new Function<Bool_, Void>(&getLEDBack),
    new Function<Bool_, Void>(&getLEDStrobe),
    new Function<Report, Void>(&getReport),
    new Function<UShort, Void>(&getWinchMode),
    new Function<Bool_, UShort>(&setWinchMode),
    new Function<Bool_, Void>(&getWinchLock),
    new Function<Bool_, Void>(&setWinchLock),
    new Function<UShort, Void>(&getServoControlMode),
    new Function<Bool_, UShort>(&setServoControlMode),
    new Function<Bool_, Void>(&getGPIO1),
    new Function<Bool_, Bool_>(&setGPIO1),
    new Function<Bool_, Void>(&getGPIO2),
    new Function<Bool_, Bool_>(&setGPIO2),
    new Function<Bool_, Void>(&getGPIO3),
    new Function<Bool_, Bool_>(&setGPIO3),
    new Function<RGB, Int>(&getRGBLed),
    new Function<Bool_, RGBLed>(&setRGBLed),
    new Function<Bool_, Configuration>(&configure),
};
#define COMMANDS_COUNT 37
#define MAX_DECODED_SIZE 9
#define MAX_ENCODED_SIZE 13
#define API_HASH 1662470386655982205UL