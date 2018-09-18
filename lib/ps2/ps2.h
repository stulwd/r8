#ifndef SPI_PS2
#define SPI_PS2

#define PS2S      0x80
#define PS2X      0x40
#define PS2O      0x20
#define PS2T      0x10
#define PS2R1     0x8
#define PS2L1     0x4
#define PS2R2     0x2
#define PS2L2     0x1
#define PS2Left   0x8000
#define PS2Down   0x4000
#define PS2Right  0x2000
#define PS2Up     0x1000
#define PS2Start  0x800
#define PS2R3     0x400
#define PS2L3     0x200
#define PS2Select 0x100

#define PS2RX     0x1
#define PS2RY     0x2
#define PS2LX     0x3
#define PS2LY     0x4

#include <Arduino.h>
namespace PS2{
    void setupPS2(int clk, int cs, int cmd, int dat);
    void updatePS2();
    unsigned char getButton(short mask);
    unsigned char getAxis(short axis);
    void getDrivePWM(int& a, int& b, int& c, int& d);
}

#endif
