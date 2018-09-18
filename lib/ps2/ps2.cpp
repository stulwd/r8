#include "ps2.h"
#define DELAY 1
int PS2CMD, PS2DAT, PS2CLK, PS2CS;

unsigned char spiSend(unsigned char working){
  unsigned char in = 0;
  
  for(byte i = 0; i < 8 ; i++){
    digitalWrite(PS2CMD, (working & 1) ? HIGH : LOW);
    digitalWrite(PS2CLK,LOW);
    delayMicroseconds(DELAY);
    in = in + ((digitalRead(PS2DAT) & 1) << i);
    digitalWrite(PS2CLK,HIGH);
    working = working >> 1;
    //in = in + ((digitalRead(PS2DAT) & 1) << i);
    delayMicroseconds(DELAY);  
  }
  return in;
}

void PS2::setupPS2(int clk, int cs, int cmd, int dat){
  pinMode(PS2CLK = clk, OUTPUT);
  pinMode(PS2CMD = cmd, OUTPUT);
  pinMode(PS2DAT = dat, INPUT);
  pinMode(PS2CS  = cs, OUTPUT);
  
  digitalWrite(PS2CS, HIGH);
  digitalWrite(PS2CLK,HIGH);
}

unsigned char ps2buf[9];
unsigned char btnState[2];
void PS2::updatePS2(){
  digitalWrite(PS2CS, LOW);
  delayMicroseconds(DELAY * 8);

  ps2buf[0] = spiSend(0x01); delayMicroseconds(DELAY * 8);
  ps2buf[1] = spiSend(0x42); delayMicroseconds(DELAY * 8);
  ps2buf[2] = spiSend(0x00); delayMicroseconds(DELAY * 8);


  if(ps2buf[1] == 0x73 && ps2buf[2] == 0x5a){
    ps2buf[3] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    ps2buf[4] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    ps2buf[5] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    ps2buf[6] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    ps2buf[7] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    ps2buf[8] = spiSend(0x00); delayMicroseconds(DELAY * 8);
    // Serial.printf("%2x, %2x, %2x, %2x | ", ps2buf[5], ps2buf[6], ps2buf[7], ps2buf[8]);
  }else{
    ps2buf[3] = 0xff;
    ps2buf[4] = 0xff;
    ps2buf[5] = 0x80;
    ps2buf[6] = 0x7f;
    ps2buf[7] = 0x80;
    ps2buf[8] = 0x7f;
  }

  digitalWrite(PS2CS, HIGH);
  delayMicroseconds(DELAY * 8);

  if(ps2buf[2] == 0x5a){
    btnState[0] = ps2buf[3];
    btnState[1] = ps2buf[4];
  }
  else
  {
    btnState[0] = 0xff;
    btnState[1] = 0xff;
  }

  // Serial.printf("%2x, %2x, %2x, %2x, %2x\n", ps2buf[0], ps2buf[1], ps2buf[2], ps2buf[3], ps2buf[4]);
}

unsigned char PS2::getButton(short mask){
  byte b4 = mask >> 8;
  byte b5 = mask & 0xff;
  return (b4 & (~btnState[0])) || (b5 & (~btnState[1]));
}

unsigned char PS2::getAxis(short mask){
  return ps2buf[4 + mask];
}

unsigned char cap(int v){
  if(v > 0) v = v / 4 + 0xbb;
  return v > 255 ? 255 : v < 0 ? 0 : v;
}

void PS2::getDrivePWM(int& a, int& b, int& c, int& d){
  auto y = PS2::getAxis(PS2LY);
  auto x = PS2::getAxis(PS2RX);
  auto py = (y > 0x7f ? y - 0x7f : 0) * 2;
  auto ny = (y < 0x7f ? 0x7f - y : 0) * 2;
  auto px = (x > 0x80 ? x - 0x80 : 0) * 2;
  auto nx = (x < 0x80 ? 0x80 - x : 0) * 2;

  a = cap(px + py);
  b = cap(nx + ny);
  c = cap(nx + py);
  d = cap(px + ny);

  static int count = 0;
  if(count++ == 19){
    count = 0;
    //Serial.printf("%2x, %2x, %2x, %2x, %2x\n", py, ny, px, nx, PS2::getButton(PS2O));
    
  }

}
