#include <Arduino.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1331.h"

#include <SPI.h>
#define OLED_DC      5
#define OLED_CS     12
#define OLED_RESET   6

#define BUTTON      11
#define CHIPINT     10  //unused
#define VBATPIN     A7  // A7 = D9 !!  // unsused
#define STEPLED     13

#include <Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

long stoss;
long firstStoss =-1;

long treshold = 1;
long barrier = 0;
long steps   = 0;

int deltax,deltay;
int lastx=0;
int lasty=0;

byte tick = 0;
byte hours   = 0;
byte minutes = 0;
byte seconds = 0;

// The temperature sensor is -40 to +85 degrees Celsius.
// It is a signed integer.
// According to the datasheet: 
//   340 per degrees Celsius, -512 at 35 degrees.
// At 0 degrees: -512 - (340 * 35) = -12412
double dT;

// Color definitions
#define BLACK           0x0000
#define GREY            0b0010000100000100
#define GREYBLUE        0b0010000100010000
#define LIGHTBLUE       0b0110001000011111
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define AMBER           0b1111101111100111
#define WHITE           0xFFFF

byte batLength = 30;

#define EYE1x 47
#define EYE1y 20
#define EYE2x 64
#define EYE2y 20

Adafruit_SSD1331 oled = Adafruit_SSD1331(OLED_CS, OLED_DC, OLED_RESET);

byte powerVal(int mv) {
  //float quot = (5100-2700)/(batLength-3); // scale: 5100 -> batLength, 2710 -> 0
  //return (mv-2700)/quot;
  float quot = (4400-3550)/(batLength-3); // scale: 4400 -> batLength, 3550 -> 0
  return (mv-3550)/quot;  
}

int readVcc() {
  float mv = analogRead(VBATPIN);
  mv *= 2;
  mv *= 3.3;
  return powerVal(mv);
}

short green2red(int val, int maxi) {
  // 16 bit = 5+6+5
  short result = 0x0000;
  int redPart   = 0;
  int greenPart = 0;
  if (val > (maxi/2)) {
    greenPart = 63;
    redPart = 31 - 62 * ((float) val)/((float) maxi); // 31 = 0b11111
  } else {
    redPart = 31;
    greenPart = 127 * ((float) val)/((float) maxi); // 63 = 0b111111
  }
  result += redPart  <<11;
  result += greenPart<<5;
  return result;
}

void batteryBar() {
  byte vccVal = readVcc();
  oled.fillRect(oled.width()-7, oled.height()  - batLength+2, 6, batLength-3-vccVal, BLACK);
  for (int v=vccVal; v>0; --v) {
    oled.drawLine(
      oled.width()-7, oled.height()-v-1,
      oled.width()-2, oled.height()-v-1,
      green2red(v, batLength)
    );
  }
}
void eyes() {
  oled.fillCircle(EYE1x, EYE1y, 8, WHITE);
  oled.drawCircle(EYE1x, EYE1y, 8, BLACK);
  oled.fillCircle(EYE2x, EYE2y, 8, WHITE);
  oled.drawCircle(EYE2x, EYE2y, 8, BLACK);  
}

void nose() {
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+7, 4, RED);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+7, 4, BLACK);  
}

void mouth() {
  int ypos = 44 + map(AcX, -7200, 32000, 0, 6);
  if (ypos>50) ypos=50;
  if (ypos<44) ypos=44;
  oled.fillRect(EYE1x, 43, 21, 10, WHITE);
  
  oled.drawLine(EYE1x, ypos,   EYE1x+5, 48, BLACK);
  oled.drawLine(EYE1x, ypos+1, EYE1x+5, 49, BLACK);
    
  oled.fillRect(EYE1x+5, 48, 10, 2, BLACK);  
  
  oled.drawLine(EYE1x+20, ypos,   EYE1x+15, 48, BLACK);
  oled.drawLine(EYE1x+20, ypos+1, EYE1x+15, 49, BLACK);
  
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(STEPLED, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  oled.begin();
  oled.fillScreen(GREY);
  oled.drawLine(0, 47, 16-treshold, 47, MAGENTA);

  // battery
  oled.drawPixel(oled.width()-5, oled.height() - batLength, WHITE);
  oled.drawPixel(oled.width()-4, oled.height() - batLength, WHITE);
  oled.drawRect(oled.width()-8, oled.height()  - batLength+1, 8, batLength-1, WHITE);  

  // kopf
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+18, 28, LIGHTBLUE);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+18, 28, BLACK);
  // schnautze
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+25, 23, WHITE);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+25, 23, BLACK);

  eyes();
  nose();

  // nasenfalte
  oled.drawLine(EYE1x + (EYE2x-EYE1x)/2, EYE2y+14, EYE1x +2 + (EYE2x-EYE1x)/2, EYE2y+26, BLACK);

  // bart
  oled.drawLine(EYE1x-4, EYE1y+14, EYE1x+5, EYE1y+16, BLACK);
  oled.drawLine(EYE1x-6, EYE1y+17, EYE1x+6, EYE1y+20, BLACK);
  oled.drawLine(EYE1x-9, EYE1y+19, EYE1x+3, EYE1y+23, BLACK);

  oled.drawLine(EYE2x+4, EYE2y+14, EYE2x-5, EYE2y+16, BLACK);
  oled.drawLine(EYE2x+6, EYE2y+17, EYE2x-6, EYE2y+20, BLACK);
  oled.drawLine(EYE2x+9, EYE2y+19, EYE2x-3, EYE2y+23, BLACK);

  mouth();

  // halsband
  oled.fillRect(EYE1x-14, 54, 47, 5, RED);
  oled.drawRect(EYE1x-14, 54, 47, 5, BLACK);
  oled.fillRect(EYE1x-15, 59, 50, 8, GREY);

  // glocke
  oled.fillCircle(EYE1x +2+ (EYE2x-EYE1x)/2, 59, 4, YELLOW);
  oled.drawCircle(EYE1x +2+ (EYE2x-EYE1x)/2, 59, 4, BLACK);
  oled.drawLine(EYE1x + (EYE2x-EYE1x)/2, 59, EYE1x +4 + (EYE2x-EYE1x)/2, 60, BLACK);

  oled.setTextColor(WHITE, GREY);
  oled.setTextSize(1);
}

void loop() {
  delay(225);
  digitalWrite(STEPLED, LOW);
  
  tick++;  
  if (tick>3) {
    seconds++;
    tick=0;
  }
  if (seconds > 59) {
    minutes += seconds / 60;
    seconds  = seconds % 60;
  }
  if (minutes > 59) {
    hours  += minutes / 60;
    minutes = minutes % 60;
  }
  if (hours > 23) {
    hours = hours % 24;
  }
  if (tick == 0) {
    oled.setTextSize(1);
    oled.setCursor(0,0);
    if(hours<10) oled.print('0');
    oled.print(hours);
    oled.print(':');
    if(minutes<10) oled.print('0');
    oled.print(minutes);
    oled.print(':');
    if(seconds<10) oled.print('0');
    oled.print(seconds);

    //oled.setCursor(75,0);
    //oled.print(dT);
  }
  
  if (digitalRead(BUTTON) == LOW) {
    treshold++;
    if (treshold>16) treshold=1;
    oled.drawLine(0, 47, 16,          47, GREY);
    oled.drawLine(0, 47, 16-treshold, 47, MAGENTA);
  }

  if (seconds%4 == 0 && tick==0) {
    batteryBar();
    eyes();
    delay(100);
    nose();
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //dT = ( (double) Tmp + 12412.0) / 340.0;
  
  stoss = ((long)AcX)*((long)AcX) + ((long)AcY)*((long)AcY) + ((long)AcZ)*((long)AcZ);
  mouth();
  
  barrier = treshold*40000000l + 260000000l;
  if (stoss > barrier) {
    steps++;
    oled.setTextSize(2);
    oled.setCursor(0,50);
    oled.print(steps);
    digitalWrite(STEPLED, HIGH);
  }
    
  if (GyX > 17200) GyX = 17200;
  if (GyY > 17200) GyY = 17200;
  if (GyX < -17200) GyX = -17200;
  if (GyY < -17200) GyY = -17200;

  deltax = map(GyX, -17200, 17200, -6, 6);
  deltay = map(GyY, -17200, 17200, 6, -6);

  oled.fillCircle(EYE1x+lastx, EYE1y+lasty, 2, WHITE);
  oled.fillCircle(EYE2x+lastx, EYE2y+lasty, 2, WHITE);
  
  oled.fillCircle(EYE1x+deltax, EYE1y+deltay, 2, BLACK);
  oled.drawPixel(EYE1x+deltax-1, EYE1y+deltay-1, WHITE);

  oled.fillCircle(EYE2x+deltax, EYE2y+deltay, 2, BLACK);
  oled.drawPixel(EYE2x+deltax-1, EYE2y+deltay-1, WHITE);
  
  lastx = deltax;
  lasty = deltay;  
}
