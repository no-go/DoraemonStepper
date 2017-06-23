#include <Arduino.h>

// -------------------------------------------Config Start
const int MPU=0x68;  // I2C address of the MPU-6050

#define OLED_DC      5
#define OLED_CS     12
#define OLED_RESET   6

#define BUTTON      11
#define CHIPINT     10  //unused
#define VBATPIN     A7  // A7 = D9 !!
#define STEPLED     13

#define VCCMAX 4400
#define VCCMIN 3550

long treshold = 2; //initial

byte batLength = 40;
int  DIMSEC = 15;
int  OFFSEC = 45;

#define EYE1x 47
#define EYE1y 22
#define EYE2x 64
#define EYE2y 22
// -------------------------------------------Config End

// git clone https://github.com/adafruit/Adafruit_ASFcore
// git clone https://github.com/adafruit/Adafruit_ZeroTimer.git
#include <Adafruit_ZeroTimer.h>

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1331.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

#include <SPI.h>
#include <Wire.h>

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

long stoss;
long firstStoss =-1;
int vccVal;

long barrier = 0;
long steps   = 0;

int deltax,deltay;
int lastx=0;
int lasty=0;

byte tick = 0;
byte hours   = 0;
byte minutes = 0;
byte seconds = 0;
int displayOnSec = 0;
int messageOnSec = -1;
byte pressTicks = 0;

char serialCache[11] = "00:00:00  ";
int cpos = 0;

// 6x16 zeichen:
#define BUFLEN 96
int countX = 0;
int countY = 0;
bool contin;
unsigned short swipp = 0;

// The temperature sensor is -40 to +85 degrees Celsius.
// It is a signed integer.
// According to the datasheet: 
//   340 per degrees Celsius, -512 at 35 degrees.
// At 0 degrees: -512 - (340 * 35) = -12412
double dT; // unused

// Color definitions
#define BLACK           0x0000
#define GREY            0b0001000010000010
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

#define BACKGROUND      0x0000

#define NO_TIME_THERE 42

Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);
Adafruit_SSD1331 oled = Adafruit_SSD1331(OLED_CS, OLED_DC, OLED_RESET);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void readVcc() {
  float mv = analogRead(VBATPIN);
  mv *= 2;
  mv *= 3.3;
  vccVal = mv;
  if (vccVal > VCCMAX) vccVal = VCCMAX;
  if (vccVal < VCCMIN) vccVal = VCCMIN;
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

void scroll() {
  unsigned short oldSwipp = swipp;
  for (int i=1; i<=10; ++i) {
    oled.fillRect(0, oldSwipp, oled.width(), i, BLACK);
    oled.writeCommand(SSD1331_CMD_DISPLAYOFFSET);
    oled.writeCommand(swipp);
    swipp++;
    delay(50);
  }
}

char umlReplace(char inChar) {
  if (inChar == 159) {
    inChar = 224; // ß
  } else if (inChar == 164) {
    inChar = 132; // ä
  } else if (inChar == 182) {
    inChar = 148; // ö
  } else if (inChar == 188) {
    inChar = 129; // ü
  } else if (inChar == 132) {
    inChar = 142; // Ä
  } else if (inChar == 150) {
    inChar = 153; // Ö
  } else if (inChar == 156) {
    inChar = 154; // Ü
  } else if (inChar == 171) {
    inChar = 0xAE; // <<
  } else if (inChar == 187) {
    inChar = 0xAF; // >>
  }  
  return inChar;  
}

void batteryBar() {
  readVcc();
  int val = map(vccVal, VCCMIN, VCCMAX, 0, batLength);
  oled.fillRect(oled.width()-5, oled.height() - batLength+2, 4, batLength-3-val, GREY);
  for (int v=val; v>0; --v) {
    oled.drawLine(
      oled.width()-5, oled.height()-v-1,
      oled.width()-2, oled.height()-v-1,
      green2red(v, batLength)
    );
  }
}

inline void treBar() {
  oled.drawLine(0, 52, 18, 52, GREEN);
  oled.drawLine(0, 52, 20-(2*treshold), 52, AMBER);
  oled.drawPixel(20-(2*treshold), 52, WHITE);
}

inline void eyes() {
  oled.fillCircle(EYE1x, EYE1y, 8, WHITE);
  oled.drawCircle(EYE1x, EYE1y, 8, BLACK);
  oled.fillCircle(EYE2x, EYE2y, 8, WHITE);
  oled.drawCircle(EYE2x, EYE2y, 8, BLACK);  
}

inline void batteryFrame() {
  oled.drawPixel(oled.width()-4, oled.height() - batLength, WHITE);
  oled.drawPixel(oled.width()-3, oled.height() - batLength, WHITE);
  oled.drawRect(oled.width()-6, oled.height()  - batLength+1, 6, batLength-1, WHITE);  
}

inline void kopf() {
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+18, 28, LIGHTBLUE);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+18, 28, BLACK);
}

inline void schnautze() {
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+25, 23, WHITE);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+25, 23, BLACK);
}

inline void nose() {
  oled.fillCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+7, 4, RED);
  oled.drawCircle(EYE1x + (EYE2x-EYE1x)/2, EYE2y+7, 4, BLACK);  
}

inline void mouth() {
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

inline void nasenfalte() {
  oled.drawLine(EYE1x + (EYE2x-EYE1x)/2, EYE2y+14, EYE1x +2 + (EYE2x-EYE1x)/2, EYE2y+26, BLACK);
}

inline void bart() {
  oled.drawLine(EYE1x-4,  EYE1y+14, EYE1x+5, EYE1y+16, BLACK);
  oled.drawLine(EYE1x-6,  EYE1y+17, EYE1x+6, EYE1y+20, BLACK);
  oled.drawLine(EYE1x-11, EYE1y+19, EYE1x+3, EYE1y+23, BLACK);

  oled.drawLine(EYE2x+4,  EYE2y+14, EYE2x-5, EYE2y+16, BLACK);
  oled.drawLine(EYE2x+6,  EYE2y+17, EYE2x-6, EYE2y+20, BLACK);
  oled.drawLine(EYE2x+11, EYE2y+19, EYE2x-3, EYE2y+23, BLACK);
}

inline void halsband() {
  oled.fillRect(EYE1x-14, 54, 47, 5, RED);
  oled.drawRect(EYE1x-14, 54, 47, 5, BLACK);
  oled.fillRect(EYE1x-15, 59, 50, 8, BACKGROUND);
}

inline void glocke() {
  oled.fillCircle(EYE1x +2+ (EYE2x-EYE1x)/2, 59, 4, YELLOW);
  oled.drawCircle(EYE1x +2+ (EYE2x-EYE1x)/2, 59, 4, BLACK);
  oled.drawLine(EYE1x + (EYE2x-EYE1x)/2, 59, EYE1x +4 + (EYE2x-EYE1x)/2, 60, BLACK);
}

inline void printClock() {
  oled.setTextColor(YELLOW, BACKGROUND);
  oled.setTextSize(2);
  oled.setCursor(0,0);
  if(hours<10) oled.print('0');
  oled.print(hours);
  oled.print(':');
  if(minutes<10) oled.print('0');
  oled.print(minutes);
  oled.print(':');
  if(seconds<10) oled.print('0');
  oled.print(seconds);
}

void Timer3Callback0(struct tc_module *const module_inst) {
  tick++;
  if (tick>3) {
    seconds++;
    if (displayOnSec >=0) displayOnSec++;
    if (messageOnSec >=0) messageOnSec++;
    if (messageOnSec > OFFSEC) messageOnSec = -1;
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
}

inline int16_t absi(int16_t val) {
  if (val<0) return -1*val;
  else val;
}

inline byte tob(char c) { return c - '0';}

void getTime() {
  byte i = NO_TIME_THERE;
  if (serialCache[2] == ':' && serialCache[5] == ':') i=0;
  if (serialCache[3] == ':' && serialCache[6] == ':') i=1;
  if (i == NO_TIME_THERE) return;
  hours = tob(serialCache[i])*10 + tob(serialCache[1+i]);
  minutes = tob(serialCache[3+i])*10 + tob(serialCache[4+i]);
  seconds = tob(serialCache[6+i])*10 + tob(serialCache[7+i]);
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(STEPLED, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  ble.begin(false);
  ble.echo(false);
  ble.sendCommandCheckOK("AT+HWModeLED=BLEUART");
  ble.sendCommandCheckOK("AT+GAPDEVNAME=DoraemonStep");
  ble.sendCommandCheckOK("ATE=0");
  ble.sendCommandCheckOK("AT+BAUDRATE=115200");
  ble.sendCommandCheckOK("ATZ");
  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.verbose(false);

  oled.begin();
  
  oled.fillScreen(BACKGROUND);
  treBar();
  batteryFrame();
  
  kopf();
  schnautze();
  eyes();
  nose();
  nasenfalte();
  bart();
  mouth();
  halsband();
  glocke();

  oled.setTextColor(GREEN, BACKGROUND);
  oled.setTextSize(1);
  
  /********************* Timer #3, 16 bit, two PWM outs, period = 65535 */
  zt3.configure(TC_CLOCK_PRESCALER_DIV1024, // prescaler
                TC_COUNTER_SIZE_16BIT,   // bit width of timer/counter
                TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode 
                );

  //zt3.setPeriodMatch(48000, 1, 0); // 48MHz / 1024   ->   48 = 1ms (to slow. 1s = 1.024064)
  zt3.setPeriodMatch(11712, 1, 0); // 250ms
  zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer3Callback0);  // this one sets pin low
  zt3.enable(true);
}

void loop() {
  delay(100);
  digitalWrite(STEPLED, LOW);
  
  if (ble.isConnected()) {
    while ( ble.available() ) {
      contin = false;
      messageOnSec = 0;
      displayOnSec = 0;
      oled.writeCommand(SSD1331_CMD_DISPLAYON);
      char inChar = (char) ble.read();
      if (inChar == 194) continue; // symbol before utf-8
      if (inChar == 195) continue; // symbol before utf-8

      if (inChar == '\n') {
        getTime();
        serialCache[cpos] = '\0';
        cpos = 0;
        contin = true;
        
        countY++;
        countX=0;
      } else {
        countX++;
        if(countX == 17) {
          countY++;
          countX=1;
        }      
      }
      inChar = umlReplace(inChar);

      if (contin == false) {
        serialCache[cpos] = inChar;
        cpos++;        
      }
      
      if (swipp == 0) {
        if(countY>5) {
          countX=1;
          if(inChar == '\n') countX=0;
          countY=0;
          scroll();
        }    
      } else {
        for (int j=0;j<5;++j) {
          if (swipp == 10*(j+1)) {
            if(countY>j) {
              countX=1;
              if(inChar == '\n') countX=0;
              countY=j+1;
              scroll();
            } 
          }    
        }
      }
      if (swipp == 60) {
        swipp=0;
        oled.writeCommand(SSD1331_CMD_DISPLAYOFFSET);
        oled.writeCommand(swipp);
      }
              
      oled.setTextColor(WHITE, BLACK);
      oled.setTextSize(1);
      oled.setCursor((countX-1)*6,countY*10);
      oled.print(inChar);
    }
  }
    
  if (tick == 0  && displayOnSec >= 0) {
    printClock();
    //oled.setCursor(75,0);
    //oled.print(dT);
  }
  
  if (digitalRead(BUTTON) == LOW) {
    pressTicks++;
    displayOnSec=0;
    batteryBar();
    eyes();
    nose();
    oled.writeCommand(SSD1331_CMD_DISPLAYON);

    if (pressTicks>2) {
      pressTicks=0;
      treshold++;
      if (treshold>10) treshold=1;
      treBar();
    }
  } else {
    pressTicks = -1;
  }

  if (seconds%4 == 0 && tick==0 && displayOnSec>=0 && messageOnSec < 0 && displayOnSec < DIMSEC) {
    batteryBar();
    eyes();
    delay(100);
    nose();
  }
  
  if (seconds%29 == 0 && tick==0 && messageOnSec < 0 && displayOnSec < DIMSEC) {
    oled.fillScreen(BACKGROUND);
    oled.writeCommand(SSD1331_CMD_DISPLAYOFFSET);
    oled.writeCommand(0);

    treBar();
    batteryFrame();
    batteryBar();
    
    kopf();
    schnautze();
    eyes();
    nose();
    nasenfalte();
    bart();
    mouth();
    halsband();
    glocke();
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
  
  stoss = absi(AcX) + absi(AcY) + absi(AcZ);
  
  barrier = treshold*5000l + 9000l;
  if (stoss > barrier) {
    steps++;
    digitalWrite(STEPLED, HIGH);
    if (steps%200 == 0 && ble.isConnected()) {
      ble.print("Steps: ");
      ble.println(steps);
      ble.print(vccVal);
      ble.println(" mV");    
    }
  }
    
  if (GyX > 17200) GyX = 17200;
  if (GyY > 17200) GyY = 17200;
  if (GyX < -17200) GyX = -17200;
  if (GyY < -17200) GyY = -17200;

  deltax = map(GyX, -17200, 17200, -6, 6);
  deltay = map(GyY, -17200, 17200, 6, -6);
  
  if (displayOnSec >= 0 && messageOnSec < 0 && displayOnSec < DIMSEC) {
    mouth();
    
    oled.setTextSize(1);
    oled.setCursor(0,55);
    oled.setTextColor(AMBER, BACKGROUND);
    oled.print(steps);
    
    oled.fillCircle(EYE1x+lastx, EYE1y+lasty, 2, WHITE);
    oled.fillCircle(EYE2x+lastx, EYE2y+lasty, 2, WHITE);
    
    oled.fillCircle(EYE1x+deltax, EYE1y+deltay, 2, BLACK);
    oled.drawPixel(EYE1x+deltax-1, EYE1y+deltay-1, WHITE);
  
    oled.fillCircle(EYE2x+deltax, EYE2y+deltay, 2, BLACK);
    oled.drawPixel(EYE2x+deltax-1, EYE2y+deltay-1, WHITE);  
  }

  if (displayOnSec == DIMSEC) {
    oled.writeCommand(SSD1331_CMD_DISPLAYDIM);
  }
  
  if (displayOnSec > OFFSEC) {
    oled.writeCommand(SSD1331_CMD_DISPLAYOFF);
    displayOnSec = -1;
    messageOnSec = -1;
  }
      
  lastx = deltax;
  lasty = deltay;

}

