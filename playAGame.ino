
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//#define ALLPASS
//#define PASSDANGER
#define OUTPUT_READABLE_ACCELGYRO

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

HMC5883L mag;
int16_t mx, my, mz;

int state = LOW; 
unsigned long previousMillis = 0;
const long safeInterval = 3000;
const long roundInterval = 5000;

const int redPin = 10;
const int greenPin = 5;
const int bluePin = 6;

void cls(){
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}

void showMsg(String label, double value){
  static int count = 0;
  
  if(count == 50){
    Serial.print(label);
    Serial.println(value);
    count = 0;
  } else count ++;
}

void setColor(int r, int g, int b){
  int red = 255 - constrain(r, 0, 255);
  int green = 255 - constrain(g, 0, 255);
  int blue = 255 - constrain(b, 0, 255);
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

const int buzzlePin = 7;

void countDown(){
  Serial.print("Start in: 5 ");
  delay(1000); Serial.print("4 ");
  delay(1000); Serial.print("3 ");
  delay(1000); Serial.print("2 ");
  delay(1000); Serial.println("1 ");
}

void beep4ever(){
  digitalWrite(buzzlePin, HIGH);
  while(true){}
}

void beepboop(){
  static long _pMillis = millis();
  static int buzzleState = LOW;
  
  unsigned long _cMillis = millis();
  if(_cMillis - _pMillis >= 250) {
    if(buzzleState == LOW) {
      buzzleState = HIGH;
      //Serial.println("boop");
    } else {
      buzzleState = LOW;
      //Serial.println("beep");
    }
    _pMillis = _cMillis;
    digitalWrite(buzzlePin, buzzleState);
  }
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  Serial.begin(38400);
  
  accelgyro.initialize();
  mag.initialize();
  randomSeed(millis());
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  pinMode(buzzlePin, OUTPUT);
  setColor(255,255,255);
  delay(3000);
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  setColor(0, 255, 0);
}

void loop() {
  Serial.print("Quest:\t");
  
  // for shaking
  float initMotion;
  
  // for rotate 20
  float initHeading;
  
  int choice = random(0, 4);
  switch(choice){
    case 0:
      Serial.println("Shaking"); // MPU
      break;
    case 1:
      Serial.println("Heading to North"); // HMC
      break;
    case 2:
      Serial.println("Rotate more than 20 degrees"); // HMC
      break;
    case 3:
      Serial.println("Turn to bottom"); // HMC
      break;
  }
  
  countDown();
  
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      initMotion = sqrt(square(ax) + square(ay) + square(az));
      
      mag.getHeading(&mx, &my, &mz);
      initHeading = atan2(my, mx);
      if(initHeading < 0) initHeading += 2 * M_PI;
      initHeading = initHeading * 180/M_PI;
      
  //bool solved = false;
  #define S_SAVE 0
  #define S_DANGER 1
  #define S_SOLVED 2 
  #define S_GAMEOVER 3
  int state = 0;
  unsigned long startMillis = millis();
  
  while(true) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);
    float nowHeading = atan2(my, mx);
    if(nowHeading < 0) nowHeading += 2 * M_PI;
    nowHeading = nowHeading * 180/M_PI;
    
    float nowMotion = sqrt(square(ax) + square(ay) + square(az));
    
    unsigned long currentMillis = millis();
    
    switch(choice){
    case 0:
      showMsg("Shake value: ", fabs(nowMotion - initMotion));
      if(fabs(nowMotion - initMotion) > 2000) {
        state = S_SOLVED;
      }
      break;
    case 1:
      showMsg("Heading value: ", nowHeading);
      if((nowHeading >= 0 && nowHeading <= 45) || (nowHeading >= 315 && nowHeading <= 360)) {
        state = S_SOLVED;
      }
      break;
    case 2:
      showMsg("Rotate Value: ", fabs(nowHeading - initHeading));
      if(fabs(nowHeading - initHeading) > 20) {
        state = S_SOLVED;
      }
      break;
    case 3:
      showMsg("Z value: ", az);
      if(az <0) {
        state = S_SOLVED;
      }
      break;
    }
    
    #ifdef ALLPASS
      state = S_SOLVED;
    #endif
    
    if(state == S_SOLVED) {
      Serial.println("solved");
      break;
    }
    
    if(currentMillis - startMillis >= safeInterval && currentMillis - startMillis < roundInterval) {
      state = S_DANGER;
    }
    if(currentMillis - startMillis >= roundInterval) {
      #ifdef PASSDANGER
        state = S_SOLVED;
      #endif
      
      if(state == S_SOLVED) {
        goto stateAction;
      } else {
        state = S_GAMEOVER;
        goto stateAction;
      }
    }

stateAction:
    switch(state){
      case S_SAVE:
        setColor(0, 255, 0);
        digitalWrite(buzzlePin, LOW);
        break;
      case S_DANGER:
        setColor(255, 0, 0);
        beepboop();
        break;
      case S_SOLVED:
        setColor(0, 255, 0);
        digitalWrite(buzzlePin, LOW);
        cls();
        break;
      case S_GAMEOVER:
        setColor(255, 0, 0);
        beep4ever();
        cls();
        Serial.println("--------------- GAME OVER -----------------");
        break;
    }
  }
}

