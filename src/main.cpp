/*
Improvements I would implement if more time
(1) Better debouncing to accurately switch machine states
(2) Interrupt over I2C for the built in button on the encoder
(3) Involve stepper motor for the 4 bar mechanism. The main issue was speed and direction
(4) A significantly better menu screen to better choose machine state
(5) A more accurate pulse count. The current approach seems to drift away a bit but works
(6) Added functionality of the NeoPixel, such as blinking yellow to indicate that its waiting or moving
(7) Better manual control. Current is insensitive in the beggining but then quicly speeds up. Goes along with better pulse count
(8) More machine states such as a time and clock (Wifi) for added functionality
(9) More data displayed on LCD screen such as current position.
*/

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include "SCMDMotor.h"

// LCD Screen Library
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Servo Motor Library
#include <SCMD.h>
#include <stdint.h>
#include <SCMD_config.h>

// ----------------- Servo Motor --------------
int servoLimitSW = A0;
int encoderA = 10;
int encoderB = 11;
int motorSpeed = 255;
#define MOTOR 1

// Motor: ID=1, encoder pins 10 and 11
SCMDMotor servo(MOTOR, encoderA, encoderB);

// ---------- Rotary Encoder, Push Button, and NeoPixel ---------
#define SS_ADDR 0x36          // Seesaw I2C address for Adafruit 5880
int SS_ENCODER_INT = 2;
int SS_NEOPIX_PIN = 6;
int rotaryEncoder = 0;

int switchPin = 12;      // Button pin

Adafruit_seesaw ss;
seesaw_NeoPixel pixel = seesaw_NeoPixel(1, SS_NEOPIX_PIN, NEO_GRB + NEO_KHZ800);

// ------------------ LCD ----------------------
// LCD Pins (modify if different)
int CS_Pin = 6;
int RST_pin = 5;
int DC_Pin = 9;

Adafruit_ST7735 tft = Adafruit_ST7735(CS_Pin, DC_Pin, RST_pin);

enum machineStates {
  offState,
  homeServo,
  //normalOperation,
  manualControl,
  stateLength,
};

volatile machineStates currentState = offState;
// ------------- Double press -------------
enum PressType {
  NoPress, // 0
  SinglePress, //1 
  DoublePress, //2
  LongPress,
};

volatile PressType currentPress = NoPress;

unsigned long prevTime = 0;
unsigned long debounceDelay = 150;
unsigned long doublePressTime = 500;
bool prevPressed = false;
unsigned long timePressed;


void buttonPress() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime > debounceDelay) {
    prevPressed = true;
    timePressed = currentTime;
    if (currentTime - prevTime < doublePressTime) {
      currentPress = DoublePress;
    } 
    else {//if (currentTime - prevTime > doublePressTime) {
      currentPress = SinglePress;
    }
  }
  prevTime = currentTime;
}

void setup() {
  //delay(1000);
  //Serial.begin(9600);
  delay(1000);
  Wire.begin();

  // ---------- Seesaw Begin --------------
  if (!ss.begin(SS_ADDR)) {
    delay(1);
  }
  pinMode(switchPin, INPUT_PULLDOWN);
  attachInterrupt(switchPin, buttonPress, CHANGE);

  while (!pixel.begin(SS_ADDR)) {
    delay(1);
  }
  pixel.setBrightness(255);

  // ------------- LCD -------------------
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(2);

  // -------------- Servo ---------------
  servo.begin();
}

void loop() {
  if (currentPress == DoublePress ) { //&& currentState != OffState
    currentState = (machineStates)(((int)currentState + 1) % (int)stateLength);
    currentState = (machineStates)max((int)currentState, 1);
  }
  else if (currentPress == LongPress) {
    currentState = (machineStates)offState;
  }

  if (currentPress != NoPress) {
    currentPress = NoPress;
  }
  
  if (digitalRead(switchPin) && prevPressed) {
    if ((millis() - timePressed) >= 3000) {
      currentPress = LongPress;
      prevPressed = false;
    }
  }
  
  switch (currentState) {
    case offState:
    tft.fillScreen(ST77XX_BLACK);
    //                                  R   G  B
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    break;
    case homeServo:
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      delay(10);
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(40,20);
      tft.print("Home");
      if (currentPress == SinglePress) {
        servo.startHome(servoLimitSW);
      }
    break;
    case manualControl:
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      delay(10);
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(25,25);
      tft.print("Manual");
      tft.setCursor(23,40);
      tft.print("Control");
      rotaryEncoder = ss.getEncoderPosition();
      servo.startMoveTo(5 * rotaryEncoder);
    break;
    default:
    break;
  }
  servo.update();
  //delay(50);
  
}