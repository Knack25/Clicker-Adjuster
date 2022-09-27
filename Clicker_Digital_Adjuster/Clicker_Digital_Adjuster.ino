
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_LC709203F.h"
#include "Button2.h" //  https://github.com/LennartHennigs/Button2
#include "ESPRotary.h"
#include <Preferences.h>
#include <Adafruit_MotorShield.h>
#include <sstream>
#include <iostream>
#include <string>

using namespace std;

//These pins define the physical pins to connect the encoder and button
#define ROTARY_PIN1  A3
#define ROTARY_PIN2 A2
#define BUTTON_PIN  A1


//declare the storage object
Preferences preferences;

// declare the encoder and button
ESPRotary r;
Button2 b;

#define CLICKS_PER_STEP   4   // this number depends on your rotary encoder 


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);


//connect to the charge controller
Adafruit_LC709203F lc;



//-------------------------------------------Var Decrlaration------------------------------------
double setup_pos = 600;
int stepSize = 5;

double i = setup_pos;
double motor_pos = 0;
double file_Motor_Pos = 0;
bool enable = true;
bool inSetup = true;
bool skipInit = false;
bool inInit = true;
String text;


// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


// on rotation change
void rotate(ESPRotary& r) {
  //some debug data
  Serial.print("Batt Voltage: ");
  Serial.println(lc.cellVoltage());
  Serial.print("Batt Percent: ");
  Serial.println(lc.cellPercent());
  Serial.print("Batt Temp: ");
  Serial.println(lc.getCellTemperature());

  //If we are in running mode
  if (!inSetup) {
    //if we are enabled
    if (enable) {
      Serial.println(r.getPosition());
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      //if the encoder was rotated left
      if (r.directionToString(r.getDirection()) == "LEFT") {
        if (i > 0) {
          i = i - stepSize;
        }
      }
      //if the encoder was rotated right
      if (r.directionToString(r.getDirection()) == "RIGHT") {
        i = i + stepSize;
      }
      tft.println(i);
    }
    //if not enabled
    else {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.println(i);
      tft.println("Controlls Locked.");
      tft.println("Long press to unlock");
      myMotor->release();
    }
  }
  //if we are in setup...
  else {
    //and we havent fully initialized...
    if (inInit) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      //if the encoder was rotated left
      if (r.directionToString(r.getDirection()) == "LEFT") {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0, 0);
        skipInit = true;
        tft.println("Previous Settings found:");
        tft.println();
        tft.println("Resume");

      }
      //if the encoder was rotated right
      if (r.directionToString(r.getDirection()) == "RIGHT") {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0, 0);
        skipInit = false;
        tft.println("Previous Settings found:");
        tft.println();
        tft.println("Re-Initialize Count");
      }
    }
  }
}


// on left or right rotation
void showDirection(ESPRotary& r) {
  Serial.println(r.directionToString(r.getDirection()));
}

// single click
void click(Button2& btn) {
  //if we are in running mode
  if (!inSetup) {
    //when the button is pushed, move the motor to match the input
    if (motor_pos < i) {
      myMotor->step((i - motor_pos), FORWARD, DOUBLE);
      motor_pos = i;
    }
    if (motor_pos > i) {
      myMotor->step((motor_pos - i), BACKWARD, DOUBLE);
      motor_pos = i;
    }
    //save the most recent input
    preferences.begin("Clicker_Data", false);
    Serial.println("Wrote: ");
    Serial.println(motor_pos);
    preferences.putDouble("motor_pos", motor_pos);
    myMotor->release();
  }
  //if we are in setup
  if (inSetup) {
    //and we want to skip the Init
    if (skipInit) {
      motor_pos = file_Motor_Pos;
      i = file_Motor_Pos;
      inSetup = false;
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextSize(3);
      tft.setCursor(0, 0);
      tft.println(i);
    }
    //and we don't want to skip the Init
    if (!skipInit) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.println("Setting up motor.");
      myMotor->step(i - motor_pos, FORWARD, DOUBLE);
      motor_pos = i;
      inSetup = false;
      myMotor->setSpeed(10);  // 10 rpm
      myMotor->release();
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.setTextSize(3);
      tft.println(i);
    }
  }
}

// long click
void resetPosition(Button2& btn) {
  //switch the enabled state
  enable = !enable;
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.println(i);

  //if we are disabled
  if (!enable) {
    myMotor->release();
    tft.println("Controlls Locked.");
    tft.println("Long press to unlock");
  }
}




void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  //------------------------------------------Encoder Init---------------------------------------

  r.begin(ROTARY_PIN1, ROTARY_PIN2, CLICKS_PER_STEP);
  r.setChangedHandler(rotate);
  r.setLeftRotationHandler(showDirection);
  r.setRightRotationHandler(showDirection);

  b.begin(BUTTON_PIN);
  b.setTapHandler(click);
  b.setLongClickHandler(resetPosition);

  //--------------------------------------------Battery Init-----------------------------------------
  // For the Feather ESP32-S2, we need to enable I2C power first!
  // this section can be deleted for other boards
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
#endif

  if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

  lc.setThermistorB(3950);
  Serial.print("Thermistor B = "); Serial.println(lc.getThermistorB());

  lc.setPackSize(LC709203F_APA_500MAH);

  lc.setAlarmVoltage(3.8);



  //------------------------------------------------------TFT Init-------------------------------------------

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  Serial.println(F("Initialized"));

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text



  text = "When life gives you lemons, don't make lemonade. Get mad. Also the cake is a lie.";
  tft.fillScreen(ST77XX_BLACK);
  tft.println(text);
  delay(1000);



  tft.fillScreen(ST77XX_BLACK);

  //------------------------------Motor Init-------------------------------


  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  myMotor->setSpeed(40);  // 40 rpm


  myMotor->release();
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);


  Serial.println("Reading in from File");
  preferences.begin("Clicker_Data", false);
  file_Motor_Pos = preferences.getDouble("motor_pos", 0);
  Serial.println("Read in:");
  Serial.println(file_Motor_Pos);



  if (file_Motor_Pos == 0) {
    inInit = false;
    tft.println("Initial Setup:");
    tft.println("1. Ensure knob is fully raised.");
    tft.println("2. Press button to confirm");
  } else {
    tft.println("Previous Settings found:");
    tft.println(file_Motor_Pos);
    tft.println("Resume");
    skipInit = true;
  }

}





void loop() {

  r.loop();
  b.loop();
}
