/////////////////////////////////////////////////////////////////

#include "ESPRotary.h"

/////////////////////////////////////////////////////////////////

#define ROTARY_PIN1	14
#define ROTARY_PIN2	12

/////////////////////////////////////////////////////////////////

ESPRotary r;

/////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  delay(50);
  Serial.println("\n\nSimple Counter");
  
  r.begin(ROTARY_PIN1, ROTARY_PIN2);
  r.setChangedHandler(rotate);
  r.setLeftRotationHandler(showDirection);
  r.setRightRotationHandler(showDirection);
}

void loop() {
  r.loop();
}

/////////////////////////////////////////////////////////////////

// on change
void rotate(ESPRotary& r) {
   Serial.println(r.getPosition());
}

// on left or right rotattion
void showDirection(ESPRotary& r) {
  Serial.println(r.directionToString(r.getDirection()));
}

/////////////////////////////////////////////////////////////////
