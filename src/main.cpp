#include <Arduino.h>

const int LED = 2;

void setup() {
    pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);   // Turn the RGB LED white
  delay(1000);
  digitalWrite(LED, LOW);    // Turn the RGB LED off
  delay(1000);}