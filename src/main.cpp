#include <Arduino.h>
#include "blink.h"

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  blinks(LED_BUILTIN, 200);                     // wait for a second
}