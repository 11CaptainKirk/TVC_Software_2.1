void blinks(int led, int delayMS){
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delayMS);                       // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(delayMS);                       // wait for a second
}