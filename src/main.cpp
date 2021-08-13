#include <Arduino.h>
#include <PWMServo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
#include <SPI.h>


// * Setup Servos
int servoYPin = 4;  // Set Servo Pin
PWMServo ServoY;      // Create Servo Object
int servoZPin = 5;  
PWMServo ServoZ;      
//

// Setup Piezo Buzzer
int piezoPin = 34;
unsigned long prevMillis = 0;
//

// Setup Button
int buttonPin = 7;
int buttonState;
int prevButtonState;
bool systemState = false;
//

// Setup SD Card
//

// Setup Barometer
double seaPressure = 1013.25;
Adafruit_BMP3XX bmp;
//

// Setup IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Create IMU Object
//

// Setup PID Controllers
double Setpoint;
double InputY, InputZ;
double OutputY, OutputZ;

PID PIDy(&InputY, &OutputY, &Setpoint, 1, 0, 0, DIRECT); // PID Y //TODO: Add variables to PID values.
PID PIDz(&InputZ, &OutputZ, &Setpoint, 1, 0, 0, DIRECT); // PID Z
//

void setup() {

  pinMode(3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  digitalWrite(3, HIGH);

  // Initialize Servos
  ServoY.attach(servoYPin);
  ServoZ.attach(servoZPin);
  //

  // Initialize Serial Readout
  Serial.begin(9600);
  //

  // Initialize Button
  pinMode(buttonPin, INPUT_PULLUP);
  //

  // Initialize IMU
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  if(!bno.begin()){
    Serial.print("** BNO055 not detected **\t\t<<");
    while(1);
  }
  delay(1000); //TODO: Why this delay?
  bno.setExtCrystalUse(true);
  //

  // Initialize Barometer
  if (!bmp.begin_I2C()){
    Serial.println("** BMP388 not detected **\t\t<<");
    while(1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_1_5_HZ);
  //

  // Initialize PID
  InputY = 0;
  InputZ = 0;
  Setpoint = 0;

  PIDy.SetMode(AUTOMATIC); // Turn PID on
  PIDz.SetMode(AUTOMATIC); 
  PIDy.SetOutputLimits(-255,255);
  PIDz.SetOutputLimits(-255,255); // Allow Negative Outputs
  PIDy.SetSampleTime(25);
  PIDz.SetSampleTime(25); // Increase update frequency (default 200)
  //


  // Initialize SD Card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    while (true);
  }
  Serial.println("initialization done.");
}


void loop() {

int servoHome = 90; // Set Home Position of servo (degrees)

buttonState = digitalRead(buttonPin);
if((buttonState != prevButtonState) && (buttonState == HIGH)) {
  systemState = !systemState;
  if (systemState == true){
    tone(piezoPin, 3000, 100);
    delay(100);
    tone(piezoPin, 3000, 100);
    delay(100);
    tone(piezoPin, 6000, 800);
    
  }  // TODO: this is messed up below but it works
  if (systemState == false){
    tone(piezoPin, 8000, 800);
    delay(100);
    tone(piezoPin, 4000, 100);
    delay(100);
    tone(piezoPin, 4000, 100);
  }
}
prevButtonState = buttonState;
Serial.print(systemState);
Serial.print("\t\t");


if(systemState == true){
// Get new sensor event
sensors_event_t event;
bno.getEvent(&event);
//
digitalWrite(A2, HIGH);
// Update inputs & compute PID
InputY = event.orientation.y;
InputZ = event.orientation.z;

PIDy.Compute();
PIDz.Compute();
//

// Move Servos based on output
if(OutputY > 10){
  ServoY.write(servoHome+20);
} else if(OutputY < -10){
  ServoY.write(servoHome-20);
} else {
  ServoY.write(servoHome+(OutputY*2));
}

if(OutputZ > 10){
  ServoZ.write(servoHome+20);
} else if(OutputZ < -10){
  ServoZ.write(servoHome-20);
} else {
  ServoZ.write(servoHome+(OutputZ*2));
}
//

// Piezo Buzzer
unsigned int interval = 1000;
if((millis()-prevMillis) > interval){
  prevMillis = millis();
  tone(piezoPin, 4000, 100);
} 
//

float Xorient = event.orientation.x;

// Log Data to SD Card
File logFile = SD.open("flightData.txt", FILE_WRITE);
  if (logFile) {
    logFile.print(millis());
    logFile.print("\t\t");
    logFile.print(Xorient,4);
    logFile.print(InputY,4);
    logFile.print(InputZ,4);
    logFile.print(OutputY,4);
    logFile.print(OutputZ,4);
    logFile.print(bmp.temperature);
    logFile.print(bmp.pressure / 100.0);
    logFile.print(bmp.readAltitude(seaPressure));
    logFile.println("");
    logFile.close();
  }

// Display FP data       (Parts commented out because they interfere with performance)
Serial.print(millis());

//Serial.print("\t\tX: ");
Serial.print(Xorient,4);
//Serial.print("\tY: ");
Serial.print(InputY,4);
//Serial.print("\tZ: ");
Serial.print(InputZ,4);
//Serial.print("\t\tOutput Y: ");
Serial.print(OutputY,4);
//Serial.print("\t\tOutput Z: ");
Serial.print(OutputZ,4);

//Serial.print("\tTemperature = ");
  Serial.print(bmp.temperature);
  //Serial.print(" *C");

  //Serial.print("\tPressure = ");
  Serial.print(bmp.pressure / 100.0);
  //Serial.print(" hPa");

  //Serial.print("\tApprox. Altitude = ");
  Serial.print(bmp.readAltitude(seaPressure));
  //Serial.print(" m");
Serial.println("");

} else {
  delay(25);
  digitalWrite(A2, LOW);
}
//  delay(25);


}