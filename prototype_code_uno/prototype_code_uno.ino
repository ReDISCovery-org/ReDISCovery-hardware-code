#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//GPS WIRING (UNO)
//TX - pin 3
//RX - pin 2
//Vin - 5V
//GND - GND

//SD CARD WIRING (UNO)
//MOSI - pin 11
//MISO - pin 12
//CLK - pin 13
//CS - pin 4
//3.3V - 3.3V
//GND - GND

//10 DOF WIRING (UNO)
//SDA - A4
//SCL - A5
//Vin - 5V
//GND - GND


SoftwareSerial mySerial(3, 2);

//Attach GPS to SoftwareSerial
Adafruit_GPS GPS(&mySerial);

boolean showRawGPSData;
boolean usingInterrupt;


//Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
float pitch;
float roll;

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  showRawGPSData = false;
  useInterrupt(true);
  pinMode(10, OUTPUT); //Set pin 10 to output so the SD library will work
  initializeSD();
  initSensors();
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (showRawGPSData && usingInterrupt)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    //run "Compare A" function
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    //do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//initialize 10 DoF sensors
void initSensors() {
  //just initialize the accelerometer for now
  if (!accel.begin()) {
    Serial.println(F("No LSM303 Detected."));
    while(1);
  }
  //set values for pitch and roll to 0
  pitch = 0;
  roll = 0;
}

//update roll & pitch values 
void updateRollPitch() {
  sensors_event_t accel_event;
  sensors_vec_t orientation;
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    roll = orientation.roll;
    pitch = orientation.pitch;
  }
}

//tests the wireless connection to see if data can be transmitted
//set to false for now
boolean canTransmitData() {
  return false;
}

//wirelessly transmits data to an Android phone to be saved
void transmitData(String data) {
  //not implemented yet
}

//initializes the SD card
//run in the setup function
void initializeSD() {
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(4)) {
    Serial.println(F("SD Initialization Failed!"));
    return;
  }
  Serial.println(F("SD Initialized Successfully."));
}



//transmit/write all data
void transmitOrWriteData() {
  if (!canTransmitData()) {
    //open SD file and test if it is open
    boolean myFile = true;
    if (myFile) {
      //start writing GPS data to SD card
      //just prints to serial for now
      Serial.print(String((int)GPS.hour, DEC));
      Serial.print(String(':'));
      Serial.print(String((int)GPS.minute, DEC));
      Serial.print(String(':'));
      Serial.print(String((int)GPS.seconds, DEC));
      Serial.print(String('.'));
      Serial.print(String((int)GPS.milliseconds, DEC));
      Serial.print(String(','));
      Serial.print(String(GPS.day, DEC));
      Serial.print(String('/'));
      Serial.print(String(GPS.month, DEC));
      Serial.print("/20");
      Serial.print(String(GPS.year, DEC));
      Serial.print(String(','));
      if (GPS.fix) {
        Serial.println("Got a GPS fix.");
        Serial.print(String(GPS.latitudeDegrees, 4));
        Serial.print(String(','));
        Serial.print(String(GPS.longitudeDegrees, 4));
        Serial.print(String(','));
        Serial.print(String(GPS.speed, 2));
        Serial.print(String(','));
        Serial.print(String(GPS.angle, 2));
        Serial.print(String(','));
        Serial.print(String(GPS.altitude, 2));
        Serial.print(String(','));
      }
      Serial.print(String(pitch, 2));
      Serial.print(String(','));
      Serial.print(String(roll, 2));
      Serial.print(String('\n'));
      //close the SD file
      //myFile.close();
    } else {
      //could not open SD file
      Serial.println(F("Error opening the SD file!"));
    }
  } else {
    //transmit not implemented yet
  } 
}

uint32_t timer = millis();
void loop() {
  if (showRawGPSData && !usingInterrupt) {
    char c = GPS.read();
    if (c) Serial.print(c);
  }

  //get current GPS values
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  //set timer, write data every second
  if (timer > millis()) timer = millis();
  if (millis() - timer > 1000) {
    timer = millis();
    updateRollPitch();
    transmitOrWriteData();
  }
}

