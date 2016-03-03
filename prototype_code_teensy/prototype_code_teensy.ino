#include <Adafruit_GPS.h>
#include <ctype.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SPI.h>
#include <SD.h>

/* Assign a unique ID to the 10 DoF sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float roll, pitch, heading;

/* Assign serial interface to the GPS */
HardwareSerial gpsSerial = Serial1;
Adafruit_GPS GPS(&gpsSerial);

/* Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
 Set to 'true' if you want to debug and listen to the raw GPS sentences.*/
#define GPSECHO  false

/*Assign SD card pins*/
#define SDCARD_CS_PIN   10
#define SDCARD_MOSI_PIN  11
#define SDCARD_MISO_PIN  12
#define SDCARD_SCK_PIN  13

/*Make file for sd card*/
File dataFile;

void setup()  
{
  Serial.begin(115200);
  initializeGPS();
  initializeIMU();
  initializeSDCard();
}

void initializeIMU() {
   /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no ADXL345 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

void initializeGPS() {
  gpsSerial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE);
}

void initializeSDCard() {
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setMISO(SDCARD_MISO_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  bool hasBegun = SD.begin(SDCARD_CS_PIN);
  delay(500);
  if (!(hasBegun)) {
    Serial.println("SD card initialization failed");
    return;
  }
  Serial.println("SD card initialized successfully.");
}

void setIMUReadings() {
  sensors_event_t accel_event;  
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    roll = orientation.roll;
    pitch = orientation.pitch;
    heading = orientation.heading;
  }
  delay(100);
}

boolean canTransmitData()
{
  return false;
}
void transmitOrWriteData() {
  if (!canTransmitData()) {
    //open SD file and test if it is open
    dataFile = SD.open("DATA00.txt", FILE_WRITE);
    if (dataFile) {
      //start writing GPS data to SD card
      //just prints to serial for now
      dataFile.print(String((int)GPS.hour, DEC));
      dataFile.print(String(':'));
      dataFile.print(String((int)GPS.minute, DEC));
      dataFile.print(String(':'));
      dataFile.print(String((int)GPS.seconds, DEC));
      dataFile.print(String('.'));
      dataFile.print(String((int)GPS.milliseconds, DEC));
      dataFile.print(String(','));
      dataFile.print(String(GPS.day, DEC));
      dataFile.print(String('/'));
      dataFile.print(String(GPS.month, DEC));
      dataFile.print("/20");
      dataFile.print(String(GPS.year, DEC));
      dataFile.print(String(','));
      if (GPS.fix) {
        dataFile.print(String(GPS.latitudeDegrees, 4));
        dataFile.print(String(','));
        dataFile.print(String(GPS.longitudeDegrees, 4));
        dataFile.print(String(','));
        dataFile.print(String(GPS.speed, 2));
        dataFile.print(String(','));
        dataFile.print(String(GPS.angle, 2));
        dataFile.print(String(','));
        dataFile.print(String(GPS.altitude, 2));
        dataFile.print(String(','));
      }
      dataFile.print(String(pitch, 2));
      dataFile.print(String(','));
      dataFile.print(String(roll, 2));
      dataFile.print(String(','));
      dataFile.print(String(heading, 2));
      dataFile.print(String('\n'));
      //close the SD file
      dataFile.close();
      Serial.println("Wrote data successfully");
    } else {
      //could not open SD file
      Serial.println(F("Error opening the SD file!"));
    }
  } else {
    //transmit not implemented yet
  } 
}

uint32_t timer = millis();
void loop()                     
{
  //read data from GPS
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;  
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // transmit every half second
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer
    setIMUReadings();
    transmitOrWriteData();
  }
}
