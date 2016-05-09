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
#include <ArduinoJson.h>

/* Assign a unique ID to the 10 DoF sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float roll, pitch, heading;

//window of roll, pitch, and heading values
#define WINDOW_SIZE 30
float rollValues[WINDOW_SIZE];
float pitchValues[WINDOW_SIZE];
float headingValues[WINDOW_SIZE];
int valuesStart = 0, valuesEnd = 0;

//threshold for roll, pitch, and heading variance
float ROLL_THRESHOLD = 0;
float PITCH_THRESHOLD = 0;
float HEADING_THRESHOLD = 0;

//variables used for computing the moving average and variance
float rollMovingAverage, rollVariance;
float pitchMovingAverage, pitchVariance;
float headingMovingAverage, headingVariance;

/* Assign serial interface to the GPS */
HardwareSerial gpsSerial = Serial1;
Adafruit_GPS GPS(&gpsSerial);

/*Serial interface for the ESP8266*/
HardwareSerial espSerial = Serial2;

/* Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
 Set to 'true' if you want to debug and listen to the raw GPS sentences.*/
#define GPSECHO  false

/*Debug mode*/
#define DEBUG true

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
  //initializeESP();
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

  //fill values 
  for (int i = 0; i < WINDOW_SIZE; i++) {
    setIMUReadings();
    rollValues[i] = roll;
    pitchValues[i] = pitch;
    headingValues[i] = heading;
    valuesEnd++;
  }

  //set initial average and variance
  rollMovingAverage = calcInitialAverage(rollValues);
  pitchMovingAverage = calcInitialAverage(pitchValues);
  headingMovingAverage = calcInitialAverage(headingValues);
  rollVariance = calcInitialVariance(rollValues, rollMovingAverage);
  pitchVariance = calcInitialVariance(pitchValues, pitchMovingAverage);
  headingVariance = calcInitialVariance(headingValues, headingMovingAverage);
  Serial.println(rollVariance);
  Serial.println(pitchVariance);
  Serial.println(headingVariance);
  dataFile = SD.open("DATA01.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Initial roll variance: " );
    dataFile.println(rollVariance);
    dataFile.print("Initial pitch variance: ");
    dataFile.println(pitchVariance);
    dataFile.print("Initial heading variance: ");
    dataFile.println(headingVariance);
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

void initializeESP() {
//  espSerial.begin(9600);
//  sendCommand("AT+RST\r\n",2000,DEBUG); // reset module
//  sendCommand("AT+CWMODE=1\r\n",1000,DEBUG); // configure as access point
//  sendCommand("AT+CWJAP=\"mySSID\",\"myPassword\"\r\n",3000,DEBUG);
//  delay(10000);
//  sendCommand("AT+CIFSR\r\n",1000,DEBUG); // get ip address
//  sendCommand("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
//  sendCommand("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80
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
}

void setIMUValues() {
  valuesStart = (valuesStart + 1) % WINDOW_SIZE;
  valuesEnd = (valuesEnd + 1) % WINDOW_SIZE;
  rollValues[valuesEnd] = roll;
  pitchValues[valuesEnd] = pitch;
  headingValues[valuesEnd] = heading;
}

float calcInitialAverage(float values[]) {
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum = sum + values[i];
  }
  return sum / (float)WINDOW_SIZE;
}

float calcInitialVariance(float values[], float mean) {
  float squareSum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    squareSum = squareSum + (values[i] * values[i]);
  }
  return squareSum / WINDOW_SIZE - mean * mean;
}

void calcMovingAverage() {
  rollMovingAverage = rollMovingAverage - (rollValues[valuesEnd] / WINDOW_SIZE) + (roll / WINDOW_SIZE);
  pitchMovingAverage = pitchMovingAverage - (pitchValues[valuesEnd] / WINDOW_SIZE) + (pitch / WINDOW_SIZE);
  headingMovingAverage = headingMovingAverage - (headingValues[valuesEnd] / WINDOW_SIZE) + (heading / WINDOW_SIZE);
}

void calcMovingVariance() {
  //subtract the old datapoint and add the old average
  rollVariance = rollVariance - (rollValues[valuesEnd]*rollValues[valuesEnd]) / WINDOW_SIZE + rollMovingAverage*rollMovingAverage;
  pitchVariance = pitchVariance - (pitchValues[valuesEnd]*pitchValues[valuesEnd]) / WINDOW_SIZE + pitchMovingAverage*pitchMovingAverage;
  headingVariance = headingVariance - (headingValues[valuesEnd]*headingValues[valuesEnd]) / WINDOW_SIZE + headingMovingAverage*headingMovingAverage;

  //calculate the new averages
  calcMovingAverage();

  //add the new datapoint and substract the new average
  rollVariance = rollVariance + (roll*roll) / WINDOW_SIZE - rollMovingAverage*rollMovingAverage;
  pitchVariance = pitchVariance + (pitch*pitch) / WINDOW_SIZE - pitchMovingAverage*pitchMovingAverage;
  headingVariance = headingVariance + (heading*heading) / WINDOW_SIZE - headingMovingAverage*headingMovingAverage;
}

void performIMUOperations() {
  setIMUReadings();
  calcMovingVariance();
  setIMUValues();
}

bool isDiscStopped() {
  //float rollVariance = calcVariance(rollValues, WINDOW_SIZE);
  //float pitchVariance = calcVariance(pitchValues, WINDOW_SIZE);
  //float headingVariance = calcVariance(headingValues, WINDOW_SIZE);
  if (rollVariance < ROLL_THRESHOLD && pitchVariance < PITCH_THRESHOLD && headingVariance < HEADING_THRESHOLD)
    return true;
  else 
    return false;
}

boolean canTransmitData()
{
  return false;
}

void transmitOrWriteGPSData() {
  //build the JSON packet
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = "gps";
  String gpsTime = String((int)GPS.hour, DEC) + ":" + String((int)GPS.minute, DEC) + ":" 
    + String((int)GPS.seconds, DEC) + "." + String((int)GPS.milliseconds, DEC);
  root["time"] = gpsTime;
  String date = String((int)GPS.day, DEC) + "/" + String((int)GPS.month, DEC) + "/20" 
    + String((int)GPS.year, DEC);
  root["date"] = date;
  if (GPS.fix) {
    JsonArray& coords = root.createNestedArray("coords");
    coords.add(double_with_n_digits(GPS.latitudeDegrees, 6));
    coords.add(double_with_n_digits(GPS.longitudeDegrees, 6));
    root["speed"] = GPS.speed;
    root["altitude"] = GPS.altitude;
  } else {
    JsonArray& coords = root.createNestedArray("coords");
    coords.add(0);
    coords.add(0);
    root["speed"] = 0;
    root["altitude"] = 0;
  }

  //transmit if in range or write to SD card
  if (canTransmitData()) {
    
  } else {
    dataFile = SD.open("DATA00.txt", FILE_WRITE);
    if (dataFile) {
      root.printTo(dataFile);
      dataFile.print("\n");
      dataFile.close();
      Serial.println("Wrote GPS data successfully");
    } else {
      root.prettyPrintTo(Serial);
    }
  }
}

void transmitOrWriteIMUData() {
  //build the JSON packet
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = "imu";
  root["time"] = millis();
  root["pitch"] = pitch;
  root["roll"] = roll;
  root["heading"] = heading;

  //transmit if in range or write to SD card
  if (canTransmitData()) {
    root.printTo(espSerial);  
  } else {
    dataFile = SD.open("DATA00.txt", FILE_WRITE);
    if (dataFile) {
      root.printTo(dataFile);
      dataFile.print("\n");
      dataFile.close();
      Serial.println("Wrote IMU data successfully.");
    } else {
      root.prettyPrintTo(Serial);
    }
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

  // transmit GPS data every second
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer
    //transmitOrWriteGPSData();
  }
  performIMUOperations();
  Serial.print("Roll: ");
  Serial.print(roll, 4);
  Serial.print(" Pitch: ");
  Serial.print(pitch, 4);
  Serial.print(" Heading: ");
  Serial.println(heading, 4);
  Serial.print("Roll variance: ");
  Serial.print(rollVariance, 4);
  Serial.print(" Pitch Variance: ");
  Serial.print(pitchVariance, 4);
  Serial.print(" Heading Variance: ");
  Serial.println(headingVariance, 4);
  Serial.print("{");
  for (int i = 0; i < WINDOW_SIZE; i++) {
    Serial.print(pitchValues[i]);
    Serial.print(", ");
  }
  Serial.println("}");
  dataFile = SD.open("DATA01.txt", FILE_WRITE);
  if (dataFile) {
//    dataFile.print("Roll: ");
//    dataFile.print(roll, 4);
//    dataFile.print(" Pitch: ");
//    dataFile.print(pitch, 4);
//    dataFile.print(" Heading: ");
//    dataFile.println(heading, 4);
//    dataFile.print("Roll variance: ");
//    dataFile.print(rollVariance, 4);
//    dataFile.print(" Pitch Variance: ");
//    dataFile.print(pitchVariance, 4);
//    dataFile.print(" Heading Variance: ");
//    dataFile.println(headingVariance, 4);
    dataFile.print("[");
    for (int i = 0; i < WINDOW_SIZE; i++) {
      dataFile.print(pitchValues[i]);
      dataFile.print(", ");
    }
    dataFile.print("]");
    dataFile.close();
  }
  Serial.println("}");
  delay(1000);
  //transmitOrWriteIMUData();
}
