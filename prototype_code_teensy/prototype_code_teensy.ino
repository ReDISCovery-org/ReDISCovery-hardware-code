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
#include <SoftwareSerial.h>

/* Assign a unique ID to the 10 DoF sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float roll, pitch, heading;
float accelX, accelY, accelZ;
float ACCEL_THRESHOLD = 1.0;

double mostRecentLongitudeReading;
double mostRecentLatitudeReading;
double mostRecentSpeedReading;

/* Assign serial interface to the GPS */
HardwareSerial gpsSerial = Serial1;
Adafruit_GPS GPS(&gpsSerial);

/*Serial interface for the ESP8266*/
SoftwareSerial esp8266(7, 8);

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

  //set most recent
  mostRecentLatitudeReading = 0;
  mostRecentLongitudeReading = 0;
  mostRecentSpeedReading = 0;
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

void initializeESP() {
  esp8266.begin(115200);
  sendCommand("AT+RST\r\n",2000,DEBUG); // reset module
  sendCommand("AT+CWMODE=1\r\n",1000,DEBUG); // configure as access point
  sendCommand("AT+CWJAP=\"ninevolz\",\"raoi911*\"\r\n",3000,DEBUG);
  delay(10000);
  sendCommand("AT+CIFSR\r\n",1000,DEBUG); // get ip address
  sendCommand("AT+CIPMUX=1\r\n",1000,DEBUG); // configure for multiple connections
  sendCommand("AT+CIPSERVER=1,80\r\n",1000,DEBUG); // turn on server on port 80
}

/*
* Name: sendData
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    
    int dataSize = command.length();
    char data[dataSize];
    command.toCharArray(data,dataSize);
           
    esp8266.write(data,dataSize); // send the read character to the esp8266
    if(debug)
    {
      Serial.println("\r\n====== HTTP Response From Arduino ======");
      Serial.write(data,dataSize);
      Serial.println("\r\n========================================");
    }
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    }
    
    return response;
}

/*
* Name: sendHTTPResponse
* Description: Function that sends HTTP 200, HTML UTF-8 response
*/
void sendHTTPResponse(int connectionId, String content)
{
     
     // build HTTP response
     String httpResponse;
     String httpHeader;
     // HTTP Header
     httpHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n"; 
     httpHeader += "Content-Length: ";
     httpHeader += content.length();
     httpHeader += "\r\n";
     httpHeader +="Connection: close\r\n\r\n";
     httpResponse = httpHeader + content + " "; // There is a bug in this code: the last character of "content" is not sent, I cheated by adding this extra space
     sendCIPData(connectionId,httpResponse);
}

/*
* Name: sendCIPDATA
* Description: sends a CIPSEND=<connectionId>,<data> command
*
*/
void sendCIPData(int connectionId, String data)
{
   String cipSend = "AT+CIPSEND=";
   cipSend += connectionId;
   cipSend += ",";
   cipSend +=data.length();
   cipSend +="\r\n";
   sendCommand(cipSend,1000,DEBUG);
   sendData(data,1000,DEBUG);
}

/*
* Name: sendCommand
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
String sendCommand(String command, const int timeout, boolean debug)
{
    String response = "";
           
    esp8266.print(command); // send the read character to the esp8266
    
    long int time = millis();
    
    while( (time+timeout) > millis())
    {
      while(esp8266.available())
      {
        
        // The esp has data so display its output to the serial window 
        char c = esp8266.read(); // read the next character.
        response+=c;
      }  
    }
    
    if(debug)
    {
      Serial.print(response);
    }
    
    return response;
}

void checkMessages() {
  
  if(esp8266.available() && isDiscStopped()) // check if the esp is sending a message 
  {
    if(esp8266.find("+IPD,"))
    {
     delay(1000); // wait for the serial buffer to fill up (read all the serial data)
     // get the connection id so that we can then disconnect
     int connectionId = esp8266.read()-48; // subtract 48 because the read() function returns 
                                           // the ASCII decimal value and 0 (the first decimal number) starts at 48
          
     // build string that is send back to device that is requesting pin toggle
     String content = "";
     dataFile = SD.open("DATA01.txt");
     if (dataFile) {
        while (dataFile.available()) {
          content += dataFile.read();
        }
        dataFile.close();
        Serial.println("Read following data successfully from cache: ");
        Serial.println(content);
     } else {
        Serial.println("Failed to read data from cache");
     }
     
     sendHTTPResponse(connectionId, content);
     
     // make close command
     String closeCommand = "AT+CIPCLOSE="; 
     closeCommand+=connectionId; // append connection id
     closeCommand+="\r\n";
     
     sendCommand(closeCommand,1000,DEBUG); // close connection
    }
  }
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

  accelX = accel_event.acceleration.x;
  accelY = accel_event.acceleration.y;
  accelZ = accel_event.acceleration.z;
}

bool isDiscStopped() {
  if (accelX < ACCEL_THRESHOLD && accelY < ACCEL_THRESHOLD)
    return true;
  else 
    return false;
}

void writeGPSData() {
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

    //set most recent
    mostRecentLatitudeReading = double_with_n_digits(GPS.latitudeDegrees, 6);
    mostRecentLongitudeReading = double_with_n_digits(GPS.longitudeDegrees, 6);
    mostRecentSpeedReading = GPS.speed * 0.51444;
  } else {
    JsonArray& coords = root.createNestedArray("coords");
    coords.add(0);
    coords.add(0);
    root["speed"] = 0;
    root["altitude"] = 0;
  }
  dataFile = SD.open("DATA00.txt", FILE_WRITE);
  if (dataFile) {
    root.printTo(dataFile);
    dataFile.print("\n");
    dataFile.close();
    Serial.println("Wrote GPS data successfully to store.");
  } else {
    Serial.println("Failed to write GPS data to store.");
  }
}

void writeIMUData() {
  //build the JSON packet
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = "imu";
  root["time"] = millis();
  root["pitch"] = pitch;
  root["roll"] = roll;
  root["heading"] = heading;
  dataFile = SD.open("DATA00.txt", FILE_WRITE);
  if (dataFile) {
    root.printTo(dataFile);
    dataFile.print("\n");
    dataFile.close();
    Serial.println("Wrote IMU data successfully to store.");
  } else {
    Serial.println("Failed to write IMU data to store.");
  }
}

void writeDataToCache() {
  if (isDiscStopped()) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonArray& array = root.createNestedArray("sensor_info");
    JsonObject& sensorInfo = array.createNestedObject();
    sensorInfo["latitude"] = mostRecentLatitudeReading;
    sensorInfo["longitude"] = mostRecentLongitudeReading;
    sensorInfo["linear_velocity"] = mostRecentSpeedReading;
    root["success"] = 1;
    dataFile = SD.open("DATA01.txt", O_WRITE | O_CREAT | O_TRUNC);
    if (dataFile) {
      root.printTo(dataFile);
      dataFile.print("\n");
      dataFile.close();
      Serial.println("Wrote data successfully to cache.");
    } else {
      Serial.println("Failed to write write data to cache.");
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
    writeGPSData();
  }
  setIMUReadings();
  writeIMUData();
  writeDataToCache();
}
