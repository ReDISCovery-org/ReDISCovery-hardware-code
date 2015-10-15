#include <SD.h>

#include <SoftwareSerial.h>

#include <Adafruit_GPS.h>

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Set up SoftwareSerial
//Attach TX to Digital pin 3
//Attach RX to Digital pin 2

SoftwareSerial mySerial(3, 2);

//Attach GPS to SoftwareSerial
Adafruit_GPS GPS(&mySerial);

boolean showRawGPSData = false;

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  showRawGPSData = true;
}

//tests the wireless connection to see if data can be transmitted
//set to false for now
boolean canTransmitData() {
  return false;
}

//wirelessly transmits data to an Android phone to be saved
void transmitData(String data) {
  
}

//writes data to an SD card
//just prints to the Serial monitor for now
void writeDataToSD(String data) {
  Serial.print(data);
}

//transmit/write GPS data
void transmitOrWriteGPSData(Adafruit_GPS gps) {
  if (!canTransmitData()) {
    //start writing GPS data to SD cart
    //just prints to serial for now
    writeDataToSD(String((int)gps.hour, DEC));
    writeDataToSD(String(':'));
    writeDataToSD(String((int)gps.minute, DEC));
    writeDataToSD(String(':'));
    writeDataToSD(String((int)gps.seconds, DEC));
    writeDataToSD(String('.'));
    writeDataToSD(String((int)gps.milliseconds, DEC));
    writeDataToSD(String(','));
    writeDataToSD(String(gps.day, DEC));
    writeDataToSD(String('/'));
    writeDataToSD(String(gps.month, DEC));
    writeDataToSD("/20");
    writeDataToSD(String(gps.year, DEC));
    if (GPS.fix) {
      writeDataToSD(String((long)gps.latitudeDegrees));
      writeDataToSD(String(','));
      writeDataToSD(String((long)gps.longitudeDegrees));
      writeDataToSD(String(','));
      writeDataToSD(String((long)gps.speed));
      writeDataToSD(String(','));
      writeDataToSD(String((long)gps.angle));
      writeDataToSD(String(','));
      writeDataToSD(String((long)gps.altitude));
    }
    writeDataToSD(String('\n'));
  } else {
    //transmit not implemented yet
  }
}

uint32_t timer = millis();
void loop() {
  if (showRawGPSData) {
    char c = GPS.read();
    if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (timer > millis()) timer = millis();
  
  if (millis() - timer > 2000) {
    timer = millis();
    transmitOrWriteGPSData(GPS);
  }
}



