#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Set up SoftwareSerial
//Attach TX to Digital pin 3
//Attach RX to Digital pin 2

SoftwareSerial mySerial(3, 2);

//Attach GPS to SoftwareSerial
Adafruit_GPS GPS(&mySerial);

boolean showRawGPSData;
boolean usingInterrupt;

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
void transmitOrWriteGPSData() {
  if (!canTransmitData()) {
    //start writing GPS data to SD cart
    //just prints to serial for now
    writeDataToSD(String((int)GPS.hour, DEC));
    writeDataToSD(String(':'));
    writeDataToSD(String((int)GPS.minute, DEC));
    writeDataToSD(String(':'));
    writeDataToSD(String((int)GPS.seconds, DEC));
    writeDataToSD(String('.'));
    writeDataToSD(String((int)GPS.milliseconds, DEC));
    writeDataToSD(String(','));
    writeDataToSD(String(GPS.day, DEC));
    writeDataToSD(String('/'));
    writeDataToSD(String(GPS.month, DEC));
    writeDataToSD("/20");
    writeDataToSD(String(GPS.year, DEC));
    if (GPS.fix) {
      writeDataToSD(String((long)GPS.latitudeDegrees));
      writeDataToSD(String(','));
      writeDataToSD(String((long)GPS.longitudeDegrees));
      writeDataToSD(String(','));
      writeDataToSD(String((long)GPS.speed));
      writeDataToSD(String(','));
      writeDataToSD(String((long)GPS.angle));
      writeDataToSD(String(','));
      writeDataToSD(String((long)GPS.altitude));
    }
    writeDataToSD(String('\n'));
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
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (timer > millis()) timer = millis();
  
  if (millis() - timer > 1000) {
    timer = millis();
    transmitOrWriteGPSData();
  }
}



