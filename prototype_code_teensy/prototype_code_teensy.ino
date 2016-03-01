#include <Adafruit_GPS.h>
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

void setup()  
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
}

boolean canTransmitData()
{
  return false;
}
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
//      Serial.print(String(pitch, 2));
//      Serial.print(String(','));
//      Serial.print(String(roll, 2));
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
void loop()                     // run over and over again
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;  
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    transmitOrWriteData();
  }
}
