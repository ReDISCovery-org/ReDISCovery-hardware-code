#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

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


SoftwareSerial mySerial(3, 2);

//Attach GPS to SoftwareSerial
Adafruit_GPS GPS(&mySerial);

boolean showRawGPSData;
boolean usingInterrupt;
boolean isSDInitialized;
File myFile;

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

//initializes the SD card
//run in the setup function
void initializeSD() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("SD Initialization Failed!");
    isSDInitialized = false;
    return;
  }
  Serial.println("SD Initialized Successfully.");
  isSDInitialized = true;
}

//open the file on the SD card
boolean openSDFile() {
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    return true;
  }
  return false;
}

//closes the file on the SD card
void closeSDFile() {
  myFile.close();
}

//writes data to an SD card
//just prints to the Serial monitor for now
void writeDataToSD(String data) {
  myFile.println(data);
  Serial.println("Wrote: ");
  Serial.println(data);
  Serial.println("to SD card successfully.");
}

//reads all data from SD card
void readDataFromSD() {
  while (myFile.available()) {
    Serial.write(myFile.read());
  }
}

//transmit/write GPS data
void transmitOrWriteGPSData() {
  if (!canTransmitData()) {
    //open SD file and test if it is open
    if (openSDFile()) {
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
      //close the SD file
      closeSDFile();
    } else {
      //could not open SD file
      Serial.println("Error opening the SD file!");
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

