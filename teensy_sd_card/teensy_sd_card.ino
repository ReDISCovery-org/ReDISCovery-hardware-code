#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define SDCARD_CS_PIN   10
#define SDCARD_MOSI_PIN  11
#define SDCARD_MISO_PIN  12
#define SDCARD_SCK_PIN  13

File myFile;

void setup() {
  Serial.begin(115200);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setMISO(SDCARD_MISO_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);

  //initialize the SD card
  Serial.print("Intializing SD card...");
  bool hasBegun = SD.begin(SDCARD_CS_PIN);
  delay(500);
  if (!(hasBegun)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done");

  //write to the SD card
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to text.txt...");
    myFile.println("testing 1, 2, 3");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("Error opening test.txt!");
  }

  //read from the SD card
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println("Error opening test.txt!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
