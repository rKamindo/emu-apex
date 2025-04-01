#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>       // This is for Openlog SD Card Reader
// OpenLogger Information
SoftwareSerial OpenLog(4, 3);      // (RX,TX)

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

//constants for FS1012
const int analogInPin0 = A0;
const int sampleAverage = 20;
const int serialRateOutput = 100;
const int analogSampleDelay = 2;

// Variables for FS1012
int sensorValue = 0;                  // value read from the flow sensor
float sensorAverage = 0.0;            // average sensor value

Adafruit_BME280 bme1; // I2C
Adafruit_BME280 bme2;
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void setup() {
    Serial.begin(9600);
    OpenLog.begin(9600);
    Serial.print("Initializing SD card...");
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status1;
    unsigned status2;
  
    // default settings
    status1 = bme1.begin();  
    //status = bme.begin(0x76, &Wire2);
    if (!status1) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme1.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    status2 = bme2.begin(0x76);  
     //status = bme.begin(0x76, &Wire2);
    if (!status2) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme2.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 2000;

    Serial.println();
    analogReference(INTERNAL);

    Serial.println("Creating data.csv");
    goToCommandMode();
    createFile("data.csv");
    Serial.println("Created file");
    writeHeader();
}

void loop() { 
    printValues();
    delay(delayTime);
}

void printValues() {
  float temperature1 = bme1.readTemperature();
  float temperature2 = bme2.readTemperature();
  float pressure1 = bme1.readPressure() / 100.0F;
  float pressure2 = bme2.readPressure() / 100.0F;
  float altitude1 = bme1.readAltitude(SEALEVELPRESSURE_HPA);
  float altitude2 = bme2.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity1 = bme1.readHumidity();
  float humidity2 = bme2.readHumidity();

  //start of FS1012 code
  sensorAverage = 0.0;

  for(int i = 0; i < sampleAverage; i++)
  {
    // read the analog in value
    sensorValue = analogRead(analogInPin0);
    // wait for analog-to-digital converter
    delay(analogSampleDelay);
    // accumulate sensor data
    sensorAverage = sensorAverage + sensorValue;
  }
  // calculate sensor average
  sensorAverage = sensorAverage / sampleAverage;

  OpenLog.print(temperature1);
  OpenLog.print(", ");
  OpenLog.print(pressure1);
  OpenLog.print(", ");
  OpenLog.print(altitude1);
  OpenLog.print(", ");
  OpenLog.print(humidity1);
  OpenLog.print(", ");
  OpenLog.print(temperature2);
  OpenLog.print(", ");
  OpenLog.print(pressure2);
  OpenLog.print(", ");
  OpenLog.print(altitude2);
  OpenLog.print(", ");
  OpenLog.print(humidity2);
  OpenLog.print(", ");
  OpenLog.print(sensorAverage);
  OpenLog.println();
}


//This function pushes OpenLog into command mode
void goToCommandMode() {
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  OpenLog.write(26);
  OpenLog.write(26);
  OpenLog.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '>') break;
  }
}

//This function creates a given file and then opens it in append mode (ready to record characters to the file)
//Then returns to listening mode
void createFile(char *filename) {

  OpenLog.print("new ");
  OpenLog.println(filename);

  //Wait for OpenLog to return to waiting for a command
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '>') break;
  }

  OpenLog.print("append ");
  OpenLog.println(filename);

  //Wait for OpenLog to indicate file is open and ready for writing
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '<') break;
  }

  //OpenLog is now waiting for characters and will record them to the new file  
}

void sendCommand(char *command) {
  OpenLog.println(command);
}

void writeHeader() {
  OpenLog.println("Temperature1,Pressure1,Altitude1,Humidity1,Temperature2,Pressure2,Altitude2,Humidity2,FlowSensor");
}
