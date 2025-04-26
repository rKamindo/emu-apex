#include <SPI.h>
#include <SoftwareSerial.h>  // this is for Openlog SD Card Reader

// OpenLogger Information
// Connect Arduino pin 4 (RX) to OpenLog TX
// Connect Arduino pin 3 (TX) to OpenLog RX
SoftwareSerial OpenLog(4, 3);  // (Arduino RX, TX)

#define SEALEVELPRESSURE_HPA (1013.25)

// constants for FS1012
const int analogInPin0 = A0;
const int sampleAverage = 20;
const int serialRateOutput = 1000;
const int analogSampleDelay = 2;

// variables for FS1012
int sensorValue = 0;        // value read from the flow sensor
float sensorAverage = 0.0;  // average sensor value

unsigned long startTime = millis();

void setup() {
  Serial.begin(9600);
  OpenLog.begin(9600);
  Serial.println("Initializing SD card...");

  analogReference(INTERNAL);

  goToCommandMode();
  createFile("flow-sensor.csv");
  writeHeader();
}

void loop() {
  // start of FS1012 code
  sensorAverage = 0.0;

  for (int i = 0; i < sampleAverage; i++) {
    // read the analog in value
    sensorValue = analogRead(analogInPin0);
    // wait for analog-to-digital converter
    delay(analogSampleDelay);
    // accumulate sensor data
    sensorAverage = sensorAverage + sensorValue;
  }
  // calculate sensor average
  sensorAverage = sensorAverage / sampleAverage;

  unsigned long timeSinceStart = millis() - startTime;

  Serial.println(sensorAverage);
  OpenLog.print(timeSinceStart);
  OpenLog.print(",");
  OpenLog.print(sensorAverage);
  OpenLog.println();

  // delay the remaining time to maintain the desired output rate
  delay(serialRateOutput - (sampleAverage * analogSampleDelay));
}

/**
 * Enters OpenLog command mode by sending three Ctrl+Z characters (ASCII 26).
 * Waits until OpenLog responds with '>' to confirm command mode is active.
 * This is required before sending any file management commands to OpenLog.
 */
void goToCommandMode() {
  // Send three control z to enter OpenLog command mode
  // Works with Arduino v1.0
  OpenLog.write(26);
  OpenLog.write(26);
  OpenLog.write(26);

  // Wait for OpenLog to respond with '>' to indicate we are in command mode
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '>') break;
  }
}

/**
 * Creates a new file on the OpenLog SD card and opens it in append mode.
 * The function sends the "new <filename>" and "append <filename>" commands
 * to OpenLog, waiting for the appropriate responses ('>' and '<') after each.
 * After this function, the file is ready to receive data.
 *
 * @param filename The name of the file to create and open (e.g.,
 * "flow-sensor.csv")
 */
void createFile(char *filename) {
  Serial.print("Creating ");
  Serial.println(filename);

  // send command to create a new file
  OpenLog.print("new ");
  OpenLog.println(filename);

  // wait for OpenLog to return to waiting for a command
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '>') break;
  }

  OpenLog.print("append ");
  OpenLog.println(filename);

  // wait for OpenLog to indicate file is open and ready for writing
  while (1) {
    if (OpenLog.available())
      if (OpenLog.read() == '<') break;
  }

  Serial.print("Created file ");
  Serial.println(filename);
}

void sendCommand(char *command) { OpenLog.println(command); }

void writeHeader() { OpenLog.println("Time, FlowSensor"); }