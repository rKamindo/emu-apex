/*
@file vented_balloon_controller.ino
@brief A weather balloon ascent and float control system
@author Randy Kamindo

This program controls the ascent and float of a weather balloon using
a BME 280 sensor for altitude measurements and a servo-controlled vent.
It uses an Exponential Moving Average (EMA) to smooth ascent rate calculations
and implements a state machine for flight phase management.

The program is designed to:
- Trigger venting when the balloon reaches a specified target altitude
- Continue venting until the ascent rate falls within the "float" range.
- Terminate the float after a predefined duration

The system transitions between flight states (fast ascent, slow ascent, float)
based on the smoothed ascent rate.
*/

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

#define TEST_MODE \
  true  // set to true when testing with mock data from Python program

// Constants for flight control
#define SEALEVELPRESSURE_HPA (1013.25)
const float TARGET_ALTITUDE = 100;  // meters
const unsigned int FLOAT_DURATION =
    30000;  // float duration in milliseconds (adjust as needed)
const float EMA_ALPHA = 0.2;  // Exponential Moving Average smoothing factor
const int REQUIRED_AGREEMENTS = 10;  // number of agreements in proposed state
                                     // required to change flight staet
const unsigned int SAMPLE_RATE_MS =
    200;  // sample rate of altitude in milliseconds

// Pin definitions and servo connection
const uint8_t BME_SCK = 13;
const uint8_t BME_MISO = 12;
const uint8_t BME_MOSI = 11;
const uint8_t BME_CS = 10;
const uint8_t VENT_SERVO_PIN =
    9;  // pin for vent servo control (replace with actual)

// vent position constants
const int VENT_CLOSED_POSITION =
    0;  // servo angle for closed vent (adjust as needed)
const int VENT_OPEN_POSITION =
    90;  // servo angle for open vent (adjust as needed)

// possible ascent rate states
enum FlightState {
  FAST_ASCENT,
  SLOW_ASCENT,
  FLOAT,
  SLOW_DESCENT,
  FAST_DESCENT,
};

// global variables
Adafruit_BME280 bme;  // BME 280 I2C
Servo ventServo;

unsigned long floatTimerStart = 0;  // timestamp when float state begins
bool floatTimerStarted = false;  // flag to indicate if float timer has started
bool floatComplete = false;
bool ventOpen = false;           // current state of the vent (open/closed)

FlightState currentState = FAST_ASCENT;  // current flight state, initialized to float
FlightState proposedState =
    FAST_ASCENT;  // proposed flight state based on recent measurements

float previousAltitude = 0;  // last recorded altitude measurement
float emaAscentRate = 0;     // exponential moving average of ascent rate

int consecutiveStateAgreements =
    0;  // counter for consecutive state proposal agreements

// function prototypes
float getAltitude();
float calculateAscentRate(float currentAltitude, float previousAltitude,
                          float timeInterval);
float calculateEMA(float new_value, float previous_ema);
void openVent();
void closeVent();
FlightState proposeState(float ascentRate);
void updateFlightState(float ascentRate);

void setup() {
  Serial.begin(9600);
  ventServo.attach(9);

  if (!TEST_MODE) {
    unsigned bme_status = bme.begin();
    // status = bme.begin(0x76, &Wire2);
    if (!bme_status) {
      Serial.println(
          "Could not find a valid BME280 sensor, check wiring, address, sensor "
          "ID!");
      Serial.print("SensorID was: 0x");
      Serial.println(bme.sensorID(), 16);
      Serial.print(
          "        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
          "085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
    }
  }
}

// Returns the altitude in meters derived from pressure measured by the BME 280
// module TO-DO: add GPS as primary altitude source for more acurracy, with
// pressure-derived reading as backup
float getAltitude() {
  float altitude;
  // wait until data is available on the serial port
  if (TEST_MODE) {
    // request data from python program
    Serial.println("REQUEST_ALTITUDE");

    // wait for the response
    while (Serial.available() == 0) {
      delay(1);  // small delay to prevent busy-waiting
    }

    if (Serial.available()) {
      altitude = Serial.parseFloat();
    }

    // consume any remaining characters in the buffer
    while (Serial.available()) {
      Serial.read();
    }
    return altitude;
  } else {
    return bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
}

// calculates and returns the ascent rate in m/s
float calculateAscentRate(float currentAltitude, float previousAltitude,
                          float timeInterval) {
  return (currentAltitude - previousAltitude) / timeInterval;
}

// calculates the Exponential Moving Average
float calculateEMA(float newValue, float previousEMA) {
  return EMA_ALPHA * newValue + (1 - EMA_ALPHA) * previousEMA;
}

// proposes a flight state based on the current ascent rate
FlightState proposeState(float ascentRate) {
  // compare avgAscentRate to differnt states and propose a state
  // return proposed state
  if (ascentRate >= 1.9) return FAST_ASCENT;
  if (ascentRate >= 0.50) return SLOW_ASCENT;
  if (ascentRate > -0.50) return FLOAT;
  if (ascentRate > -1.9) return SLOW_DESCENT;
  if (ascentRate > -10.0) return FAST_DESCENT;
}

// updates the current flight state if there are enough consecutive agreements
// modifies the currentState global variable with new flight state when updating
// flight state
void updateFlightState(float ascentRate) {
  FlightState newProposedState = proposeState(ascentRate);

  if (newProposedState == proposedState) {
    consecutiveStateAgreements++;
    if (consecutiveStateAgreements >= REQUIRED_AGREEMENTS &&
        newProposedState != currentState) {
      currentState = newProposedState;  // modify currentState global variable
                                        // to reflect new state
      consecutiveStateAgreements = 0;   // reset counter after state change
      
      Serial.print(REQUIRED_AGREEMENTS);
      Serial.println(" consecutive state proposals reached, new state");
      Serial.println(currentState);
    }
  } else {
    proposedState = newProposedState;
    consecutiveStateAgreements =
        1;  // reset to 1 as this state differs from the previous state, serving
            // as the first agreement
  }
}

// opens the vent
void openVent() {
  Serial.println("VENT_OPEN");
  ventServo.write(VENT_OPEN_POSITION);
}

// closes the vent
void closeVent() {
  Serial.println("VENT_CLOSE");
  ventServo.write(VENT_CLOSED_POSITION);
}

void loop() {
  static unsigned long lastTimeAltitudeMeasured =
      millis();  // timestamp of last altitude measurement
  static unsigned long lastTimeStateProposed = millis();
  unsigned long currentTime = millis();

  float currentAltitude;
  static float ascentRate;
  static float emaAscentRate;
  // check if it is time to sample based on global SAMPLE_RATE_MS variable
  if (currentTime - lastTimeAltitudeMeasured >= SAMPLE_RATE_MS) {
    currentAltitude = getAltitude();
    float timeInterval =
        (currentTime - lastTimeAltitudeMeasured) / 1000.0;  // time in seconds
    ascentRate =
        calculateAscentRate(currentAltitude, previousAltitude, timeInterval);

    emaAscentRate = calculateEMA(ascentRate, emaAscentRate);
    previousAltitude = currentAltitude;

    lastTimeAltitudeMeasured = currentTime;
  }

  if (currentTime - lastTimeStateProposed >=
      2000) {  // propose a state every two seconds
    // print statements
    Serial.print("Current Altitude: ");
    Serial.print(currentAltitude);
    Serial.print(" m, Ascent Rate: ");
    Serial.print(ascentRate);
    Serial.print(" m/s, EMA Ascent Rate: ");
    Serial.print(emaAscentRate);
    Serial.println(" m/s");
    
    Serial.print("Current flight state: ");
    Serial.println(currentState);

    updateFlightState(emaAscentRate);

    // act on current state
    switch (currentState) {
      case FAST_ASCENT:
      case SLOW_ASCENT:
        if (currentAltitude >= TARGET_ALTITUDE && !ventOpen) {
          openVent();
          ventOpen = true;
          Serial.println("Target altitude reached! Opening vent.");
        }
        break;
      case FLOAT:
        if (currentAltitude >= TARGET_ALTITUDE) {
          if (ventOpen) {   // Close vent if it's open and ascent rate indicates descent
            closeVent();
            ventOpen = false;
            Serial.println("Floating. Closing vent.");
          }

          if (!floatTimerStarted) {
            floatTimerStart = millis();
            floatTimerStarted = true;
            Serial.println("Float timer started.");
          }

          // check if float duration has been exceeded
          if (!floatComplete && floatTimerStarted && (millis() - floatTimerStart >= FLOAT_DURATION)) {
            if (!ventOpen) {  // Open the vent only if it's not already open
              openVent();
              ventOpen = true;
              floatComplete = true;
              Serial.println("Float duration exceeded. Opening vent to release all gas.");
            }
          }
        }
        break;
      case SLOW_DESCENT:
        break;
      case FAST_DESCENT:
        break;
      default:
        Serial.print("Unknown flight state encountered: ");
        Serial.println(currentState);
        break;
    }
    lastTimeStateProposed = currentTime;
  }
}
