#include <Notecard.h>
#include "TSYS01.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Arduino.h"
#include "MS5837.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/*
  HARDWARE DEFINITIONS
*/
#define NUMBER_OF_SENSORS 3
#define usbSerial Serial

#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;
#endif

TSYS01 tempSensor;
MS5837 pressureSensor;

TinyGPSPlus gps;
const uint32_t GPS_BAUD = 9600;

#define address 100
char computerdata[32];
#define MUX_ADDR 0x70
int time_ = 570;
char AtlasCommand[32] = "r";
byte code = 0;
byte in_char = 0;
byte i = 0;
char ec_data[32];

// SPI pins for microSD data saving
static const int sck = 5;
static const int miso = 19;
static const int mosi = 18;
static const int cs = 21;

// UART pins for GPS module
static const int RXPin = 16, TXPin = 17;
HardwareSerial gpsSerial(1);

/*
  Global Variables
*/
const float P0 = 101325.0;
const float R = 8.3144598;
const float g = 9.80665;
const float M = 0.0289644;

// Hardcoded device type for this platform
const char* DEVICE_TYPE = "Drone";

String error = "";
float altitude;
float salinity;
float pressure;
float temperature;
float tempPress;
double lat;
double lon;

String measurementTimestamp = "";

int fileCount = 0;
String fileName = "";
File file;

// Blues Notehub ID
#define productUID "com.gmail.willronan4:drone"

Notecard notecard;

/*
  MUX Methods
*/
void enableMuxPort(byte portNumber) {
  if (portNumber > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << portNumber);
  Wire.endTransmission();
}

void disableMuxPort(byte portNumber) {
  (void)portNumber;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

/*
  Math / Time Helpers
*/
float calculateAltitude(float P, float T_c) {
  float T = T_c + 273.15;
  float factor = (R * T) / (g * M);
  return factor * log(P0 / P);
}

String getDateTimeFromUnix(time_t time, int minutesOffset) {
  time_t adjustedTime = time + (minutesOffset * 60);
  struct tm timeInfo;
  gmtime_r(&adjustedTime, &timeInfo);

  char buffer[25];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
           timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  return String(buffer);
}

/*
  Hardware Methods
*/
float readAtlasSensor() {
  memset(ec_data, 0, sizeof(ec_data));
  i = 0;

  Wire.beginTransmission(address);
  Wire.write((uint8_t *)AtlasCommand, strlen(AtlasCommand));
  Wire.endTransmission();

  delay(time_);

  Wire.requestFrom(address, 32, 1);
  code = Wire.read();

  switch (code) {
    case 1:
      break;
    case 2:
      Serial.println("Failed");
      break;
    case 254:
      Serial.println("Pending");
      break;
    case 255:
      Serial.println("No Data");
      break;
  }

  while (Wire.available()) {
    in_char = Wire.read();
    ec_data[i] = in_char;
    i += 1;

    if (i >= sizeof(ec_data) - 1 || in_char == 0) {
      ec_data[i] = 0;
      i = 0;
      break;
    }
  }

  Serial.println(ec_data);
  return atof(ec_data);
}

void getGPS(double* lat, double* lon) {
  Serial.println("getting GPS");

  if (!gpsSerial.available()) {
    Serial.println("gps failed");
    error = "GPS disconnected";
  }

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    *lat = gps.location.lat();
    *lon = gps.location.lng();
  } else {
    Serial.println(F("INVALID"));
    *lat = 0;
    *lon = 0;
  }
}

String getCardTime() {
  J *req = NoteNewRequest("card.time");
  if (!req) {
    return "";
  }

  if (J *rsp = NoteRequestResponse(req)) {
    int time = JGetNumber(rsp, "time");
    int minutes = JGetNumber(rsp, "minutes");
    String timestamp = getDateTimeFromUnix(time, minutes);

    NoteDeleteResponse(rsp);
    return timestamp;
  }

  return "";
}

void writeToSD(const String& timestamp,
               double latitude,
               double longitude,
               double pressureVal,
               double alt,
               double sal,
               double temp,
               File &f) {
  Serial.println("In writeToSD");

  f.print(timestamp);
  f.print(F(","));
  f.print(latitude, 6);
  f.print(F(","));
  f.print(longitude, 6);
  f.print(F(","));
  f.print(pressureVal, 6);
  f.print(F(","));
  f.print(alt, 6);
  f.print(F(","));
  f.print(sal, 6);
  f.print(F(","));
  f.println(temp, 6);
  f.flush();

  Serial.println("Line flushed");
}

void setup() {
  Serial.begin(9600);
  for (unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime <= 5000); ) { }

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(1);

  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);

  {
    J *req = notecard.newRequest("hub.set");
    if (req != NULL) {
      JAddStringToObject(req, "product", productUID);
      JAddStringToObject(req, "mode", "continuous");
      NoteRequest(req);
    }
  }

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXPin, TXPin);
  Wire.begin();

  for (byte x = 0; x <= 7; x++) {
    disableMuxPort(x);
  }

  enableMuxPort(0);
  delay(50);
  if (pressureSensor.init()) {
    pressureSensor.setModel(MS5837::MS5837_02BA);
    pressureSensor.setFluidDensity(997);
    Serial.println("Port 0: MS5837 initialized!");
  } else {
    Serial.println("Port 0: MS5837 init FAILED!");
  }
  disableMuxPort(0);

  enableMuxPort(3);
  delay(50);
  if (tempSensor.init()) {
    Serial.println("Port 3: TSYS01 initialized!");
  } else {
    Serial.println("Port 3: TSYS01 init FAILED!");
  }
  disableMuxPort(3);

  enableMuxPort(4);
  delay(50);
  Wire.beginTransmission(address);
  byte i2cerror = Wire.endTransmission();
  if (i2cerror == 0) {
    Serial.println("Port 4: Atlas Salinity sensor detected!");
  } else {
    Serial.println("Port 4: Atlas sensor NOT detected!");
  }
  disableMuxPort(4);

  SPI.begin(sck, miso, mosi, cs);
  if (!SD.begin(cs)) {
    Serial.println("Card Mount Failed");
    error = "MicroSD reader disconnected";
    return;
  }

  measurementTimestamp = getCardTime();
  if (measurementTimestamp.length() == 0) {
    measurementTimestamp = "unknown_time";
  }

  String safeFileTimestamp = measurementTimestamp;
  safeFileTimestamp.replace(":", "");
  safeFileTimestamp.replace("Z", "");

  do {
    fileName = "/drone_reading_" + safeFileTimestamp + "_session" + String(fileCount) + ".csv";
    Serial.print("Checking: ");
    Serial.println(fileName);
    fileCount++;
  } while (SD.exists(fileName));

  if (file) {
    file.close();
  }

  file = SD.open(fileName, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
  } else {
    file.println("Timestamp,Latitude,Longitude,Pressure,Altitude,Salinity,Temperature");
    file.flush();
  }

  Serial.println("Created new local data file " + fileName);
  Serial.println("Initialization complete.\n");
}

void loop() {
  error = "";

  Serial.println("Getting card time");
  measurementTimestamp = getCardTime();

  if (measurementTimestamp.length() == 0) {
    error = "Failed to get card time";
  }

  enableMuxPort(4);
  delay(10);
  salinity = readAtlasSensor();
  Serial.print("Salinity = ");
  Serial.println(salinity);
  disableMuxPort(4);

  Serial.println("Polling other sensors");

  enableMuxPort(0);
  delay(10);
  pressureSensor.read();
  pressure = pressureSensor.pressure();
  tempPress = pressureSensor.temperature();
  altitude = calculateAltitude(pressure * 100, tempPress);
  disableMuxPort(0);

  enableMuxPort(3);
  delay(10);
  tempSensor.read();
  temperature = tempSensor.temperature();
  disableMuxPort(3);

  getGPS(&lat, &lon);

  Serial.println("Sending notecard request");
  {
    J *req = notecard.newRequest("note.add");
    if (req != NULL) {
      JAddBoolToObject(req, "sync", true);

      J *body = JCreateObject();
      if (body) {
        JAddNumberToObject(body, "temperature", temperature);
        JAddNumberToObject(body, "salinity", salinity);
        JAddNumberToObject(body, "pressure", pressure);
        JAddNumberToObject(body, "altitude", altitude);
        JAddNumberToObject(body, "longitude", lon);
        JAddNumberToObject(body, "latitude", lat);
        JAddNumberToObject(body, "tempPress", tempPress);

        // Canonical sample time from the device
        JAddStringToObject(body, "Timestamp", measurementTimestamp.c_str());

        // Device metadata
        JAddStringToObject(body, "deviceType", DEVICE_TYPE);

        JAddStringToObject(body, "error", error.c_str());

        JAddItemToObject(req, "body", body);
      }

      NoteRequest(req);
    }

    Serial.print("Timestamp = ");
    Serial.println(measurementTimestamp);
    Serial.print("deviceType = ");
    Serial.println(DEVICE_TYPE);
    Serial.print("lat = ");
    Serial.println(lat);
    Serial.print("lon = ");
    Serial.println(lon);
    Serial.print("pressure = ");
    Serial.println(pressure);
    Serial.print("salinity = ");
    Serial.println(salinity);
    Serial.print("temperature = ");
    Serial.println(temperature);
    Serial.print("error = ");
    Serial.println(error);

    if (file) {
      writeToSD(measurementTimestamp, lat, lon, pressure, altitude, salinity, temperature, file);
    }

    delay(100);
  }
}
