#include <Arduino.h>
#include <Wire.h>
#include <Notecard.h>
#include <ping1d.h>
#include "TSYS01.h"
#include "MS5837.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// =====================================================
// Drone Water-State Data Collection + Transmission
//
// Behavior:
//   - Always polls Ping + Atlas EC for Air/Water detection.
//   - Always transmits a Notecard message, whether in Air or Water.
//   - Uses the deviceType field to tell the dashboard the state:
//       * "water" when the detector says the system is in water
//       * "air" when the detector says the system is in air
//   - Always polls pressure and temperature in both Air and Water.
//   - Always writes latest TSYS01 temperature compensation to Atlas: "T,<temperature>".
//   - Always transmits full main sensor data through Notecard.
//   - When Water is detected:
//       * saves main sensor data locally
//       * saves Ping echo profile locally
//   - When Air is detected:
//       * transmits main sensor data only
//       * does NOT save Ping echo profiles locally
// =====================================================

#define usbSerial Serial

#define MUX_ADDR 0x70
#define ATLAS_ADDR 100
#define productUID "com.gmail.willronan4:drone"

#define GPS_MUX_PORT      3
#define PRESSURE_MUX_PORT 2
#define TEMP_MUX_PORT     5
#define ATLAS_MUX_PORT    7

#define PING_RX 16
#define PING_TX 17

#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;
#endif

TSYS01 tempSensor;
MS5837 pressureSensor;
Notecard notecard;
SFE_UBLOX_GNSS myGNSS;


HardwareSerial PingSerial(2);
Ping1D ping(PingSerial);

// microSD SPI pins from your drone code
static const int sck  = 5;
static const int miso = 19;
static const int mosi = 18;
static const int cs   = 21;

// Device metadata
const char* DEVICE_TYPE_WATER = "water";
const char* DEVICE_TYPE_AIR = "air";

// Sensor values
float pressure = -9999.0;
float altitude = -9999.0;
float salinity = -9999.0;
float conductivity = -9999.0;
float temperature = -9999.0;
float tempPress = -9999.0;

double lat = 0.0;
double lon = 0.0;

uint32_t pingDistanceMm = 0;
uint16_t pingConfidence = 0;

// Atlas
char AtlasCommand[32] = "r";
char ec_data[32];
byte atlasCode = 0;
const int atlasReadDelayMs = 600;
const int atlasCommandDelayMs = 300;

// Files
File dataFile;
File profileFile;
String dataFileName;
String profileFileName;
uint32_t sampleNumber = 0;
uint32_t waterLandingNumber = 0;

// Time/error
String error = "";
String measurementTimestamp = "";

// Altitude helper constants
const float P0 = 101325.0;
const float R = 8.3144598;
const float g = 9.80665;
const float M = 0.0289644;

// =====================================================
// Water detector settings
// Baseline conductivity assumed to be 0.
// Enter: conductivity + Ping confidence.
// Exit: Ping confidence only.
// =====================================================

const float COND_ENTER_THRESHOLD = 20.0;

const uint16_t PING_ENTER_CONF = 70;
const uint16_t PING_EXIT_CONF  = 40;
const uint16_t MIN_TRANSMIT_CONF = 60;

const int ENTER_REQUIRED_COUNT = 5;
const int EXIT_REQUIRED_COUNT  = 5;

int enterCount = 0;
int exitCount = 0;
bool isWater = false;

// =====================================================
// Mux
// =====================================================

void enableMuxPort(byte portNumber) {
  if (portNumber > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << portNumber);
  Wire.endTransmission();
}

void disableMux() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// =====================================================
// Helpers
// =====================================================

float calculateAltitude(float P_pa, float T_c) {
  float T = T_c + 273.15;
  float factor = (R * T) / (g * M);
  return factor * log(P0 / P_pa);
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

String getCardTime() {
  disableMux(); // Ensure no other sensors are "visible" to the bus
  delay(20);    // Let the I2C lines settle to "High"
  J *req = NoteNewRequest("card.time");
  if (!req) return "";

  if (J *rsp = NoteRequestResponse(req)) {
    int time = JGetNumber(rsp, "time");
    int minutes = JGetNumber(rsp, "minutes");
    String timestamp = getDateTimeFromUnix(time, minutes);
    NoteDeleteResponse(rsp);
    return timestamp;
  }

  return "";
}

String makeUniqueFileName(const String& base, const String& ext) {
  int count = 0;
  String name;

  do {
    name = "/" + base + "_" + String(count) + ext;
    count++;
  } while (SD.exists(name));

  return name;
}

bool readGPSLocation() {
  enableMuxPort(GPS_MUX_PORT);
  delay(20);

  long latitudeRaw = myGNSS.getLatitude();
  long longitudeRaw = myGNSS.getLongitude();

  byte siv = myGNSS.getSIV();

  disableMux();
  delay(20);

  lat = latitudeRaw / 10000000.0;
  lon = longitudeRaw / 10000000.0;

  if (latitudeRaw == 0 && longitudeRaw == 0) {
    error += "GPS no fix or zero coordinates; ";
    return false;
  }

  Serial.print("GPS lat=");
  Serial.print(lat, 7);
  Serial.print(" lon=");
  Serial.print(lon, 7);
  Serial.print(" SIV=");
  Serial.println(siv);

  return true;
}

String sanitizeTimestamp(String timestamp) {
  if (timestamp.length() == 0) timestamp = String(millis());
  timestamp.replace(":", "");
  timestamp.replace("Z", "");
  return timestamp;
}

void closeWaterLandingFiles() {
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  if (profileFile) {
    profileFile.flush();
    profileFile.close();
  }
}

void openNewWaterLandingFiles(const String& timestampRaw) {
  closeWaterLandingFiles();

  String timestamp = sanitizeTimestamp(timestampRaw);

  dataFileName = makeUniqueFileName(
    "landing_" + String(waterLandingNumber) + "_data_" + timestamp,
    ".csv"
  );

  profileFileName = makeUniqueFileName(
    "landing_" + String(waterLandingNumber) + "_ping_profiles_" + timestamp,
    ".txt"
  );

  dataFile = SD.open(dataFileName, FILE_WRITE);
  profileFile = SD.open(profileFileName, FILE_WRITE);

  if (dataFile) {
    dataFile.println("Timestamp,Latitude,Longitude,Pressure,Altitude,Conductivity,Salinity,Temperature,TempPressureSensor,PingDistanceMm,PingConfidence,Error");
    dataFile.flush();
  } else {
    Serial.println("Failed to open new water landing data file.");
  }

  if (!profileFile) {
    Serial.println("Failed to open new water landing profile file.");
  }

  Serial.print("New water landing data file: ");
  Serial.println(dataFileName);

  Serial.print("New water landing profile file: ");
  Serial.println(profileFileName);

  sampleNumber = 0;
  waterLandingNumber++;
}

// =====================================================
// Ping
// =====================================================

bool readPingDistanceOnly() {
  bool ok = ping.request(PingMessageId::PING1D_DISTANCE_SIMPLE);

  if (!ok) {
    pingDistanceMm = 0;
    pingConfidence = 0;
    return false;
  }

  pingDistanceMm = ping.distance();
  pingConfidence = ping.confidence();
  return true;
}

bool readPingWithProfile(const uint8_t** profile, uint16_t* profileLength) {
  bool okDist = ping.request(PingMessageId::PING1D_DISTANCE_SIMPLE);
  bool okProf = ping.request(PingMessageId::PING1D_PROFILE);

  if (!okDist || !okProf) {
    pingDistanceMm = 0;
    pingConfidence = 0;
    *profile = nullptr;
    *profileLength = 0;
    error += "Ping request failed; ";
    return false;
  }

  pingDistanceMm = ping.distance();
  pingConfidence = ping.confidence();
  *profile = ping.profile_data();
  *profileLength = ping.profile_data_length();

  return (*profile != nullptr && *profileLength > 0);
}

// =====================================================
// Atlas
// =====================================================

bool sendAtlasCommandAndDrain(const char* command, int waitMs) {
  memset(AtlasCommand, 0, sizeof(AtlasCommand));
  strncpy(AtlasCommand, command, sizeof(AtlasCommand) - 1);

  Wire.beginTransmission(ATLAS_ADDR);
  Wire.write((uint8_t*)AtlasCommand, strlen(AtlasCommand));
  Wire.endTransmission();

  delay(waitMs);

  Wire.requestFrom(ATLAS_ADDR, 32, 1);

  if (!Wire.available()) {
    atlasCode = 0;
    return false;
  }

  atlasCode = Wire.read();

  while (Wire.available()) {
    Wire.read();
  }

  return atlasCode == 1;
}

bool readAtlasConductivitySalinity(float* cond, float* sal) {
  memset(ec_data, 0, sizeof(ec_data));
  memset(AtlasCommand, 0, sizeof(AtlasCommand));
  strcpy(AtlasCommand, "r");

  Wire.beginTransmission(ATLAS_ADDR);
  Wire.write((uint8_t*)AtlasCommand, strlen(AtlasCommand));
  Wire.endTransmission();

  delay(atlasReadDelayMs);

  Wire.requestFrom(ATLAS_ADDR, 32, 1);

  if (!Wire.available()) {
    atlasCode = 0;
    *cond = -9999.0;
    *sal = -9999.0;
    return false;
  }

  atlasCode = Wire.read();

  if (atlasCode != 1) {
    *cond = -9999.0;
    *sal = -9999.0;
    return false;
  }

  int i = 0;
  while (Wire.available() && i < 31) {
    char c = Wire.read();
    if (c == 0) break;
    ec_data[i++] = c;
  }
  ec_data[i] = 0;

  char* comma = strchr(ec_data, ',');

  if (comma) {
    *comma = 0;
    *cond = atof(ec_data);
    *sal = atof(comma + 1);
    return true;
  }

  *cond = -9999.0;
  *sal = -9999.0;
  return false;
}

bool readAtlasForDetection() {
  enableMuxPort(ATLAS_MUX_PORT);
  bool ok = readAtlasConductivitySalinity(&conductivity, &salinity);
  disableMux();
  delay(20);
  return ok;
}

bool setAtlasTemperatureCompensation(float tempC) {
  char tempCommand[32];
  snprintf(tempCommand, sizeof(tempCommand), "T,%.2f", tempC);

  enableMuxPort(ATLAS_MUX_PORT);
  bool ok = sendAtlasCommandAndDrain(tempCommand, atlasCommandDelayMs);
  disableMux();
  delay(20);

  return ok;
}

bool readAtlasAfterTempComp() {
  enableMuxPort(ATLAS_MUX_PORT);
  bool ok = readAtlasConductivitySalinity(&conductivity, &salinity);
  disableMux();
  delay(20);

  return ok;
}

// =====================================================
// Water detector
// =====================================================

void updateWaterState(bool atlasOk, bool pingOk) {
  bool conductivityWater =
    atlasOk &&
    conductivity > COND_ENTER_THRESHOLD;

  bool pingWater =
    pingOk &&
    pingConfidence >= PING_ENTER_CONF;

  bool pingAir =
    !pingOk ||
    pingConfidence <= PING_EXIT_CONF;

  bool enterCondition = conductivityWater && pingWater;
  bool exitCondition = pingAir;  // exit uses Ping only because Atlas probe can stay wet

  if (!isWater) {
    if (enterCondition) {
      enterCount++;
    } else {
      enterCount = 0;
    }

    if (enterCount >= ENTER_REQUIRED_COUNT) {
      isWater = true;
      enterCount = 0;
      exitCount = 0;
      Serial.println("STATE_CHANGE,WATER");
    }
  } else {
    if (exitCondition) {
      exitCount++;
    } else {
      exitCount = 0;
    }

    if (exitCount >= EXIT_REQUIRED_COUNT) {
      isWater = false;
      enterCount = 0;
      exitCount = 0;
      Serial.println("STATE_CHANGE,AIR");
    }
  }
}

// =====================================================
// SD writing
// =====================================================

void writeProfileToSD(uint32_t sample,
                      const String& timestamp,
                      uint32_t distance_mm,
                      uint16_t confidence,
                      const uint8_t* profile,
                      uint16_t N) {

  if (!profileFile || profile == nullptr || N == 0) return;

  // ============================================
  // Read Ping profile configuration
  // ============================================

  uint32_t scanStartMm = ping.scan_start();
  uint32_t scanLengthMm = ping.scan_length();
  uint32_t speedOfSound = ping.speed_of_sound();

  float binSizeMm = 0.0;

  if (N > 0) {
    binSizeMm = (float)scanLengthMm / (float)N;
  }

  // ============================================
  // Save metadata
  // ============================================

  profileFile.println("----- PROFILE START -----");

  profileFile.print("sample,");
  profileFile.println(sample);

  profileFile.print("timestamp,");
  profileFile.println(timestamp);

  profileFile.print("lat,");
  profileFile.println(lat, 7);

  profileFile.print("lon,");
  profileFile.println(lon, 7);

  profileFile.print("distance_mm,");
  profileFile.println(distance_mm);

  profileFile.print("confidence,");
  profileFile.println(confidence);

  profileFile.print("profile_length,");
  profileFile.println(N);

  profileFile.print("scan_start_mm,");
  profileFile.println(scanStartMm);

  profileFile.print("scan_length_mm,");
  profileFile.println(scanLengthMm);

  profileFile.print("speed_of_sound_mm_per_s,");
  profileFile.println(speedOfSound);

  profileFile.print("bin_size_mm,");
  profileFile.println(binSizeMm, 4);

  // ============================================
  // Profile data
  // ============================================

  profileFile.println("index,intensity");

  for (uint16_t j = 0; j < N; j++) {

    profileFile.print(j);
    profileFile.print(',');

    profileFile.println((uint16_t)profile[j]);
  }

  profileFile.println("----- PROFILE END -----");
  profileFile.println();

  profileFile.flush();
}

// =====================================================
// Notecard transmit
// =====================================================

void transmitMainData(const String& timestamp, const char* deviceType) {
  J *req = notecard.newRequest("note.add");
  if (req == NULL) {
    error += "Failed to create note.add; ";
    return;
  }

  //JAddBoolToObject(req, "sync", false);

  J *body = JCreateObject();
  if (body) {
    JAddNumberToObject(body, "temperature", temperature);
    JAddNumberToObject(body, "conductivity", conductivity);
    JAddNumberToObject(body, "salinity", salinity);
    JAddNumberToObject(body, "pressure", pressure);
    JAddNumberToObject(body, "altitude", altitude);
    JAddNumberToObject(body, "longitude", lon);
    JAddNumberToObject(body, "latitude", lat);
    JAddNumberToObject(body, "tempPress", tempPress);
    JAddNumberToObject(body, "depth", pingDistanceMm);
    JAddNumberToObject(body, "depthConfidence", pingConfidence);
    JAddStringToObject(body, "Timestamp", timestamp.c_str());
    JAddStringToObject(body, "deviceType", deviceType);
    JAddStringToObject(body, "waterState", deviceType);
    JAddStringToObject(body, "error", error.c_str());
    JAddItemToObject(req, "body", body);
  }

  NoteRequest(req);
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  for (unsigned long start = millis(); !Serial && (millis() - start <= 5000); ) { }

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== Drone Water-State Data Collection ===");

  Wire.begin();
  Wire.setClock(100000);
  disableMux();

  // Notecard setup
  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);

  disableMux();

  {
    J *req = notecard.newRequest("hub.set");
    if (req != NULL) {
      JAddStringToObject(req, "product", productUID);
      JAddStringToObject(req, "mode", "continuous");
      NoteRequest(req);
    }
  }

  disableMux();
  J *req = notecard.newRequest("card.location.mode");
  JAddStringToObject(req, "mode", "off");
  notecard.sendRequest(req);

  // Initialize GPS
  enableMuxPort(GPS_MUX_PORT);
  delay(100);

  if (myGNSS.begin(Wire) == false) {
    Serial.print("GPS init FAILED on mux port ");
    if (myGNSS.begin(Wire, 0x42) == false) {
        Serial.println("GPS still failing.");
    }
    error += "GPS disconnected; ";
  } else {
    Serial.print("GPS initialized on mux port ");
    Serial.println(GPS_MUX_PORT);
    myGNSS.setAutoPVT(false); // Disable automatic polling
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(1);
    //myGNSS.setProcessAuto(false);

    // Only use this once if needed, then comment it out to avoid repeated flash writes.
    // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  }

  disableMux();
  delay(20);

  // Initialize pressure sensor
  enableMuxPort(PRESSURE_MUX_PORT);
  delay(50);
  if (pressureSensor.init()) {
    pressureSensor.setModel(MS5837::MS5837_02BA);
    pressureSensor.setFluidDensity(997);
    Serial.print("MS5837 initialized on mux port ");
    Serial.println(PRESSURE_MUX_PORT);
  } else {
    Serial.print("MS5837 init FAILED on mux port ");
    Serial.println(PRESSURE_MUX_PORT);
    error += "MS5837 disconnected; ";
  }
  disableMux();
  delay(20);

  // Initialize temperature sensor
  enableMuxPort(TEMP_MUX_PORT);
  delay(50);
  if (tempSensor.init()) {
    Serial.print("TSYS01 initialized on mux port ");
    Serial.println(TEMP_MUX_PORT);
  } else {
    Serial.print("TSYS01 init FAILED on mux port ");
    Serial.println(TEMP_MUX_PORT);
    error += "TSYS01 disconnected; ";
  }
  disableMux();
  delay(20);

  // Detect Atlas
  enableMuxPort(ATLAS_MUX_PORT);
  delay(50);
  Wire.beginTransmission(ATLAS_ADDR);
  byte i2cerror = Wire.endTransmission();
  if (i2cerror == 0) {
    Serial.print("Atlas sensor detected on mux port ");
    Serial.println(ATLAS_MUX_PORT);
  } else {
    Serial.print("Atlas sensor NOT detected on mux port ");
    Serial.println(ATLAS_MUX_PORT);
    error += "Atlas disconnected; ";
  }
  disableMux();
  delay(20);

  // Ping
  PingSerial.begin(115200, SERIAL_8N1, PING_RX, PING_TX);
  delay(200);

  while (!ping.initialize()) {
    Serial.println("Ping init failed. Check wiring.");
    delay(1000);
  }

  ping.set_mode_auto(true);
  Serial.println("Ping initialized with auto mode enabled.");

  // SD
  SPI.begin(sck, miso, mosi, cs);
  if (!SD.begin(cs)) {
    Serial.println("SD card mount failed");
    error += "MicroSD reader disconnected; ";
    return;
  }

  dataFileName = "";
  profileFileName = "";

  Serial.println("Initialization complete.");
  Serial.println("Air mode: transmitting full sensor data without local profile logging.");
}

// =====================================================
// Loop
// =====================================================

void loop() {
  error = "";

  // Always poll only detector sensors first.
  bool pingOk = readPingDistanceOnly();
  bool atlasOk = readAtlasForDetection();

  bool wasWater = isWater;
  updateWaterState(atlasOk, pingOk);

  if (!wasWater && isWater) {
    String landingTimestamp = getCardTime();
    openNewWaterLandingFiles(landingTimestamp);
  }

  if (wasWater && !isWater) {
    closeWaterLandingFiles();
    Serial.println("Closed water landing files.");
  }

  Serial.print(isWater ? "Water" : "Air");
  Serial.print(" detector_cond=");
  Serial.print(conductivity, 2);
  Serial.print(" detector_sal=");
  Serial.print(salinity, 3);
  Serial.print(" ping_conf=");
  Serial.print(pingConfidence);
  Serial.print(" ping_mm=");
  Serial.print(pingDistanceMm);
  Serial.print(" enterCount=");
  Serial.print(enterCount);
  Serial.print(" exitCount=");
  Serial.println(exitCount);

  // ===================================================
  // AIR MODE:
  // Do not transmit.
  // Do not save.
  // Do not poll GPS, pressure, or temp.
  // Only keep checking detector sensors.
  // ===================================================

  if (!isWater) {
    Serial.println("AIR: no transmit, no save, detector only.");
    delay(1000);
    return;
  }

  // ===================================================
  // WATER MODE ONLY:
  // Now collect full payload, save, and transmit.
  // ===================================================

  disableMux();
  delay(20);

  measurementTimestamp = getCardTime();
  if (measurementTimestamp.length() == 0) {
    measurementTimestamp = String(millis());
    error += "Failed to get card time; ";
  }

  if (!readGPSLocation()) {
    lat = 0.0;
    lon = 0.0;
  }

  // Read temperature first for Atlas temperature compensation.
  enableMuxPort(TEMP_MUX_PORT);
  delay(20);
  tempSensor.read();
  temperature = tempSensor.temperature();
  disableMux();
  delay(20);

  // Send Atlas temperature compensation.
  if (!setAtlasTemperatureCompensation(temperature)) {
    error += "Atlas temp comp failed; ";
  }

  // Read Atlas again after temperature compensation.
  if (!readAtlasAfterTempComp()) {
    error += "Atlas read failed; ";
  }

  // Read pressure.
  enableMuxPort(PRESSURE_MUX_PORT);
  delay(20);
  pressureSensor.read();
  pressure = pressureSensor.pressure();
  tempPress = pressureSensor.temperature();
  altitude = calculateAltitude(pressure * 100.0, tempPress);
  disableMux();
  delay(20);

  // Read Ping profile and current distance/confidence.
  const uint8_t* profile = nullptr;
  uint16_t profileLength = 0;

  bool profileOk = readPingWithProfile(&profile, &profileLength);

  delay(20);

  if (profileOk) {
    writeProfileToSD(sampleNumber, measurementTimestamp, pingDistanceMm, pingConfidence, profile, profileLength);
  }

  writeMainDataToSD(measurementTimestamp);

  disableMux();
  delay(20);
  transmitMainData(measurementTimestamp, DEVICE_TYPE_WATER);

  Serial.print("TRANSMITTED WATER sample=");
  Serial.print(sampleNumber);
  Serial.print(" time=");
  Serial.print(measurementTimestamp);
  Serial.print(" lat=");
  Serial.print(lat, 7);
  Serial.print(" lon=");
  Serial.print(lon, 7);
  Serial.print(" temp=");
  Serial.print(temperature, 2);
  Serial.print(" cond=");
  Serial.print(conductivity, 2);
  Serial.print(" sal=");
  Serial.print(salinity, 3);
  Serial.print(" pressure=");
  Serial.print(pressure, 2);
  Serial.print(" ping_mm=");
  Serial.print(pingDistanceMm);
  Serial.print(" conf=");
  Serial.print(pingConfidence);
  Serial.print(" error=");
  Serial.println(error);

  sampleNumber++;

  delay(500);
}
