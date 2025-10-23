/*
  This sketch is used for the embedded sensor suite of the
  drone water quality monitoring system. It handles the collection
  of sensor and GPS data, and the transmission to the Arduino cloud
  over LTE.
*/


/*
  LIBRARY DECLARATIONS
*/
#include <Notecard.h>
#include "thingProperties.h"
#include "TSYS01.h"
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "MS5837.h"
#include <Wire.h>


/*
  HARDWARE DEFINITIONS
*/
#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;                         // LED definition
#endif


/*
  Basic control and commands for the PCA9548A/TCA9548A I2C multiplexer
*/

#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address


#define address 100       // I2C ID number for salinity sensor      
char computerdata[32];    // buffer used for communication from computer to integrated sality sensor circuit

TSYS01 tempSensor;        // Blues Robotics temperature sensor 

MS5837 pressureSensor;

TinyGPSPlus gps;

// SPI pins for microSD data saving
static const int sck = 5;
static const int miso = 19;
static const int mosi = 18;
static const int cs = 21;
// UART pins for GPS module
static const int RXPin = 16, TXPin = 17;
HardwareSerial gpsSerial(1);

// Reference sea-level pressure (Pa)
const float P0 = 101325.0;

// Physical constants
const float R = 8.3144598;     // J/(mol*K)
const float g = 9.80665;       // m/s^2
const float M = 0.0289644;     // kg/mol

#define NUMBER_OF_SENSORS 3


#define productUID "com.gmail.willronan4:drone"


/*
  GLOBAL VARIABLES
*/
File file;
const uint32_t GPS_BAUD = 9600;
int time_ = 570;                 // delay for between salinity measurments
char AtlasCommand[32] = "r";     // read command to poll salinity sensor
byte code = 0;                   // holds sensor response
byte in_char = 0;                // 1 byte buffer to store inbound bytes from the EC Circuit
byte i = 0;                      // counter used for ec_data array
char ec_data[32];                // 32 byte character array to hold incoming data from the EC circuit.

float pressure = 0;

/*
  LOCAL DATA SAVING VARIABLES
*/
bool microSdSafety = false;        // ensures data file gets created
int fileCount = 0;

/*
 * Choose an interrupt capable pin to reduce polling and improve
 * the overall responsiveness of the ArduinoIoTCloud library
 */
// #define ATTN_PIN 9


//Enables a specific port number
void enableMuxPort(byte portNumber) {
  if (portNumber > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << portNumber);
  Wire.endTransmission();
}

void disableMuxPort(byte portNumber) {
  (void)portNumber;  // Unused, but kept for clarity
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0);  // disable all ports
  Wire.endTransmission();
}

/*

  SYSTEM SETUP

 */
void setup() {
  // Initialize serial and wait up to 5 seconds for port to open 
  Serial.begin(9600);
  for(unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime <= 5000); ) { }

  Serial.println(0);
  /* Set the debug message level:
   * - DBG_ERROR: Only show error messages
   * - DBG_WARNING: Show warning and error messages
   * - DBG_INFO: Show info, warning, and error messages
   * - DBG_DEBUG: Show debug, info, warning, and error messages
   * - DBG_VERBOSE: Show all messages
   */
  setDebugMessageLevel(DBG_INFO);

  // Configure LED pin as an output 
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(1);

  // initialize sensor data 
  lat = 0.000000;
  lon = 0.000000;

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXPin, TXPin); 


  //  I2C
  Wire.begin();
  //Disable all eight mux ports initially, then we can enable them one at a time
  for (byte x = 0 ; x <= 7 ; x++)
  {
    disableMuxPort(x);
  }


 // --- Port 0: MS5837 (pressure sensor) ---
  enableMuxPort(0);
  delay(50);
  if (pressureSensor.init()) {
    pressureSensor.setModel(MS5837::MS5837_02BA);

    pressureSensor.setFluidDensity(997); // freshwater
    Serial.println("Port 0: MS5837 initialized!");
  } else {
    Serial.println("Port 0: MS5837 init FAILED!");
  }
  disableMuxPort(0);

  // --- Port 3: TSYS01 (temperature sensor) ---
  enableMuxPort(3);
  delay(50);
  if (tempSensor.init()) {
    Serial.println("Port 3: TSYS01 initialized!");
  } else {
    Serial.println("Port 3: TSYS01 init FAILED!");
  }
  disableMuxPort(3);

  // --- Port 4: Atlas Salinity sensor ---
  enableMuxPort(4);
  delay(50);
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  if (error == 0)
    Serial.println("Port 4: Atlas Salinity sensor detected!");
  else
    Serial.println("Port 4: Atlas sensor NOT detected!");
  disableMuxPort(4);

  Serial.println("Initialization complete.\n");

  // connects variables to arduino cloud
  initProperties();

  /* Initialize Arduino IoT Cloud library */
#ifndef ATTN_PIN
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.setNotecardPollingInterval(2000);  // default: 1000ms, min: 250ms
#else
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, ATTN_PIN);
#endif
  ArduinoCloud.printDebugInfo();
}

/*

  COLLECT & TRANSMIT DATA

 */
void loop() {

    // update cloud variables
  if(ArduinoCloud.connected()){
    ArduinoCloud.update();
  } 

  // DATA COLLECTION HAPPENS HERE 
  if (led == true){

    //poll pressure sensor
    enableMuxPort(0);
    delay(10);
    pressureSensor.read();
    pressure = pressureSensor.pressure();
    altitude = calculateAltitude(pressureSensor.pressure()*100, pressureSensor.temperature());
    disableMuxPort(0);

    //poll temp sensor
    enableMuxPort(3);
    delay(10);
    tempSensor.read();
    temperature = tempSensor.temperature();
    disableMuxPort(3);
  
    // --- Port 4: Atlas Salinity Sensor ---
    enableMuxPort(4);
    delay(10);
    float salinity = readAtlasSensor();
    disableMuxPort(4);


    getGPS(&lat, &lon); 
    location = Location(lat, lon);
    
    delay(100);

    Serial.print("Salinity: ");
    Serial.println(salinity);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Altitude");
    Serial.println(altitude);
    
  

    // save in microSD file
    if(microSdSafety == true){
      Serial.println("calling writeToSD");
      writeToSD(time_and_date, lat, lon, altitude, salinity, temperature, note, file);

    }

  }
}

/*
 * 'onLedChange' is called when the "led" property of your Thing changes
 */
void onLedChange() {
  Serial.println(led);
  digitalWrite(LED_BUILTIN, led);
}

void onMicroSDChange(){
  if(microSD){

    SPI.begin(sck, miso, mosi, cs);     //  SPI
    if (!SD.begin(cs)) {                
      Serial.println("Card Mount Failed");
      error = "MicroSD reader disconnected";
      return;
    }

    time_and_date = getCardTime();
    time_and_date.replace(":", "");  // Replace colons with underscores
    time_and_date.replace("Z", "");   // Optional: remove 'Z'
    fileCount = fileCount + 1;
    String fileName = "/drone_reading_" + time_and_date + "_session" + String(fileCount) + ".txt";

    // Close the old file if it's open
    if (file) {
      file.close();
    }


    file = SD.open(fileName, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      // if error blink LED
      signalErrorWithLED();
    }
    file.println("Logging data:");
    Serial.println("Created new local data file" + fileName);
    
    microSdSafety = true;
  }
  if (!microSD){
    microSdSafety = false;
    SD.end();  // deinitializes SPI and unmounts the card
    SPI.end();
    Serial.printf("SPI ended, safe to eject microSD");

  }
}

void onNoteChange(){
  delay(0);

}


float calculateAltitude(float P, float T_c) {
  // Convert temp to Kelvin
  float T = T_c + 273.15;

  // Hypsometric equation
  float factor = (R * T) / (g * M);
  return factor * log(P0 / P);
}

void getGPS(double* lat, double* lon){   
  Serial.println("getting GPS");
  if (!gpsSerial.available()){
    error = "GPS disconnected";
  }

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    *lat = gps.location.lat();
    *lon = gps.location.lng();
  } else {
    Serial.print(F("INVALID")); 

    *lat = -27.108573;
    *lon = -109.291542;
  }
}

// COLLECT TIME STAMP FOR DATA POINTS
String getCardTime(){
    // get current time
  J *req = NoteNewRequest("card.time");
  if (J *rsp = NoteRequestResponse(req))
  {

    int time = JGetNumber(rsp, "time");
    int minutes = JGetNumber(rsp, "minutes");
    String time_and_date = getDateTimeFromUnix(time, minutes);

    NoteDeleteResponse(rsp);
    return time_and_date;
  }
}


// change a unix timestamp to a date time variable
String getDateTimeFromUnix(time_t time, int minutesOffset) {

  time_t adjustedTime = time + (minutesOffset * 60);  // Adjust for timezone
  struct tm timeInfo;
  gmtime_r(&adjustedTime, &timeInfo);  // Convert to UTC struct tm

  char buffer[25];  // Enough space for "YYYY-MM-DDTHH:MM:SSZ"
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
            timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
            timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  return String(buffer);
}

// COLLECT SALINITY SENSOR DATA
float readAtlasSensor(){
  Wire.beginTransmission(address);                                            //call the circuit by its ID number.                        // temp calibration
  Wire.write((uint8_t *)AtlasCommand, strlen(AtlasCommand));                  //transmit the command that was sent through the serial port.
  Wire.endTransmission();                                                     //end the I2C data transmission.

  delay(time_);

  Wire.requestFrom(address, 32, 1);

  code = Wire.read();

  switch (code) {                           //switch case based on what the response code is.
    case 1:                                 //decimal 1.
      //Serial.println("Success");            //means the command was successful.
      break;                                //exits the switch case.

    case 2:                                 //decimal 2.
      Serial.println("Failed");             //means the command has failed.
      break;                                //exits the switch case.

    case 254:                               //decimal 254.
      Serial.println("Pending");            //means the command has not yet been finished calculating.
      break;                                //exits the switch case.

    case 255:                               //decimal 255.
      Serial.println("No Data");            //means there is no further data to send.
      break;                                //exits the switch case.
  }

  while (Wire.available()) {                 //are there bytes to receive.
    in_char = Wire.read();                   //receive a byte.
    ec_data[i] = in_char;                    //load this byte into our array.
    i += 1;                                  //incur the counter for the array element.
    if (in_char == 0) {                      //if we see that we have been sent a null command.
      i = 0;                                 //reset the counter i to 0.
      break;                                 //exit the while loop.
    }
  }

  Serial.println(ec_data);                  //print the data.
  return atof(ec_data);

}

void writeToSD(String wtime, double latitude, double longitude, double alt, double sal, double temp, String notepad, File &f){
  Serial.println("In writeToSD");
  f.print(notepad);
  f.print(F(","));
  f.print(wtime);
  f.print(F(","));
  f.print(latitude, 6);
  f.print(F(","));
  f.print(longitude, 6);
  f.print(F(","));
  f.print(alt);
  f.print(F(","));
  f.print(sal);
  f.print(F(","));
  f.println(temp);
  f.flush();
  Serial.println("Line flushed");
}

void signalErrorWithLED(){
  while(true){
    digitalWrite(LED_BUILTIN, true);
    delay(1000);
    digitalWrite(LED_BUILTIN, false);
    delay(500);
    digitalWrite(LED_BUILTIN, true);
    delay(500);
    digitalWrite(LED_BUILTIN, false);
    delay(350);
    digitalWrite(LED_BUILTIN, true);
    delay(350);
    digitalWrite(LED_BUILTIN, false);
    delay(250);
    digitalWrite(LED_BUILTIN, true);
    delay(250);
    digitalWrite(LED_BUILTIN, false);
    delay(200);
    digitalWrite(LED_BUILTIN, true);
    delay(200);
    digitalWrite(LED_BUILTIN, false);
    delay(150);
    digitalWrite(LED_BUILTIN, true);
    delay(150);
    digitalWrite(LED_BUILTIN, false);
    delay(125);
    digitalWrite(LED_BUILTIN, true);
    delay(125);
    digitalWrite(LED_BUILTIN, false);
    delay(100);
    digitalWrite(LED_BUILTIN, true);
    delay(100);
    digitalWrite(LED_BUILTIN, false);
    delay(75);
    digitalWrite(LED_BUILTIN, true);
    delay(75);
    digitalWrite(LED_BUILTIN, false);
    delay(50);
    digitalWrite(LED_BUILTIN, true);
    delay(50);
    digitalWrite(LED_BUILTIN, false);
    delay(50);
    digitalWrite(LED_BUILTIN, true);
    delay(50);
    digitalWrite(LED_BUILTIN, false);
    delay(50);
    digitalWrite(LED_BUILTIN, true);
    delay(50);
    digitalWrite(LED_BUILTIN, false);
    delay(1000);
  }
}


