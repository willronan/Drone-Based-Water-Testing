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

/*
  HARDWARE DEFINITIONS
*/
#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;                         // LED definition
#endif

#define address 100       // I2C ID number for salinity sensor      
char computerdata[32];    // buffer used for communication from computer to integrated sality sensor circuit

TSYS01 tempSensor;        // Blues Robotics temperature sensor 

TinyGPSPlus gps;

// SPI pins for microSD data saving
static const int sck = 5;
static const int miso = 19;
static const int mosi = 18;
static const int cs = 21;
// UART pins for GPS module
static const int RXPin = 16, TXPin = 17;
SoftwareSerial ss(RXPin, TXPin);


/*
  GLOBAL VARIABLES
*/
File file;
int time_ = 570;                 // delay for between salinity measurments
char AtlasCommand[32] = "r";     // read command to poll salinity sensor
byte code = 0;                   // holds sensor response
byte in_char = 0;                // 1 byte buffer to store inbound bytes from the EC Circuit
byte i = 0;                      // counter used for ec_data array
char ec_data[32];                // 32 byte character array to hold incoming data from the EC circuit.

bool fileCreated = false;        // ensures data file gets created

/*
 * Choose an interrupt capable pin to reduce polling and improve
 * the overall responsiveness of the ArduinoIoTCloud library
 */
// #define ATTN_PIN 9


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
  double salinity = 0;
  double temperature = 0;
  double lat = 0.000000;
  double lon = 0.000000;
  double lastLat = 0.000000;
  double lastLon = 0.000000;


  
  //  UART defined in 
  Wire.begin();                       //  I2C 
  tempSensor.init();                  //  I2C
  SPI.begin(sck, miso, mosi, cs);     //  SPI
  if (!SD.begin(cs)) {                
    Serial.println("Card Mount Failed");
    return;
  }
  Serial.println(2);

  // connects variables to arduino cloud
  initProperties();
  Serial.println(3);

  /* Initialize Arduino IoT Cloud library */
#ifndef ATTN_PIN
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.setNotecardPollingInterval(3000);  // default: 1000ms, min: 250ms
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

  if(!fileCreated){
    file = SD.open("/drone_data.txt", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      // if error blink LED
      signalErrorWithLED();
    }
    file.println("Logging data:");
    fileCreated = true;
    Serial.println(5);
  }


  if (led == true){ 
    // collect data
    salinity = readAtlasSensor();
    temperature = tempSensor.temperature(); 
    tempSensor.read();
    getGPS(&lat, &lon);
    location = Location(lat, lon);
    time_and_date = getCardTime();

    // save in microSD file
    writeToSD(time_and_date, lat, lon, salinity, temperature, file);


  }
}

/*
 * 'onLedChange' is called when the "led" property of your Thing changes
 */
void onLedChange() {
  Serial.print("LED set to ");
  Serial.println(led);
  digitalWrite(LED_BUILTIN, led);
}


float readAtlasSensor(){

  Wire.beginTransmission(address);                                            //call the circuit by its ID number.
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

  //Serial.println(ec_data);                  //print the data.
  return atof(ec_data);

}

void getGPS(double* lat, double* lon){     
  while (ss.available() > 0) {
    if (gps.encode(ss.read())){ 
      *lat = gps.location.lat();
      *lon = gps.location.lng();
    } else {
      Serial.print(F("INVALID"));
    }
  }
}

String getCardTime(){
      // get current time
    J *req = NoteNewRequest("card.time");
    if (J *rsp = NoteRequestResponse(req))
    {

      int time = JGetNumber(rsp, "time");
      int minutes = JGetNumber(rsp, "minutes");
      String time_and_date = getDateTimeFromUnix(time, minutes);

      NoteDeleteResponse(rsp);
    }
  return time_and_date;
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

void writeToSD(String wtime, double latitude, double longitude, double sal, double temp, File &f){
  f.print(wtime);
  f.print(F(","));
  f.print(latitude, 6);
  f.print(F(","));
  f.print(longitude, 6);
  f.print(F(","));
  f.print(sal);
  f.print(F(","));
  f.println(temp);
  f.flush();
}

void signalErrorWithLED(){
  while(true){
    digitalWrite(LED_BUILTIN, true);
    delay(100);
    digitalWrite(LED_BUILTIN, false);
  }
}