/*
  This sketch demonstrates how to exchange data between your board and the
  Arduino IoT Cloud, while using the Notecard for wireless communication.

  * Connect a potentiometer (or other analog sensor) to A0.
  * When the potentiometer (or sensor) value changes the data is sent to the Cloud.
  * When you flip the switch in the Cloud dashboard the onboard LED lights gets turned ON or OFF.

  IMPORTANT:
  This sketch works with any Wi-Fi, Cellular, LoRa or Satellite enabled Notecard.

  The full list of compatible boards can be found here:
   - https://github.com/arduino-libraries/ArduinoIoTCloud#what
*/

#include <Notecard.h>
#include "thingProperties.h"
#include "TSYS01.h"

#if !defined(LED_BUILTIN) && !defined(ARDUINO_NANO_ESP32)
static int const LED_BUILTIN = 2;
#endif

// Hardware serial for communicating with Atlas sensor
HardwareSerial myserial(2);

// UART pins for Atlas sensor
#define RXD2 16
#define TXD2 17
TSYS01 tempSensor;

const char *time_and_date;
int time_UNX = 0;
int minutes = 0;
String sensorstring;
String tempsensorstring;
boolean sensor_string_complete = false;  

/*
 * Choose an interrupt capable pin to reduce polling and improve
 * the overall responsiveness of the ArduinoIoTCloud library
 */
// #define ATTN_PIN 9

void setup() {
  /* Initialize serial and wait up to 5 seconds for port to open */
  Serial.begin(9600);
  myserial.begin(9600, SERIAL_8N1, RXD2, TXD2); 
  for(unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime <= 5000); ) { }

  //while (myserial.available() == 0){
  //  Serial.println("UART Unavailable");
  //  delay(1000);
  //}

  /* Set the debug message level:
   * - DBG_ERROR: Only show error messages
   * - DBG_WARNING: Show warning and error messages
   * - DBG_INFO: Show info, warning, and error messages
   * - DBG_DEBUG: Show debug, info, warning, and error messages
   * - DBG_VERBOSE: Show all messages
   */
  setDebugMessageLevel(DBG_INFO);

  /* Configure LED pin as an output */
  pinMode(LED_BUILTIN, OUTPUT);

  salinity = 0;
  temperature = 0;

  Wire.begin();
  tempSensor.init();

  

  /* This function takes care of connecting your sketch variables to the ArduinoIoTCloud object */
  initProperties();

  /* Initialize Arduino IoT Cloud library */
#ifndef ATTN_PIN
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.setNotecardPollingInterval(3000);  // default: 1000ms, min: 250ms
#else
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, ATTN_PIN);
#endif

  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  potentiometer = analogRead(A0);
  seconds = millis() / 1000;

  if (led == true){

    salinity = readAtlasSensor();

    Serial.print("Salinity: ");
    Serial.println(salinity);

    temperature = tempSensor.temperature(); //poll your sensor here;
    tempSensor.read();

    Serial.print("Temperature: ");
    Serial.println(temperature);


    J *req = NoteNewRequest("card.time");
    if (J *rsp = NoteRequestResponse(req))
    {

      lat   = JGetNumber(rsp, "lat");
      lon  = JGetNumber(rsp, "lon");
      gps = Location(lat, lon);;

      longitude = String(lon, 10);
      latitude = String(lat, 10);

      Serial.print("Longitude: ");
      Serial.println(longitude); 
      Serial.print("Latitude: "); 
      Serial.println(latitude);  

      time_and_date = getDateTimeFromUnix(time_UNX, minutes);
      Serial.print("Time&Date: "); 
      Serial.println(time_and_date);  


      NoteDeleteResponse(rsp);

    }
  }

  delay(1000);
}

/*
 * 'onLedChange' is called when the "led" property of your Thing changes
 */
void onLedChange() {
  Serial.print("LED set to ");
  Serial.println(led);
  digitalWrite(LED_BUILTIN, led);
}


// change a unix timestamp to a date time variable
const char *getDateTimeFromUnix(time_t time, int minutesOffset) {

    time_t adjustedTime = time + (minutesOffset * 60);  // Adjust for timezone
    struct tm timeInfo;
    gmtime_r(&adjustedTime, &timeInfo);  // Convert to UTC struct tm

   static char buffer[25];  // Enough space for "YYYY-MM-DDTHH:MM:SSZ"
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
             timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);


    return buffer;
}


float readAtlasSensor(){

  sensorstring = "";

  myserial.print("R");  

  Serial.println("In the atlas function");
  // Read from UART2 and store message until full line received
  while (myserial.available()) {

   //Serial.println("In the atlas while loop");

    char inchar = (char)myserial.read();
    if (inchar == '\r') {  // Message complete
      Serial.print("Received: ");  
      Serial.println(tempsensorstring); 
      sensor_string_complete = true; 
      
    } else {
      tempsensorstring += inchar;  
      Serial.print("Adding char: ");
      Serial.println(inchar);
    }
  }

  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    sensorstring = tempsensorstring;
    sensor_string_complete = false;
    tempsensorstring = "";
    
    return sensorstring.toFloat(); 

  }


}

