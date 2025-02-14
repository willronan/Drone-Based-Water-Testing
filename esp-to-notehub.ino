#include <Notecard.h>
#include "TSYS01.h"
#include <time.h>

#define usbSerial Serial

// Blues notehub product UID
#define productUID "com.gmail.willronan4:ubcdronesensing"

// Instanciate the notecard and temperature sensor
Notecard notecard;
TSYS01 tempSensor;

// Setup time and temprature at global vaiables
const char *time_and_date;
int time = 0;
int minutes = 0;
float temperature 

void setup() {
  delay(2500);
  usbSerial.begin(115200); //921600

  // Connect to notecard
  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);

  //establish notecard to notehub connection
  {
    J *req = notecard.newRequest("hub.set");
    if (req != NULL) {
      JAddStringToObject(req, "product", productUID);
      JAddStringToObject(req, "mode", "continuous");
      notecard.sendRequest(req);
    }
  }

  // setup temperature sensor
  Wire.begin();
  tempSensor.init();
}

void loop() {

  // poll the temprature sensor 
  tempSensor.read();
  temperature = tempSensor.temperature(); //poll your sensor here;

  // send a request to notehub
  {
    J *req = notecard.newRequest("note.add");
    if (req != NULL) {
      JAddBoolToObject(req, "sync", true);

      // create JSON with data to transmit
      J *body = JCreateObject();
      if (body) {
        
        // add temperature variable to JSON
        JAddNumberToObject(body, "temperature", temperature);

        // request the current time from notehub
        J *rsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
        if (rsp != NULL) {

          // extract the unix time stamp and time zone minutes adjustment from the respond
          time = JGetNumber(rsp, "time");
          minutes = JGetNumber(rsp, "minutes");

          // tranform unix time stamp into datetime variable
          time_and_date = getDateTimeFromUnix(time, minutes);

          // add datetime variable to JSON
          JAddStringToObject(body, "time_and_date", time_and_date);

          // cleanup
          notecard.deleteResponse(rsp);
        }
        // add the JSON to the request
        JAddItemToObject(req, "body", body);
      }
      // send the request (send the data) to notehub
      notecard.sendRequest(req);
    }
  }
  delay(15000);
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

