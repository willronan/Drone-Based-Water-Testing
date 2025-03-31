#include <string.h>

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

/* The Notecard can provide connectivity to almost any board via ESLOV (I2C)
  * or UART. An empty string (or the default value provided below) will not
  * override the Notecard's existing configuration.
  * Learn more at: https://dev.blues.io */
#define NOTECARD_PRODUCT_UID "com.gmail.willronan4:drone_sensing"

/* Uncomment the following line to use the Notecard over UART */
// #define UART_INTERFACE Serial1

void onLedChange();

bool led;
int potentiometer;
int seconds;
float temperature;
float salinity;
CloudLocation gps;
double lon;
double lat;
String longitude;
String latitude;

#ifndef UART_INTERFACE
NotecardConnectionHandler ArduinoIoTPreferredConnection(NOTECARD_PRODUCT_UID);
#else
NotecardConnectionHandler ArduinoIoTPreferredConnection(NOTECARD_PRODUCT_UID, UART_INTERFACE);
#endif

void initProperties() {
  ArduinoCloud.addProperty(led, Permission::ReadWrite).onUpdate(onLedChange);
  //ArduinoCloud.addProperty(potentiometer, Permission::Read).publishOnChange(10);
  //ArduinoCloud.addProperty(seconds, Permission::Read).publishEvery(5 * MINUTES);
  ArduinoCloud.addProperty(temperature, Permission::Read).publishOnChange(0);
  ArduinoCloud.addProperty(salinity, Permission::Read).publishOnChange(0);
  ArduinoCloud.addProperty(latitude, Permission::Read).publishOnChange(0);
  ArduinoCloud.addProperty(longitude, Permission::Read).publishOnChange(0);
  ArduinoCloud.addProperty(gps, Permission::Read).publishOnChange(0);



  if (::strncmp(SECRET_WIFI_SSID, "YOUR_WIFI_NETWORK_NAME", sizeof(SECRET_WIFI_SSID))) {
    ArduinoIoTPreferredConnection.setWiFiCredentials(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  }
}
