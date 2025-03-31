*for full description document, head to:
https://ubcca-my.sharepoint.com/:w:/r/personal/wr2001_student_ubc_ca/Documents/Drone-Based%20Water%20Testing/Designs/CLEAR%20Drone%20AquaDrone%20V1.docx?d=wc9372d95ef984ddfa8d932de3aed7dbb&csf=1&web=1&e=fes6pq

# System Description 

AquaDrone V1.A (shown in Figure 1) was a water-landing drone with the ability to take measurements of the water and transmit data to the cloud. The idea was that it could take pinpoint measurements of the temperature, salinity, dissolved oxygen, etc. across a body of water. Each measurement would be linked to a GPS coordinate giving an understanding of how these characteristics change along the body of water.  

The system was comprised of two main components: the drone, and the sensor suite. The drone (purchased from SwellPro) provided a 4K video stream, easy-to-use flight controls, and the ability to land on and take off from water.  

The sensor suite (shown in Figure 2) was designed and assembled in house, using components ordered from several different manufacturers. An ESP32 microcontroller coded with C was the brain of the box. It was connected to a Notecarrier IoT companion board which provided cellular connectivity.  A temperature sensor over I2C, as well as as Atlas Isolated Carrier Board. This Atlas carrier board easily connected to any Atlas water sensor (salinity, dissolved oxygen, etc.)  and was also conncted via I2C. The on/off switch was wired in series between GND and F-enable, putting the board in reset mode when switched on. The pressure gauge allowed for pressure testing using a Mityvac hand-vacuum-pump. Finally the antenna was the interface for GPS and LTE connectivity. 

## Connectivity  
Blues Notecards: 
The Blues Notecarrier companion board works by communicating with a Blues Notecard. Notecards can be purchased for LTE, WiFi, LoRa (long range), and Satellite communication. Every notecard is made to fit with the Notecarrier.  

### Wifi/LTE:
The WiFi/LTE notecard easily connects and transmits JSON files containing data to the Blues Notehub. From the Notehub, routes can be created to various cloud service providers. The WiFi/LTE notecard comes with a dual antenna for LTE and GPS. 

### LoRa:
The Blues LoRa option comes with LoRa notecards and a LoRa gateway. From a coding perspective, there is no change between using the WiFi/LTE or LoRa. However, the LoRa notecards need to be configured with the gateway. The gateway works as a router connected to WiFi. The notecards are able to transmit data using long wavelength communication over large distances to the gateway. The gateway then transmits the data to the Notehub where it can be routed to a cloud service provider. Information for setting up LoRa is found on the Blues website. 

## Arduino IoT Cloud:
Specific instructions for setting up a connection do to the Arduino IoT Cloud can be found on the Blues website. In general, the connection can be described by the flow chart below. 

Using the ArduinoIoTNotecard library, variables in the Arduino sketch such as temperature measurements and GPS coordinates can be linked to cloud variables in your Arduino IoT Cloud. These variables can be set to update on change, or periodically. When running the sketch on the MCU connected to the notecarrier companion board, these updates will automatically be transmitted to your Blues Notehub, and routed to your Arduino IoT Cloud. Your cloud variables can easily be displayed on an Arduino IoT hub dashboard as shown in Figure 4. 

## Waterproofing 
In order to ensure the electronics were not subject to water damage, a waterproof enclosure was purchased from Serpac. Some components like the sensor and antenna needed to be passed through the electronics box, so cable penetrators were used as shown in Figure 5. 
Cables were pass through these penetrators, and epoxy was used to fill the gaps. The epoxy was mixed, placed in the penetrator with the wires threaded through, and left to dry overnight (Figure 6). When the epoxy dried, the cable penetrators were waterproof. 
The antenna was waterproofed in a different method using a silicon rubber to ensure flexibility. The antenna was placed in a mold and left to dry overnight as shown in Figure 7. After drying, the antenna was cut out into its final shape.  


## Successes 
- Drone flight was successful for taking off and landing on water in low winds and rain 
- Sensor suite could successfully be dipped into water using the drone without any water leakage 
- Data could successfully transmit over LTE including temperature and salinity measurements from the water (although connection strength was poor) 
- Several system error were fixed on the fly. The Atlas sensor was switched from UART to I2C as to not interfere with the  
- LoRa transmission worked to transmit temperature data when tested for 5 hours straight before disconnecting

## Shortcomings  
- While GPS location was found on initial boot-up, the GPS did not actively update the location (likely a software error) 
- Connectivity over LTE was very poor; while data transmitted every 5-10 seconds in Vancouver, the transmission rate was about 1 packet ever 2 minutes in Bamfield 
- Moreover, disconnection revealed an issue where the Arduino Cloud could not be reconnected to without resetting the Cloud server 
  
