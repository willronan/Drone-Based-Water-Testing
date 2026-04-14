#include <SPI.h>
#include <SD.h>

#define CS_PIN 21  // Adjust this to your actual CS pin (e.g. 5, 21, 4, etc.)

/*
First add the write file name you which to extract.

Then find the right USB Port ID using 
  "ls /dev/tty.*"

  Close arduino and run this in the terminal
  "stty -f /dev/cu.usbserial-0287BD14 115200
  cat /dev/cu.usbserial-0287BD14 > "/Users/williamronan/Desktop/UBC/Drone-Based-Water-Testing/Local_Data_Saving/output.txt"

  Hit the reset button and the text from the file will be printed to ouptut.txt 

  Then use this in the terminal after to get the saved data
  "screen /dev/tty.usbserial-XXXX 115200 | tee /Users/williamronan/Desktop/UBC/Drone-Based-Water-Testing/Local_Data_Saving/output.txt"
*/
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for serial monitor

  Serial.println("Initializing SD...");

  if (!SD.begin(CS_PIN)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialized.");

  const char *filename = "test.txt";  // <-- change this to your target file
  printFile(filename);
}

void loop() {}

void printFile(const char *filename) {
  File file = SD.open(filename);
  if (!file) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    return;
  }

  Serial.print("=== START OF ");
  Serial.print(filename);
  Serial.println(" ===");

  while (file.available()) {
    Serial.write(file.read());
  }

  Serial.print("\n=== END OF ");
  Serial.print(filename);
  Serial.println(" ===");

  file.close();
}
