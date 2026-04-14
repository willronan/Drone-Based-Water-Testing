#include <SPI.h>
#include <SD.h>

#define CS_PIN 21  // Change this to your actual chip select pin

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for Serial port to connect
  }

  Serial.println("Initializing SD card...");

  if (!SD.begin(CS_PIN)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // Start at the root directory
  File root = SD.open("/");
  printDirectory(root, 0);
  root.close();
}

void loop() {
  // Nothing here
}

// Recursive function to print directory contents
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
