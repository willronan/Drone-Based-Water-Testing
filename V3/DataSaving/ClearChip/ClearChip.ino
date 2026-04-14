#include <SPI.h>
#include <SD.h>

#define CS_PIN 21  // Your SD chip select pin

void deleteAllFiles(File dir);

void setup() {
  Serial.begin(9600);
  delay(2000);  // Give time for Serial monitor to open

  Serial.println("Initializing SD card...");

  if (!SD.begin(CS_PIN)) {
    Serial.println("SD Initialization failed!");
    return;
  }

  Serial.println("SD Initialization done.");

  // Open root directory
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory!");
    return;
  }

  Serial.println("Starting deletion of all files...");
  deleteAllFiles(root);
  root.close();

  Serial.println("All files deleted.");
}

void loop() {
  // Nothing needed here
}

// Recursively delete every file
void deleteAllFiles(File dir) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // No more files
      break;
    }

    if (entry.isDirectory()) {
      Serial.print("Entering directory: ");
      Serial.println(entry.name());

      // Recurse into the directory
      deleteAllFiles(entry);

      entry.close();
    } 
    else {
      // Delete the file
      Serial.print("Deleting file: ");
      Serial.print(entry.name());
      
      String path = "/" + String(entry.name());
      entry.close(); // Must close before deletion

      if (SD.remove(path)) {
        Serial.println("  [OK]");
      } else {
        Serial.println("  [FAILED]");
      }
    }
  }
}
