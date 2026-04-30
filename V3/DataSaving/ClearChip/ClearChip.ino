#include <SPI.h>
#include <SD.h>

#define CS_PIN 21

void setup() {
  Serial.begin(115200);  // match your terminal
  while (!Serial) { ; }

  Serial.println("Initializing SD...");

  if (!SD.begin(CS_PIN)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialized.");

  const char *filename = "test.txt";  // user input
  printFile(filename);
}

void loop() {}

void printFile(const char *filename) {
  // Build corrected path
  String path = String(filename);
  if (!path.startsWith("/")) {
    path = "/" + path;
  }

  File file = SD.open(path.c_str(), FILE_READ);
  if (!file) {
    Serial.print("Failed to open file: ");
    Serial.println(path);
    return;
  }

  Serial.print("=== START OF ");
  Serial.print(path);
  Serial.println(" ===");

  while (file.available()) {
    Serial.write(file.read());
  }

  Serial.print("\n=== END OF ");
  Serial.print(path);
  Serial.println(" ===");

  file.close();
}
