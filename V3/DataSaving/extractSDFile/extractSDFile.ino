#include <SPI.h>
#include <SD.h>

static const int sck  = 5;
static const int miso = 19;
static const int mosi = 18;
static const int cs   = 21;

const char* filename = "/cellmetricssession7.txt";

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing SD...");

  SPI.begin(sck, miso, mosi, cs);

  if (!SD.begin(cs)) {
    Serial.println("SD init failed!");
    return;
  }

  Serial.println("SD init OK.");

  Serial.print("Opening file in READ mode: ");
  Serial.println(filename);

  File f = SD.open(filename, FILE_READ);

  if (!f) {
    Serial.println(" ERROR: Failed to open file for reading");
    return;
  }

  Serial.println("File open SUCCESS. Dumping file contents:\n");

  while (f.available()) {
    Serial.write(f.read());
  }

  f.close();
  Serial.println("\n\n=== FILE READ COMPLETE ===");
}

void loop() {}
