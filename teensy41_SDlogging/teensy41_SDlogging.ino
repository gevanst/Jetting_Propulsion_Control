#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD; // Teensy 4.1's built-in SD card slot
FsFile logFile;

const int ledPin = LED_BUILTIN; // Built-in LED
const unsigned long loggingDuration = 20000; // Log for 20 seconds (in milliseconds)
const unsigned int loggingInterval = 1000; // Log at 1 kHz (1 ms interval)
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize built-in LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Open log file
  logFile = SD.sdfs.open("timelogbatcheck.bin", O_WRITE | O_CREAT | O_TRUNC);
  if (!logFile) {
    Serial.println("Failed to open log file!");
    while (1);
  }

  Serial.println("Logging timestamps to SD card...");
  digitalWrite(ledPin, HIGH); // Turn LED ON to indicate logging
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Log data at 1 kHz (every 1 ms)
  static unsigned long lastLogTime = 0;
  if (currentTime - lastLogTime >= 1 && (currentTime - startTime) < loggingDuration) {
    uint32_t timestamp = millis();
    logFile.write((uint8_t *)&timestamp, sizeof(timestamp));
    lastLogTime = currentTime;
  }

  // Stop logging after 20 seconds
  if (currentTime - startTime >= loggingDuration) {
    Serial.println("Logging complete. Closing log file...");
    logFile.close();
    digitalWrite(ledPin, LOW); // Turn LED OFF to indicate logging stopped
    while (1); // Stop further execution
  }
}
