// Teensy Encoder Logger for AMT22 12-bit Encoder
// Logs time and position to an SD card in human-readable format

#include <SD.h>
#include <SPI.h>
#include <Metro.h>

// Pin Definitions
const int chipSelect = BUILTIN_SDCARD;
const int encoderCS = 0; // Chip select for encoder
const int ledPin = LED_BUILTIN;
const int magSwitchPin = 4;

const int RESOLUTION = 12;

// State Variables
bool lastSwitchState = HIGH;
bool currentSwitchState = LOW;
int systemState = 0; // 0: idle, 1: running
const unsigned long debounceInterval = 500; // Debounce interval in ms
unsigned long lastSwitchTime = 0;

// SD Card and Logging
FsFile logFile;
int logFileCount = 1;
Metro logTimer(2); // Logging interval 

// SPI Settings for Encoder
const SPISettings encoderSPISettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz clock, SPI mode 0

void setup() {
    pinMode(encoderCS, OUTPUT);
    digitalWrite(encoderCS, HIGH); // Set encoder CS inactive

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    pinMode(magSwitchPin, INPUT);

    SPI1.begin();

    if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) { // Initialize SD card
        while (true) { // Blink LED on failure
            digitalWrite(ledPin, HIGH);
            delay(100);
            digitalWrite(ledPin, LOW);
            delay(100);
        }
    }

    initializeLogFileCount();
}

void loop() {
    currentSwitchState = digitalRead(magSwitchPin);
    if (lastSwitchState != currentSwitchState) {
        lastSwitchTime = millis();
    }

    if (millis() - lastSwitchTime >= debounceInterval) {
        if (currentSwitchState == HIGH && systemState == 0) {
            systemState = 1;
            digitalWrite(ledPin, HIGH);
            createNewLogFile();
        } else if (currentSwitchState == LOW && systemState == 1) {
            systemState = 0;
            digitalWrite(ledPin, LOW);
            if (logFile) {
                logFile.close();
            }
        }
    }

    if (logTimer.check() && systemState == 1) {
        logData();
    }

    lastSwitchState = currentSwitchState; // Reset switch state
}

uint16_t readEncoder() {
    SPI1.beginTransaction(encoderSPISettings);
    digitalWrite(encoderCS, LOW); // Activate CS
    delayMicroseconds(3);

    // Send and receive data
    uint8_t highByte = SPI1.transfer(0x00);
    delayMicroseconds(3);
    uint8_t lowByte = SPI1.transfer(0x00);
    delayMicroseconds(3);

    digitalWrite(encoderCS, HIGH); // Deactivate CS
    SPI1.endTransaction();

    uint16_t encoderPosition = (highByte << 8) | lowByte;

    // Verify checksum
    if (verifyChecksumSPI(encoderPosition)) {
        encoderPosition &= 0x3FFF; // Discard upper two checksum bits
        if (RESOLUTION == 12) encoderPosition >>= 2; // Adjust for 12-bit resolution
        return encoderPosition; // Return valid position
    } else {
        return 0xFFFF; // Return error code
    }
}

bool verifyChecksumSPI(uint16_t message) {
    // Checksum is the inverse of XOR of bits, starting with 0b11
    uint16_t checksum = 0x3;
    for (int i = 0; i < 14; i += 2) {
        checksum ^= (message >> i) & 0x3;
    }
    return checksum == (message >> 14);
}

void initializeLogFileCount() {
    while (true) {
        String fileName = "Log_encoder_" + String(logFileCount) + ".txt";
        if (!SD.sdfs.exists(fileName.c_str())) {
            break;
        }
        logFileCount++;
    }
}

void createNewLogFile() {
    String fileName = "Log_encoder_" + String(logFileCount) + ".txt";
    logFile = SD.sdfs.open(fileName.c_str(), O_WRITE | O_CREAT | O_TRUNC);
    if (logFile) {
        logFileCount++;
        // Write a header for readability
        logFile.println("Time (ms), Position");
    }
}

void logData() {
    if (systemState == 1 && logFile) {
        unsigned long currentTime = millis();
        uint16_t position = readEncoder();

        // Write time and position in CSV format
        logFile.print(currentTime);
        logFile.print(", ");
        logFile.println(position);
    }
}
