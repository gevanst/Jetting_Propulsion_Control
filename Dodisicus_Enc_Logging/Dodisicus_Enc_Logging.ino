// test script to log encoder and time data with a timer to ensure consistent logging. Test SPI communcation, test SD log with DMA drivers.
// HARDWARE:
// MCU - teensy 4.1 600 MHz clock
// Encoder - AMT 222A-V absoulte 12-bit encoder with SPI communication (get pos command: 0X00 0X00)
// SD card - Samsung 128 GB pro ultimate sdxc UHS-1 formatted: FAT32
// Motor Driver IC - TI LMD18200 3A, 55V H-Bridge
// DC Motor - Pittman brushed dc motor 12v
// Battery - 2200 mAh 3 cell 11.1v LiPO battery
// DC-DC converter - PTR08060W 12v->5v
// logs: time, encoder position in csv format

#include <SD.h>
#include <SPI.h>
#include <Metro.h>

// Pin Definitions
const int chipSelect = BUILTIN_SDCARD;
const int encoderCS = 0; // Chip select for encoder
const int ledPin = LED_BUILTIN;
const int magSwitchPin = 4;

const int RESOLUTION = 12; // 12-bit resolution of the encoder

// State Variables
bool lastSwitchState = HIGH;
bool currentSwitchState = LOW;
int systemState = 0; // 0: idle, 1: running
const unsigned long debounceInterval = 500; // Debounce interval in ms
unsigned long lastSwitchTime = 0;

// SD Card and Logging
FsFile logFile;
int logFileCount = 1;
Metro logTimer(2); // Logging interval 1 - 1kHz, 2 - 500Hz, 10 - 100Hz, etc.

// SPI Settings for Encoder
const SPISettings encoderSPISettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz clock, SPI mode 0

const int VELOCITY_SAMPLES = 5;// for velocity calculation
uint16_t positionBuffer[VELOCITY_SAMPLES] = {0};
unsigned long timeBuffer[VELOCITY_SAMPLES] = {0};
int velocityIndex = 0;
bool bufferFilled = false;


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

    // Detect a LOW to HIGH transition (magnet waved past sensor)
    if (currentSwitchState == HIGH && lastSwitchState == LOW) {
        // Ensure the switch is stable for debounceInterval
        if (millis() - lastSwitchTime >= debounceInterval) {
            // Toggle the system state
            if (systemState == 0) {
                systemState = 1; // Start logging
                digitalWrite(ledPin, HIGH); // Turn on LED
                createNewLogFile();
            } else {
                systemState = 0; // Stop logging
                digitalWrite(ledPin, LOW); // Turn off LED
                if (logFile) {
                    logFile.close();
                }
            }

            // Update the last switch time
            lastSwitchTime = millis();
        }
    }

    // Log data only when the system is in state 1
    if (logTimer.check() && systemState == 1) {
        logData();
    }

    // Update the last switch state
    lastSwitchState = currentSwitchState;
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
        String fileName = "Log_encoder_vel" + String(logFileCount) + ".txt";
        if (!SD.sdfs.exists(fileName.c_str())) {
            break;
        }
        logFileCount++;
    }
}

void createNewLogFile() {
    String fileName = "Log_encoder_vel" + String(logFileCount) + ".txt";
    logFile = SD.sdfs.open(fileName.c_str(), O_WRITE | O_CREAT | O_TRUNC);
    if (logFile) {
        logFileCount++;
        // Write a header
        logFile.println("Time (ms), Position (count), Velocity (count/s)");
    }
}

void logData() {
    if (logFile) {
        unsigned long currentTime = millis();
        uint16_t currentPosition = readEncoder();

        // Add current data to buffers
        positionBuffer[velocityIndex] = currentPosition;
        timeBuffer[velocityIndex] = currentTime;

        // Compute velocity if buffer has at least two samples
        float velocity = 0.0;
        if (bufferFilled || velocityIndex > 0) {
            int samplesToUse = bufferFilled ? VELOCITY_SAMPLES : velocityIndex;
            for (int i = 0; i < samplesToUse - 1; i++) {
                int index1 = (velocityIndex + i) % VELOCITY_SAMPLES;
                int index2 = (index1 + 1) % VELOCITY_SAMPLES;

                // Calculate position difference with wraparound handling
                int32_t deltaPos = positionBuffer[index2] - positionBuffer[index1];
                if (deltaPos > 2048) { // Forward wraparound
                    deltaPos -= 4096;
                } else if (deltaPos < -2048) { // Reverse wraparound
                    deltaPos += 4096;
                }

                float deltaTime = (timeBuffer[index2] - timeBuffer[index1]) / 1000.0; // Convert ms to s
                if (deltaTime > 0) {
                    velocity += deltaPos / deltaTime;
                }
            }

            // Average the velocity
            velocity /= (samplesToUse - 1);
        }

        // Log data
        logFile.print(currentTime);
        logFile.print(", ");
        logFile.print(currentPosition);
        logFile.print(", ");
        logFile.println(velocity);

        // Update buffer index
        velocityIndex = (velocityIndex + 1) % VELOCITY_SAMPLES;
        if (velocityIndex == 0) {
            bufferFilled = true;
        }
    }
}


