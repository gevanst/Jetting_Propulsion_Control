// Teensy 4.1 Script: Read AMT22 Encoder Data with Checksum Verification

#include <SPI.h>

// Pin Definitions
const int encoderCS = 0;  // Chip Select
const int encoderMISO = 1;  // Alternate MISO pin (SPI1)
const int encoderMOSI = 26; // Alternate MOSI pin (SPI1)
const int encoderSCK = 27;  // Alternate SCK pin (SPI1)

// SPI Settings
const SPISettings encoderSPISettings(1000000, MSBFIRST, SPI_MODE0);

// Encoder Resolution (12-bit or 14-bit)
const int RESOLUTION = 12;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial monitor to connect
    }

    pinMode(encoderCS, OUTPUT);
    digitalWrite(encoderCS, HIGH); // Set CS inactive
    
    SPI1.begin(); // Initialize SPI1 communication

    Serial.println("AMT22 Encoder Reader Initialized with Checksum Verification.");
}

void loop() {
    uint16_t position = readEncoder();
    if (position == 0xFFFF) {
        Serial.println("Error: Encoder not responding or checksum invalid.");
    } else {
        Serial.print("Position: ");
        Serial.println(position); // Print valid position to Serial monitor
    }
    delay(100);
}

uint16_t readEncoder() {
    SPI1.beginTransaction(encoderSPISettings);
    digitalWrite(encoderCS, LOW); // Activate CS
    delayMicroseconds(10);

    // Send and receive data
    uint8_t highByte = SPI1.transfer(0x00);
    delayMicroseconds(10);
    uint8_t lowByte = SPI1.transfer(0x00);
    delayMicroseconds(10);

    digitalWrite(encoderCS, HIGH); // Deactivate CS
    SPI1.endTransaction();

    uint16_t encoderPosition = (highByte << 8) | lowByte;

    // Verify checksum
    if (verifyChecksumSPI(encoderPosition)) {
        encoderPosition &= 0x3FFF; // Discard upper two checksum bits
        if (RESOLUTION == 12) encoderPosition >>= 2; // Adjust for 12-bit resolution
        return encoderPosition; // Return valid position
    } else {
        Serial.println("Checksum verification failed.");
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
