// main control script for the pulsed jetting device dodisicus for jetting thrust research
// HARDWARE:
// MCU - teensy 4.1 600 MHz clock
// Encoder - AMT 222A-V absoulte 12-bit encoder with SPI communication (get pos command: 0X00 0X00)
// SD card - Samsung 128 GB pro ultimate sdxc UHS-1 formatted: FAT32
// Motor Driver IC - TI LMD18200 3A, 55V H-Bridge
// DC Motor - Pittman brushed dc motor 12v
// Battery - 2200 mAh 3 cell 11.1v LiPO battery
// DC-DC converter - PTR08060W 12v->5v
// logs: time, shaft pos, shaft vel, shaft acceleration, desired shaft vel, desired shaft acceleration, motor current

#include <SD.h>
#include <SPI.h>
#include <ADC.h>
#include <ADC_util.h>
#include <Metro.h>

String Prog = "prog_05Hz_256pts.txt"; // velocity program name

const int chipSelect = BUILTIN_SDCARD; // set up built in sd card slot for data logging; SD card formatted as FAT32
FsFile logFile;

const int ledPin = LED_BUILTIN; // use built in led for visual cues

const int MotorCurrent = A9; // ADC0 for current sense on motor driver (pin 8 on ic)

const int chipSelect1 = 0; // 0 pin for CS1; this is for the encoder; MISO-1, MOSI-26, SCK-27, gnd, 5v

const int mPWM = 3; //pwm signal for motor driver IC (pwm timer: FlexPWM4.2, pin 3 also on same timer)
const int mDIR = 2; // direction signal for motor driver IC (high or low to switch direcitons)

const int MagS = 4; //input pin for magnetic sensor to turn on jetting

const int Kp = 40; // Proportional Control Gain
const int Kd = 1; // Derivative Control Gain
const int Ki = 20; // Integral Control Gain

bool LastSwitchState = HIGH; //mag switch/sensor is normally open (hamlin 59140 1-U-02-A)
bool SwitchState = LOW;
int State = 0; //0-idle, 1-running
const unsigned long debounce = 500; //debounce interval (ignores switches in mag state shorter than 0.5s)
unsigned long LastSwitchTime = 0;

float int velProgram[256][3]; //change length based on type of program loaded
int velProgLength = 0;
float currentVelocity = 0;
float currentAcceleration = 0;
float desiredVelocity = 0;
float desiredAcceleration = 0;

float velocityHistory[5] = {0}; // History for averaging velocity
float accelerationHistory[5] = {0}; // History for averaging acceleration
int historyIndex = 0;

Metro logTimer(2);

void setup() {
  pinMode(chipSelect1, OUTPUT); // set up chipselct pin mode for encoder comms
  digitalWrite(chipSelect1, HIGH); // set encoder to inactive state

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(MotorCurrent, INPUT);

  pinMode(mPWM, OUTPUT); // control pwm via analogWrite(pin, value) value between 0 and 256, 
  pinMode(mDIR, OUTPUT);

  pinMode(MagS, INPUT); // will read as high or low

  SPI1.begin();

  if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) { // Initialize SD card with DMA support
      while (true) {//visual warning it failed
          digitalWrite(ledPin, HIGH);// toggle the led on and off indicating that it didn't initalize the sd card
          delay(100);
          digitalWrite(ledPin, LOW);
          delay(100);
      }
  }

  //***********drive the motor to top dead center (where enc position = 0), read current pos, drive motor at 15% duty cycle until 0 is reached and stop motor
  driveToTDC();
  //***********

  //***********load velocty program off of SD card to a matrix that will be interpolated with in the main PID loop
  loadVelProgram();
  //***********

  initializeLogFileCount();

}

void loop() {
  SwitchState = digitalRead(MagS);
  if (SwitchState == HIGH && LastSwitchState == LOW) {
    if(millis() - LastSwitchTime >= debounce) {
      if(State == 0) {
        State = 1;
        digitalWrite(ledPin, HIGH);
        createNewLogFile();
      }
      else {
        State = 0;
        digitalWrite(ledPin, LOW);
        if (logFile) {
          logFile.close();
        }
      }
      lastSwitchTime = millis(); //update switch time
    }
  }

  if (logTimer.check() && State == 1) {
    logData();
  }

}

uint16_t readEncoder() {
    SPI1.beginTransaction(encoderSPISettings);
    digitalWrite(encoderCS, LOW); // Activate CS
    delayMicroseconds(3);
    uint8_t highByte = SPI1.transfer(0x00);
    delayMicroseconds(3);
    uint8_t lowByte = SPI1.transfer(0x00);
    delayMicroseconds(3);
    digitalWrite(encoderCS, HIGH); // Deactivate CS
    SPI1.endTransaction();
    uint16_t encoderPosition = (highByte << 8) | lowByte;
    if (verifyChecksumSPI(encoderPosition)) { //verify data
        encoderPosition &= 0x3FFF; // Discard upper two checksum bits
        if (RESOLUTION == 12) encoderPosition >>= 2; // Adjust for 12-bit resolution
        return encoderPosition; // Return valid position
    } else {
        return 0xFFFF; // Return error code if bad
    }
}

bool verifyChecksumSPI(uint16_t message) {
    uint16_t checksum = 0x3;
    for (int i = 0; i < 14; i += 2) {
        checksum ^= (message >> i) & 0x3;
    }
    return checksum == (message >> 14);
}

void initializeLogFileCount() {
    while (true) {
        String fileName = "Log_05Hz_256pts_" + String(logFileCount) + ".txt";
        if (!SD.sdfs.exists(fileName.c_str())) {
            break;
        }
        logFileCount++;
    }
}

void createNewLogFile() {
    String fileName = "Log_05Hz_256pts_" + String(logFileCount) + ".txt";
    logFile = SD.sdfs.open(fileName.c_str(), O_WRITE | O_CREAT | O_TRUNC);// open for write, create file if doens't exist, overwrite if file exists
    if (logFile) {
        logFileCount++; // Increment the file count for the next log file
        logFile.println("Time (ms), Position")
    }
}

void logData() {
    if (systemState == 1 && logFile) {
        unsigned long currentTime = millis();
        uint16_t position = readEncoder();
        logFile.print(currentTime);//csv format
        logFile.print(", ");
        logFile.println(position);
    }
}

void loadVelProgram() {
  File file = SD.sdsf.open(Prog.c_str(), O_READ);
  if (file) {
    velProgLength = 0;
    while (file.available() && velProgLength < 256) {
      String line = file.readStringUntil('\n'); //read until the new line
      int separator1 = line.indexOf(',');//find first comma in string
      int separator2 = line.indexOf(',', separator1 + 1);//find second comma in string by starting after the first comma
      if (separator1 > 0 && separator2 > 0) {
        velocityProgram[velProgLength][0] = line.substring(0, separator1).toInt(); // Position
        velocityProgram[velProgLength][1] = line.substring(separator1 + 1, separator2).toInt(); // Velocity
        velocityProgram[velProgLength][2] = line.substring(separator2 + 1).toInt(); // Acceleration
        velProgLength++;
    }
    }
  }
}

void driveToTDC() {
    uint16_t position = ReadEnc();
    unsigned long lastUpdateTime = millis();
    while (millis() - lastUpdateTime < 5000) { // Timeout after 5 seconds
        position = ReadEnc();
        if (position == 0) {
            setMotor(1, 0, mPWM, mDIR); // Stop motor at TDC
            break;
        }
        setMotor(1, 64, mPWM, mDIR); // Small duty cycle to move toward TDC
        delay(10); // Allow motor to move slightly before rechecking position
    }
    setMotor(1, 0, mPWM, mDIR); // Ensure motor stops in case of timeout
}

//############# set motor function #####################
void setMotor(int dir, int dc, int pwmPin, int dirPin) {
    if (dc <= 5) {
        digitalWrite(pwmPin, LOW); // Turn off the motor
    } else if (dc >= 250) {
        digitalWrite(pwmPin, HIGH); // Full duty cycle
    } else {
        analogWrite(pwmPin, dc); // Set PWM duty cycle
    }
    digitalWrite(dirPin, dir == 1 ? LOW : HIGH);
}
//############ end of set motor function ##############