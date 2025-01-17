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

#define LOG_BUFFER_SIZE 1024 //entries, 4bytes for time, vel, accel, motorcurrent, 2 bytes for the rest
#define VELOCITY_HISTORY_SIZE 50 //
#define RESOLUTION 12 // 12- bit resoltion of the encoder

String Prog = "prog_05Hz_256pts_OSC.txt"; // velocity program name
int velProgram[256][3]; //
int velProgLength = 0;

const SPISettings encoderSPISettings(1000000, MSBFIRST, SPI_MODE0); // 1 MHz clock, SPI mode 0 (encoder is capable of up to 2MHz)

const int chipSelect = BUILTIN_SDCARD; // set up built in sd card slot for data logging; SD card formatted as FAT32
FsFile logFile;
struct LogEntry {
  unsigned long timestamp;
  uint16_t position;
  float velocity;
  float acceleration;
  int vel_d;
  int acc_d;
  int controlSignal;
  float motorCurrent;
};

int logFileCount = 1;
LogEntry logBuffer[LOG_BUFFER_SIZE];
volatile int logBufferWriteIndex = 0;
volatile int logBufferReadIndex = 0;

const int ledPin = LED_BUILTIN; // use built in led for visual cues

const int MotorCurrent = A9; // ADC0 for current sense on motor driver (pin 8 on driver ic)

const int chipSelect1 = 0; // 0 pin for CS1; this is for the encoder; MISO-1, MOSI-26, SCK-27, gnd, 5v

const int mPWM = 3; //pwm signal for motor driver IC (pwm timer: FlexPWM4.2, pin 3 also on same timer)
const int mDIR = 2; // direction signal for motor driver IC (high or low to switch direcitons)

const int MagS = 4; //input pin for magnetic sensor to turn on jetting

const float Kp = 0.1; // Proportional Control Gain
const float Kd = 0; // Derivative Control Gain
const float Ki = 0; // Integral Control Gain

bool LastSwitchState = HIGH; //mag switch/sensor is normally open (hamlin 59140 1-U-02-A)
bool SwitchState = LOW;
int State = 0; //0-idle, 1-running
const unsigned long debounce = 500; //debounce interval (ignores switches in mag state shorter than 0.5s)
unsigned long LastSwitchTime = 0;

float currentVelocity = 0;
float currentAcceleration = 0;
int desiredVelocity = 0;
int desiredAcceleration = 0;
int controlSignal = 0;
float integralError = 0;
float lastError = 0;
unsigned long lastUpdateTime = 0;

static int velocityIndex = 0;        // ring buffer index
static bool bufferFilled = false;    // indicates if we've wrapped around
uint16_t positionBuffer[VELOCITY_HISTORY_SIZE] = {0};
unsigned long timeBuffer[VELOCITY_HISTORY_SIZE] = {0};
float velocityHistory[VELOCITY_HISTORY_SIZE] = {0}; // used to compute acceleration

Metro MainTimer(2);

// Function Declarations
void loadVelProgram();
void initializeLogFileCount();
void createNewLogFile();
uint16_t readEncoder();
float getMotorCurrent();
void updateVelocityAcceleration(uint16_t position);
void getDesiredState(uint16_t position, int &v_desired, int &a_desired);
float computePID(int v_desired, float v_measured, int a_desired, float dt);
void setMotor(int dir, int dc, int pwmPin, int dirPin);
void logData(unsigned long time, uint16_t position, float velocity, float acceleration, int v_desired, int a_desired, int controlSignal, float motorCurrent);
void flushLogToSD();

void setup() {
  pinMode(chipSelect1, OUTPUT); // set up chipselct pin mode for encoder comms
  digitalWrite(chipSelect1, HIGH); // set encoder to inactive state to prevent conflicts

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
          delay(1000);
          digitalWrite(ledPin, LOW);
          delay(1000);
      }
  }

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
        pinMode(mPWM,OUTPUT);
        digitalWrite(mPWM, LOW);
        digitalWrite(ledPin, LOW);
        if (logFile) {
          logFile.close();
        }
      }
      LastSwitchTime = millis(); //update switch time
    }
  }

  if (MainTimer.check() && State == 1) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;
    // 1) Read encoder
    uint16_t position = readEncoder();
    if (position != 0xFFFF) {// if good position proceed with control loop and logging
      // 2) Update velocity & accel via partial sums
      updateVelocityAcceleration(position);
      // 3) Grab Desired states
      getDesiredState(position, desiredVelocity, desiredAcceleration);
      // 4) PID
      controlSignal = computePID(desiredVelocity, currentVelocity, desiredAcceleration, dt);
      // 5) Set motor
      int pwmVal = (int)round(controlSignal);
      pwmVal = constrain(pwmVal, -255, 255);
      setMotor((pwmVal > 0 ? 1 : -1), abs(pwmVal), mPWM, mDIR);
      // 6) Log data
      float motorCurrent = getMotorCurrent();
      unsigned long now = millis();
      logData(now, position, currentVelocity, currentAcceleration, desiredVelocity, desiredAcceleration, controlSignal, motorCurrent);
    }
  }    

  flushLogToSD();

  LastSwitchState = SwitchState;

}

uint16_t readEncoder() {
    SPI1.beginTransaction(encoderSPISettings);
    digitalWrite(chipSelect1, LOW); // Activate CS
    delayMicroseconds(3);
    uint8_t highByte = SPI1.transfer(0x00);
    delayMicroseconds(3);
    uint8_t lowByte = SPI1.transfer(0x00);
    delayMicroseconds(3);
    digitalWrite(chipSelect1, HIGH); // Deactivate CS
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
    String fileName = "Test_Log_05Hz_256pts_OSC_" + String(logFileCount) + ".txt";
    if (!SD.sdfs.exists(fileName.c_str())) {
        break;
    }
    logFileCount++;//increment up if log file exists already
  }
}

void updateVelocityAcceleration(uint16_t position) {
  // Store current position & time in ring buffers
  positionBuffer[velocityIndex] = position;
  timeBuffer[velocityIndex] = millis();

  // Compute velocity (partial sums)
  float velocity = 0.0f;
  if (bufferFilled || velocityIndex > 0) {
    int samplesToUse = bufferFilled ? VELOCITY_HISTORY_SIZE : velocityIndex;
    if (samplesToUse > 1) {
      // Sum partial intervals
      for (int i = 0; i < samplesToUse - 1; i++) {
        int index1 = (velocityIndex + i) % VELOCITY_HISTORY_SIZE;
        int index2 = (index1 + 1) % VELOCITY_HISTORY_SIZE;

        // Wrap-corrected deltaPos
        int32_t deltaPos = positionBuffer[index2] - positionBuffer[index1];
        if (deltaPos > 2048) deltaPos -= 4096;
        else if (deltaPos < -2048) deltaPos += 4096;

        // Time difference
        unsigned long t1 = timeBuffer[index1];
        unsigned long t2 = timeBuffer[index2];
        if (t2 >= t1) {
          float dtSec = (t2 - t1) / 1000.0f;
          if (dtSec > 0) {
            velocity += (deltaPos / dtSec);
          }
        }
      }
      velocity /= (samplesToUse - 1);
    }
  }

  // Now store velocity so we can do partial-sums for acceleration
  velocityHistory[velocityIndex] = velocity;

  // Compute acceleration
  float acceleration = 0.0f;
  if (bufferFilled || velocityIndex > 0) {
    int samplesToUse = bufferFilled ? VELOCITY_HISTORY_SIZE : velocityIndex;
    if (samplesToUse > 1) {
      for (int i = 0; i < samplesToUse - 1; i++) {
        int index1 = (velocityIndex + i) % VELOCITY_HISTORY_SIZE;
        int index2 = (index1 + 1) % VELOCITY_HISTORY_SIZE;

        unsigned long t1 = timeBuffer[index1];
        unsigned long t2 = timeBuffer[index2];
        if (t2 >= t1) {
          float dtSec = (t2 - t1) / 1000.0f;
          if (dtSec > 0) {
            float deltaVel = velocityHistory[index2] - velocityHistory[index1];
            acceleration += (deltaVel / dtSec);
          }
        }
      }
      acceleration /= (samplesToUse - 1);
    }
  }

  // Update global
  currentVelocity = velocity;
  currentAcceleration = acceleration;

  // Increment ring buffer index
  velocityIndex = (velocityIndex + 1) % VELOCITY_HISTORY_SIZE;
  if (velocityIndex == 0) {
    bufferFilled = true;
  }
}

void getDesiredState(uint16_t position, int &v_desired, int &a_desired) {
    // Normalize position within the velocity program range
    int programPos = position % velProgram[velProgLength - 1][0];

    // Find the two closest points for interpolation
    int lowerIndex = 0;
    int upperIndex = 0;
    for (int i = 0; i < velProgLength - 1; i++) {
        if (velProgram[i][0] <= programPos && velProgram[i + 1][0] > programPos) {
            lowerIndex = i;
            upperIndex = i + 1;
            break;
        }
    }

    // Perform linear interpolation
    int pos1 = velProgram[lowerIndex][0];
    int vel1 = velProgram[lowerIndex][1];
    int acc1 = velProgram[lowerIndex][2];
    
    int pos2 = velProgram[upperIndex][0];
    int vel2 = velProgram[upperIndex][1];
    int acc2 = velProgram[upperIndex][2];

    float alpha = (float)(programPos - pos1) / (pos2 - pos1);

    v_desired = (vel1 + alpha * (vel2 - vel1));
    a_desired = (acc1 + alpha * (acc2 - acc1));
}

float computePID(int v_desired, float v_measured, int a_desired, float dt) {
    float error = v_desired - v_measured;
    integralError += error * dt;
    float derivativeError = (error - lastError) / dt;
    lastError = error;
    float pidOut = Kp * error + Ki * integralError + Kd * derivativeError;
    return pidOut;
}

float getMotorCurrent() {
  return analogRead(MotorCurrent); // will need conversion factor for getting actual amperage
}

void createNewLogFile() {
    String fileName = "Test_Log_05Hz_256pts_OSC_" + String(logFileCount) + ".txt";
    logFile = SD.sdfs.open(fileName.c_str(), O_WRITE | O_CREAT | O_TRUNC);// open for write, create file if doens't exist, overwrite if file exists
    if (logFile) {
        logFileCount++; // Increment the file count for the next log file
        logFile.print("Time (ms), Position (count), Velocity (count/s), Acceleration (count/s2), Desired Vel, Desired Acc, control signal, motorCurrent (raw):Kp=");
        logFile.print(Kp,2);
        logFile.print(",Kd=");
        logFile.print(Kd,2);
        logFile.print(",Ki=");
        logFile.println(Ki,2);
    }
}

void logData(unsigned long time, uint16_t position, float velocity, float acceleration, int v_desired, int a_desired, int controlSignal, float motorCurrent) {
    int nextIndex = (logBufferWriteIndex + 1) % LOG_BUFFER_SIZE;
    if (nextIndex != logBufferReadIndex) {
        logBuffer[logBufferWriteIndex] = {time, position, velocity, acceleration, v_desired, a_desired, controlSignal, motorCurrent};
        logBufferWriteIndex = nextIndex;
    }
}

void flushLogToSD() {
    while (logBufferReadIndex != logBufferWriteIndex) {
        LogEntry entry = logBuffer[logBufferReadIndex];
        logFile.print(entry.timestamp);
        logFile.print(", ");
        logFile.print(entry.position);
        logFile.print(", ");
        logFile.print(entry.velocity);
        logFile.print(", ");
        logFile.print(entry.acceleration);
        logFile.print(", ");
        logFile.print(entry.vel_d);
        logFile.print(", ");
        logFile.print(entry.acc_d);
        logFile.print(", ");
        logFile.print(entry.controlSignal);
        logFile.print(", ");
        logFile.println(entry.motorCurrent);
        logBufferReadIndex = (logBufferReadIndex + 1) % LOG_BUFFER_SIZE;
    }
}

void loadVelProgram() {
  FsFile file = SD.sdfs.open(Prog.c_str(), O_READ);
  if (file) {
    velProgLength = 0;
    while (file.available() && velProgLength < 256) {
      digitalWrite(ledPin, HIGH);
      delay(100); // Blink LED at 250ms intervals while loading
      digitalWrite(ledPin, LOW);
      delay(100);
      String line = file.readStringUntil('\n'); //read until the new line
      int separator1 = line.indexOf(',');//find first comma in string
      int separator2 = line.indexOf(',', separator1 + 1);//find second comma in string by starting after the first comma
      if (separator1 > 0 && separator2 > 0) {
        velProgram[velProgLength][0] = line.substring(0, separator1).toInt(); // Position
        velProgram[velProgLength][1] = line.substring(separator1 + 1, separator2).toInt(); // Velocity
        velProgram[velProgLength][2] = line.substring(separator2 + 1).toInt(); // Acceleration
        velProgLength++;
    }
    }
  }
  else {
    while(true) {
      digitalWrite(ledPin, HIGH);
      delay (1000);
      digitalWrite(ledPin, LOW);
      delay(1000);
    }
  }
}

void setMotor(int dir, int dc, int pwmPin, int dirPin) {
    if (dc <= 5) {
        digitalWrite(pwmPin, LOW); // Turn off the motor
    } else if (dc >= 250) {
        digitalWrite(pwmPin, HIGH); // Full duty cycle
    } else {
        analogWrite(pwmPin, dc); // Set PWM duty cycle
    }
    digitalWrite(dirPin, dir == 1 ? HIGH : LOW);
}