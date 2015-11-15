//<<Pin Definitions>>
//<Digital Pins>
//Pin 0 and 1 for RX and TX for Serial communication
//Pin 2 for INT
#define RedLED 3
#define SDCS 4
#define GreenLED 5
#define Switch 7
//Pin 11 through 13 for communication with SPI bus
//Pin 11 MOSI
//Pin 12 MISO
//Pin 13 CLK

//<Analog Pins>
//Pin 4 for SDA(data line)
//Pin 5 for SCL(clock line)

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SPI.h>
#include <SD.h>

File sensorLogFile;

// class default I2C address is 0x68
MPU6050 mpu;
//#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool noErrors = true;
long startTime;
unsigned long currentTime;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    startTime = millis();

    pinMode(RedLED, OUTPUT);
    pinMode(GreenLED, OUTPUT);
    pinMode(Switch, INPUT);

    if ( !digitalRead(Switch) ) switchedOff();
    LEDs(1);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 10;
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    //initialize SD card
    Serial.print(F("Initializing SD card..."));
    if (!SD.begin(SDCS)) {
        Serial.println(F("SD card initialization failed!"));
        LEDs(0);
        return;
    }
    Serial.println(F("SD card initialization done."));

    if ( !openFile() ) {
      return;
    }

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        LEDs(0);
        return;
    }
}

void loop() {
    if ( !digitalRead(Switch) ) switchedOff();

    //if setup had some errors, don't do anything
    if (!noErrors) return;
    LEDs(1);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte and current FIFO count
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        currentTime = millis() - startTime;
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display real acceleration, adjusted to remove gravity
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        String dataString = String(currentTime) + ", " + String(aaWorld.x/8192.0) + ", " + String(aaWorld.y/8192.0) + ", " + String(aaWorld.z/8192.0);

        //write the sensor data to the opened file
        Serial.print(F("Writing to file..."));
            sensorLogFile.println(dataString);
            Serial.println(dataString);
    }
}

void LEDs(bool state)
{
  noErrors = state;
  digitalWrite(RedLED, !state);
  digitalWrite(GreenLED, state);
}

void switchedOff()
{
  //if log file is open, then close it
  if (sensorLogFile) { sensorLogFile.close(); }

  digitalWrite(GreenLED, LOW);
  while ( !digitalRead(Switch) ) {
    //blink the Red LED
    digitalWrite(RedLED, HIGH);
    delay(1000);
    digitalWrite(RedLED, LOW);
    delay(1000);
  }
  return;
}

bool openFile()
{
  //open a file to write
  sensorLogFile = SD.open("noclose.csv", FILE_WRITE);
  if ( sensorLogFile ) {
    sensorLogFile.println(", , , ,"); //Just a leading blank line, incase there was previous data
    sensorLogFile.println("t, X, Y, Z");
    return 1;
  } else {
    Serial.println(F("Error opening file"));
    LEDs(0);
    return 0;
  }
}

