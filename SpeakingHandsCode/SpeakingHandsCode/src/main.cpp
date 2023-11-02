#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <FlexSensor.h>
#include <SD.h>
#include <SerialFlash.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// setting up training parameters
int trainingButton = 3; // assigning trainingButton to digital pin 3, change if required to any other digital pin
int trainingLed = 4;    // assigning trainingLed which glows during training mode, attached to digital pin 4
byte lastTrainingState, currentTrainingState;
const int arryWidth = 8;                           // total no of sensors
const int arryDepth = 200;                         // no of time stamps to be recorded
const int lenArryReadings = arryWidth * arryDepth; // total no of readings to be stored in flattened format for one training sample
int arryReadings[lenArryReadings];                 // initialize 1D array of sensor readings (flattened)
int minErrorIndex;                                 // index of the training sample with minimum error
int minError = 1661992959;                         // minimum error value
int threshold = 1636800 * 0.2;                     // threshold error value for playing audio file (327360)

// setting up testing parameters
int testingButton = 5; // assigning testingButton to digital pin 5, change if required to any other digital pin
int testingLed = 6;    // assigning testingLed which glows during testing mode, attached to digital pin 6
byte testingState;

// setting up led
int greenLed = 41;

// setting up audio
void playFile(const char *filename);
AudioPlaySdWav playWav1;
AudioOutputMQS audioOutput;
AudioConnection patchCord1(playWav1, 0, audioOutput, 0);
AudioConnection patchCord2(playWav1, 1, audioOutput, 1);
#define SDCARD_CS_PIN BUILTIN_SDCARD
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

// setting up gyroscope mpu6050
void mpu_init();
void mpu_updateReadings();
int mpu_getYAW(int minOutputYaw, int maxOutputYaw);
int mpu_getPITCH(int minOutputPitch, int maxOutputPitch);
int mpu_getROLL(int minOutputRoll, int maxOutputRoll);
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 gy;
VectorFloat gravity;
float euler[3];
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// setting up flex sensors
int numReadings = 10;
FlexSensor thumbF(A0, numReadings);
FlexSensor indexF(A1, numReadings);
FlexSensor middleF(A2, numReadings);
FlexSensor ringF(A3, numReadings);
FlexSensor littleF(A6, numReadings);

// setting up SD card
void sdcard_init();
void printArryReadingsToSD();
void checkReadingsFromSD(int testArryReadings[lenArryReadings]);
File myFile;

void setup()
{
  Serial.begin(9600);
  pinMode(trainingButton, INPUT_PULLDOWN);
  pinMode(trainingLed, OUTPUT);
  pinMode(testingButton, INPUT_PULLDOWN);
  pinMode(testingLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  Serial.println("***** GYROSCOPE SETUP *****");
  mpu_init();
  Serial.println("***** GYROSCOPE SETUP COMPLETE *****");
  Serial.println();

  Serial.println("***** SD CARD SETUP *****");
  sdcard_init();
  Serial.println("***** SD CARD SETUP COMPLETE *****");
  Serial.println();

  Serial.println("***** AUDIO SETUP *****");
  AudioMemory(8);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN)))
  {
    while (1)
    {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  Serial.println("***** AUDIO SETUP COMPLETE *****");
  Serial.println();

  Serial.println("***** CALIBRATION SETUP *****");
  Serial.println("---> Calibrating Thumb (5sec)");
  digitalWrite(greenLed, HIGH);
  thumbF.Calibrate();
  digitalWrite(greenLed, LOW);
  delay(500);
  Serial.println("---> Calibrating Index (5sec)");
  digitalWrite(greenLed, HIGH);
  indexF.Calibrate();
  digitalWrite(greenLed, LOW);
  delay(500);
  Serial.println("---> Calibrating Middle (5sec)");
  digitalWrite(greenLed, HIGH);
  middleF.Calibrate();
  digitalWrite(greenLed, LOW);
  delay(500);
  Serial.println("---> Calibrating Ring (5sec)");
  digitalWrite(greenLed, HIGH);
  ringF.Calibrate();
  digitalWrite(greenLed, LOW);
  delay(500);
  Serial.println("---> Calibrating Little (5sec)");
  digitalWrite(greenLed, HIGH);
  littleF.Calibrate();
  digitalWrite(greenLed, LOW);
  delay(500);
  Serial.println("Done Calibration");
  Serial.println("Thumb (Min,Max)   :   (" + String(thumbF.getMinInput()) + "," + String(thumbF.getMaxInput()) + ")");
  Serial.println("Index (Min,Max)   :   (" + String(indexF.getMinInput()) + "," + String(indexF.getMaxInput()) + ")");
  Serial.println("Middle (Min,Max)  :   (" + String(middleF.getMinInput()) + "," + String(middleF.getMaxInput()) + ")");
  Serial.println("Ring (Min,Max)    :   (" + String(ringF.getMinInput()) + "," + String(ringF.getMaxInput()) + ")");
  Serial.println("Little (Min,Max)  :   (" + String(littleF.getMinInput()) + "," + String(littleF.getMaxInput()) + ")");
  Serial.println("***** CALIBRATION SETUP COMPLETE *****");
  Serial.println();

  digitalWrite(greenLed, HIGH);
  Serial.println("Starting in 5 seconds... ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(String(i + 1) + " ");
    delay(1000);
  }
  Serial.println();
  digitalWrite(greenLed, LOW);
  
  currentTrainingState = digitalRead(trainingButton);
}

void loop()
{
  // TRAINING CODE (RUNS ONLY WHEN TRAINING BUTTON IS PRESSED/RELEASED)
  lastTrainingState = currentTrainingState;
  currentTrainingState = digitalRead(trainingButton);
  if (lastTrainingState == LOW && currentTrainingState == HIGH)
  {
    Serial.println("***** TRAINING MODE STARTED *****");
    digitalWrite(trainingLed, HIGH);
    // storing the flatten value of sensors in arryReadings Array
    for (int i = 0; i < arryDepth; i++)
    {
      mpu_updateReadings();
      arryReadings[i * arryWidth + 0] = thumbF.getSensorValue();
      arryReadings[i * arryWidth + 1] = indexF.getSensorValue();
      arryReadings[i * arryWidth + 2] = middleF.getSensorValue();
      arryReadings[i * arryWidth + 3] = ringF.getSensorValue();
      arryReadings[i * arryWidth + 4] = littleF.getSensorValue();
      arryReadings[i * arryWidth + 5] = mpu_getYAW(0, 1023);
      arryReadings[i * arryWidth + 6] = mpu_getPITCH(0, 1023);
      arryReadings[i * arryWidth + 7] = mpu_getROLL(0, 1023);
      delay(25); // total delay of 25*200 = 5000ms = 5sec
    }
    // storing flattened arryReadings Array to SD card
    printArryReadingsToSD();
    Serial.println("***** TRAINING MODE ENDED *****");
    digitalWrite(trainingLed, LOW);
    Serial.println();
  }

  // TESTING CODE (RUNS ONLY WHEN TESTING BUTTON IS PRESSED)
  testingState = digitalRead(testingButton);
  if (testingState == HIGH)
  {
    Serial.println("***** TESTING MODE STARTED *****");
    delay(5000);
    digitalWrite(testingLed, HIGH);
    for (int i = 0; i < arryDepth; i++)
    {
      mpu_updateReadings();
      arryReadings[i * arryWidth + 0] = thumbF.getSensorValue();
      arryReadings[i * arryWidth + 1] = indexF.getSensorValue();
      arryReadings[i * arryWidth + 2] = middleF.getSensorValue();
      arryReadings[i * arryWidth + 3] = ringF.getSensorValue();
      arryReadings[i * arryWidth + 4] = littleF.getSensorValue();
      arryReadings[i * arryWidth + 5] = mpu_getYAW(0, 1023);
      arryReadings[i * arryWidth + 6] = mpu_getPITCH(0, 1023);
      arryReadings[i * arryWidth + 7] = mpu_getROLL(0, 1023);
      delay(25); // total delay of 25*200 = 5000ms = 5sec
    }
    digitalWrite(testingLed, LOW);
    // comparing the arryReadings Array with the training data Arrays stored in SD card and updating the minError and minErrorIndex
    checkReadingsFromSD(arryReadings);
    Serial.println("Minimum Error value is : " + String(minError) + " at line Index : " + String(minErrorIndex) + " in SD card");
    // playing the audio file corresponding to the minimum error readingsArray in SD card (if error < threshold)
    // if (abs(minError) < threshold)
    if (1)
    {
      String playFileName = "AUDIO" + String(minErrorIndex) + ".WAV";
      playFile(playFileName.c_str());
      delay(500);
    }
    Serial.println("***** TESTING MODE ENDED *****");
  }
}

void playFile(const char *filename)
{
  Serial.println();
  Serial.print("Playing file: ");
  Serial.print(filename);
  playWav1.play(filename);
  delay(25);
  Serial.printf("  %d ms\n", playWav1.lengthMillis());
  uint32_t ms = millis();
  while (playWav1.isPlaying())
  {
    if (millis() - ms > 1000)
    {
      Serial.print(".");
      ms = millis();
    }
  }
}

void mpu_init()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
}

void mpu_updateReadings()
{
  if (!dmpReady)
    return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

int mpu_getYAW(int minOutputYaw = 0, int maxOutputYaw = 1023)
{
  int yawVal = (int)((ypr[0] * 180 / M_PI) + 180);
  int mapYaw = yawVal * (maxOutputYaw - minOutputYaw) / 360 + minOutputYaw;
  return minOutputYaw <= maxOutputYaw ? constrain(mapYaw, minOutputYaw, maxOutputYaw) : constrain(mapYaw, maxOutputYaw, minOutputYaw);
}

int mpu_getPITCH(int minOutputPitch = 0, int maxOutputPitch = 1023)
{
  int pitchVal = (int)((ypr[1] * 180 / M_PI) + 180);
  int mapPitch = pitchVal * (maxOutputPitch - minOutputPitch) / 360 + minOutputPitch;
  return minOutputPitch <= maxOutputPitch ? constrain(mapPitch, minOutputPitch, maxOutputPitch) : constrain(mapPitch, maxOutputPitch, minOutputPitch);
}

int mpu_getROLL(int minOutputRoll = 0, int maxOutputRoll = 1023)
{
  int rollVal = (int)((ypr[2] * 180 / M_PI) + 180);
  int mapRoll = rollVal * (maxOutputRoll - minOutputRoll) / 360 + minOutputRoll;
  return minOutputRoll <= maxOutputRoll ? constrain(mapRoll, minOutputRoll, maxOutputRoll) : constrain(mapRoll, maxOutputRoll, minOutputRoll);
}

void sdcard_init()
{
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  Serial.print("Creating file...");
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile)
  {
    Serial.println("file created.");
  }
  else
  {
    Serial.println("error creating file!");
  }
  myFile.close();
}

void printArryReadingsToSD()
{
  String readings = "";
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile)
  {
    for (int i = 0; i < lenArryReadings; i++)
    {
      readings += String(arryReadings[i]);
      if (i != lenArryReadings - 1)
      {
        readings += ",";
      }
    }
    myFile.println(readings);
    myFile.close();
  }
  else
  {
    Serial.println("error opening file!");
  }
}

void checkReadingsFromSD(int testArryReadings[lenArryReadings])
{
  int j = 0;
  int maxTrainingRecords = 17; // considering 6 training sample records as max
  long long errorVec[maxTrainingRecords];
  for (int v = 0; v < maxTrainingRecords; v++)
  {
    errorVec[v] = INT64_MAX;
  }
  myFile = SD.open("data.txt");
  if (myFile)
  {
    while (myFile.available())
    {
      String reading = myFile.readStringUntil('\n');
      int strLen = reading.length() + 1;
      char charArray[strLen];
      reading.toCharArray(charArray, strLen);
      int readingsVec[lenArryReadings];
      char *ptr = NULL;
      byte index = 0;
      ptr = strtok(charArray, ",");
      while (ptr != NULL)
      {
        readingsVec[index] = atoi(ptr);
        index++;
        ptr = strtok(NULL, ",");
      }
      long long error = 0;
      for (int k = 0; k < lenArryReadings; k++)
      {
        long long a1 = sq(readingsVec[k] - testArryReadings[k]);
        error += a1;
      }
      errorVec[j] = (error);
      j++;
    }
    myFile.close();
  }
  else
  {
    Serial.println("error opening file!");
  }
  int minIndex = 0;
  for (int m = 1; m < maxTrainingRecords; m++)
  {
    if (errorVec[m] < errorVec[minIndex])
    {
      minIndex = m;
    }
  }
  minErrorIndex = minIndex;
  minError = errorVec[minIndex];
}
