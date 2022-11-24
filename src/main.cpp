/*
  
  IMU Unit
  Description here

*/

//sd card dependencies
#include <SPI.h>
#include <SD.h>
//mpu6050 dependencies
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const int chipSelect = 10;

Adafruit_MPU6050 mpu;

void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(115200);
   while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  // see if SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  
  
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize MPU6050 chip
  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
  
}

void loop()
{
  if(mpu.getMotionInterruptStatus()){
    //get new sensor events with the readings
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt",FILE_WRITE);


    // if the file is available, write to it:
    if (dataFile) {
      while (dataFile.available()) {
        Serial.write(dataFile.read());
      }
      Serial.println("writing to file...");
      
      // Print out the values
      dataFile.print("AccelX:");
      dataFile.print(a.acceleration.x);
      dataFile.print(",");
      dataFile.print("AccelY:");
      dataFile.print(a.acceleration.y);
      dataFile.print(",");
      dataFile.print("AccelZ:");
      dataFile.print(a.acceleration.z);
      dataFile.print(", ");
      dataFile.print("GyroX:");
      dataFile.print(g.gyro.x);
      dataFile.print(",");
      dataFile.print("GyroY:");
      dataFile.print(g.gyro.y);
      dataFile.print(",");
      dataFile.print("GyroZ:");
      dataFile.print(g.gyro.z);
      dataFile.println("");

      dataFile.close();
    }  
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    } 
  }

  delay(10);
}
