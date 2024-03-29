#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


// ==== include statements
   #include <SD.h>

// RTC libary by Adafruit. Must be manually installed
   #include "RTClib.h" ;

// SD card pin to use
   const int SD_PIN = 9;
   File logfile;

// Real time clock type and variable
   RTC_DS1307 rtc;

File dataFile;

void setupSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G); // I'm not using this, but it is an option, you can incrase the number of G
  
  // // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS); // You can increase the number of gauss to alter sensitivity

  // // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // I'm not using this, but it is an option you can increase the number of DPS 
}

// =========================================
// initializes the RTC, 
// and checks to see if it has been set
// =========================================
void init_RTC() {
  Serial.print("Initializing RTC...");
  //
  if (!rtc.begin()) {
    Serial.println(" failed!");
    while (1);
  }
  //
  Serial.println(" done!");
  //
  if (!rtc.isrunning())
    Serial.println(
    "WARNING: RTC has not been previously set");
  }


// ======================================================
// attempts to initialize the SD card for reading/writing
// ======================================================

void init_SD() {
  Serial.print("Initializing SD card...");
  //
  delay(100);
  if (!SD.begin(SD_PIN)) {
    Serial.println(" failed!");
    while (1);
  }
  //
  Serial.println(" done!");
  }


// ====================================================
// SETUP
// ====================================================

void setup() {
   // Open serial communications
      Serial.begin(9600);
      // Serial.println("Started Setup");
   // start the real time clock
      init_RTC();
     // (note 24-hour time: 3pm -> 15)	
     // This line sets the RTC with an 
     // explicit date & time, for example: 
     // to set January 21, 2014 at 3:18pm 
     // you would use the following line: 
     // rtc.adjust(DateTime(2014, 1, 21, 15, 18, 0));
   // initialize the SD card
      init_SD();

    while (!Serial) {
      delay(1); // will pause Zero, Leonardo, etc until serial console opens
    }
    
    Serial.println("LSM9DS1 data read demo");
    
    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");
    // helper to just set the default scaling we want, see above!
    setupSensor();
    // Serial.println("Ended Setup");
    delay(1000);
  }  // End of setup


// ====================================================
// LOOP
// ====================================================
  
void loop( ) {
  // print_all_data();
  write_all_data();
  delay(1000); // time between data points
  }

void write_all_data() {
  File dataFile = SD.open("datalog10.txt", FILE_WRITE);
  write_time_data(dataFile);
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  write_mag_data(m, dataFile);
  write_accel_data(a, dataFile);
  write_gyro_data(g, dataFile);
  dataFile.println();
  // delay(50);
  dataFile.close();
  Serial.println("Data Line Written");
}
void write_time_data(File dataFile) {
  DateTime now = rtc.now();
  // String Datastring = String(now.year())+"-"+String(now.month())+"-"+String(now.day())+" "+String(now.hour())+":"+String(now.minute())+":"+String(now.second());
  String Datastring = String(now.hour())+":"+String(now.minute())+":"+String(now.second());
  dataFile.print(Datastring);
  }
void write_mag_data(sensors_event_t m, File dataFile) {
  // String Datastring = ", Mag X: "+String(m.magnetic.x)+", Y: "+String(m.magnetic.y)+", Z: "+String(m.magnetic.z);
  String Datastring = ", Mag X:"+String(m.magnetic.x)+", Y:"+String(m.magnetic.y)+", Z:"+String(m.magnetic.z);
  dataFile.print(Datastring);
  }
void write_accel_data(sensors_event_t a, File dataFile) {
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  dataFile.print(", X:");
  dataFile.print(ax);
  dataFile.print(", Y:");
  dataFile.print(ay);
  dataFile.print(", Z:");
  dataFile.print(az);
  }
void write_gyro_data(sensors_event_t g, File dataFile) {
  // String Datastring = ", Gyro X: "+String(g.gyro.x)+", Y: "+String(g.gyro.y)+", Z: "+String(g.gyro.z);
  String Datastring = ", Gyro X:"+String(g.gyro.x)+", Y:"+String(g.gyro.y)+", Z:"+String(g.gyro.z);
  dataFile.print(Datastring);
  }
//   }
