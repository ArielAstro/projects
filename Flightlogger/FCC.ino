/*
  Flight computer

  -Initialization and test of Accelerometer, Barometric presure sensor, and SD card reader.
  -Reading from each sensor and displaying to console
  TO-DO
  Format readings to format set on Initialization
  Integrate GPS

*/
#include <string>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_GPS.h"


#define SEALEVELPRESSURE_HPA (1013.25)
#define GPSSerial Serial
#define GPSECHO true

Adafruit_GPS GPS (&GPSSerial);
Adafruit_LSM6DSOX sox; 
Adafruit_BMP3XX bmp;
uint32_t timer = millis();
String testNMEA;
String NMEA1 = "zeeeero";
String NMEA2;
File myFile;

int ledPin = LED_BLUE; 
const int SDpin = 11;
char filename[15];
char c;

void setup() {
  Serial.begin(9600); // Open serial communications baud rate 115200
  String leadString ="*-*-*-*-*-*-*-*-*-* System test *-*-*-*-*-*-*-*-*-*-* \n";

// SD Card
  leadString += "\t Initializing SD card... \n";
  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed! (Ignore following line)");
    leadString += "\t SD Unavailable. (**Ignore following line**) \n";
  }
  Serial.println("SD Card Online");
  leadString += "\t SD Card Online!\n";

  strcpy(filename, "/fdata00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

// GPS 
  GPS.begin(9600);  // initalize GPS at default baud rate
  if (!GPS.begin(9600)) {
    leadString += "\t GPS not available (**Ignore following line**) \n";
  }
  leadString += " \t GPS Online! \n";
  GPSSerial.println(PMTK_Q_RELEASE);
  GPS.sendCommand("$PGCMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_BAUD_115200);
  GPS.sendCommand(PMTK_AWAKE);
  delay(1000);

// Accelerometer
   if (!sox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX");
    leadString += "\t Accelerometer not available (**Ignore following line**) \n";
  }
  Serial.println("LSM6DSOX Online!");
  leadString += "\t Accelerometer Online! \n";
  Serial.print("Accelerometer range set to: ");
  leadString += "\t \t Accelerometer set to: ";
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    leadString += "+- 2G\t";
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    leadString += "+-4G \t";
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
       leadString += "+-8G \t";
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    leadString += "+-16G \t";
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  leadString += "Gyro range set to: ";
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    leadString += "125 degrees/s";
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    leadString += "250 degrees/s";
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    leadString += "500 degrees/s";
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    leadString += "1000 degrees/s";
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    leadString += "2000 degrees/s";
    break;
  case ISM330DHCT_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  
// Barometric Pressure Sensor
  if (!bmp.begin()) {
      Serial.println("Failed to find BMP sensor");
      leadString += "\n \t Failed to find BMP sensor (**Ignore following line**) \n";
  }
  Serial.println("BMP Online");
  leadString += "\t BMP Online! \n";

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  myFile = SD.open(filename, FILE_WRITE);
  if( !myFile ) {
    Serial.print("error opening "); 
    Serial.println(filename);
    while(1);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.println("Ready!");
  
  uint8_t i=0;

  Serial.print("Writing header to ");
  Serial.println(filename);
  myFile.println(leadString);
  myFile.println("");
  myFile.println("________________________________________ F l i g h t  -  D a t a ________________________________________");
  myFile.println("time \t Avg. Altitude (m) \t Bar Alt(m) \t GPS Alt(m) \t Avg. temp(C) \t Latitude \t Longitude \t x-accel (m/s) \t y-accel(m/s) \t z-accel(m/s) \t heading ");
  myFile.close();
  Serial.println("done.");
}

void loop () {
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println(NMEA1);
  myFile.println("in between NMEAs");
  myFile.print(GPSSerial.read());
  myFile.print(GPSSerial.read());
  myFile.println(GPSSerial.read());
  myFile.println(NMEA2);
  myFile.println("Before if()");
  c = GPS.read();
  if (c) {
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println("cs");
    myFile.print(c);
  }  
  myFile.close();
  
  /*myFile.close();
  readGPS();
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println(NMEA1);
  myFile.println("in between NMEAs");
  myFile.println(NMEA2);
  myFile.println("After readGPS()");
  myFile.close();
  
  if(GPS.fix==1){
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println(NMEA1);
    myFile.println("in between NMEAs");
    myFile.println(Serial.readString());
    myFile.println(NMEA2);
    myFile.close();
  } else if (GPS.fix==0){
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println("No fix everything is fine yah Turkey!");
    myFile.println(Serial.read());
    myFile.close();
  }*/
 }

 void readGPS() {
   NMEA1 = "before while in read GPS";
   myFile = SD.open(filename, FILE_WRITE);
   myFile.println(NMEA1);
   GPS.lastSentence;
   myFile.println(NMEA2);
   clearGPS();
   while(!GPS.newNMEAreceived()){
     c = GPS.read();
   }
   GPS.parse(GPS.lastNMEA());
   NMEA1 = GPS.lastNMEA();

   while(!GPS.newNMEAreceived()){
     c = GPS.read();
   }
   GPS.parse(GPS.lastNMEA());
   NMEA2 = GPS.lastNMEA();

  NMEA1 = "after while in readGPS()";
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println(NMEA1);
    myFile.println(NMEA2);
 }

 void clearGPS(){
  NMEA1 = "before while in clearGPS()";
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println(NMEA1);
    myFile.println(NMEA2);
   while(!GPS.newNMEAreceived()){
     c = GPS.read();
   }
   GPS.parse(GPS.lastNMEA());
   while(!GPS.newNMEAreceived()){
     c = GPS.read();
   }
   GPS.parse(GPS.lastNMEA());
      while(!GPS.newNMEAreceived()){
     c = GPS.read();
   }
   GPS.parse(GPS.lastNMEA());
  NMEA1 = "after while in clearGPS()";
    myFile = SD.open(filename, FILE_WRITE);
    myFile.println(NMEA1);
    myFile.println(NMEA2);
 }
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 /*
 
 
  String dataPrint = "";
  myFile = SD.open(filename, FILE_WRITE);
  
  /////////
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  if (c) myFile.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  GPS.wakeup();
  myFile.println(GPS.lastNMEA());
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    myFile.println(String(GPS.lastNMEA())); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 200) {
    timer = millis(); // reset the timer
    myFile.print("\nTime: ");
    if (GPS.hour < 10) { myFile.print('0'); }
    myFile.print(String(GPS.hour, DEC)); myFile.print(':');
    if (GPS.minute < 10) { myFile.print('0'); }
    myFile.print(GPS.minute, DEC); myFile.print(':');
    if (GPS.seconds < 10) { myFile.print('0'); }
    myFile.print(GPS.seconds, DEC); myFile.print('.');
    if (GPS.milliseconds < 10) {
      myFile.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      myFile.print("0");
    }
    myFile.println(GPS.milliseconds);
    myFile.print("Date: ");
    myFile.print(GPS.day, DEC); Serial.print('/');
    myFile.print(GPS.month, DEC); Serial.print("/20");
    myFile.println(GPS.year, DEC);
    myFile.print("Fix: "); Serial.print((int)GPS.fix);
    myFile.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      myFile.print("Location: ");
      myFile.print(GPS.latitude, 4); Serial.print(GPS.lat);
      myFile.print(", ");
      myFile.print(GPS.longitude, 4); Serial.println(GPS.lon);
      myFile.print("Speed (knots): "); Serial.println(GPS.speed);
      myFile.print("Angle: "); Serial.println(GPS.angle);
      myFile.print("Altitude: "); Serial.println(GPS.altitude);
      myFile.print("Satellites: "); Serial.println((int)GPS.satellites);
    }else if (!GPS.fix){
       myFile.print("NF Location: ");
      myFile.print(GPS.latitude, 4); Serial.print(GPS.lat);
      myFile.print("NF , ");
      myFile.print(GPS.longitude, 4); Serial.println(GPS.lon);
      myFile.print("NF Speed (knots): "); Serial.println(GPS.speed);
      myFile.print("NF Angle: "); Serial.println(GPS.angle);
      myFile.print("NF Altitude: "); Serial.println(GPS.altitude);
      myFile.print("NF Satellites: "); Serial.println((int)GPS.satellites);
    }
  //////

  // myFile.println(testNMEA);
  // myFile.println(NMEA1);
    }
  } myFile.close();
}

 void readGPS(){
  
}


void loop() {
  // put your main code here, to run repeatedly:

  // make a string for assembling the data to log:
  String dataString = "";
  myFile = SD.open("fdata.txt", FILE_WRITE);

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  // Display the results (rotation is measured in rad/s)
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);

  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();

  // if the file is available, write to it:
  if (myFile) {
    myFile.println(dataString);
    myFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening fdata.txt");
  }
}
*/
