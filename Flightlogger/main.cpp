/*
  Flight computer

  -Initialization and test of Accelerometer, Barometric presure sensor, and SD card reader.
  -Reading from each sensor and displaying to console
*/
#include <string>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_BMP3XX.h"
#include <SoftwareSerial.h>

SoftwareSerial GPS(A2, A3);

#define SEALEVELPRESSURE_HPA (1013.25)
#define GPSECHO true

Adafruit_LSM6DSOX sox; 
Adafruit_BMP3XX bmp;
uint32_t timer = millis();
String testNMEA;
String NMEA1;
String PGGA;
String tgga;
String latgga;
String longgga;
String altggas;
int altgga;
String PRMC;
String trmc;
String latrmc;
String longrmc;
String speedrmc;
String date;
File myFile;
String xacc;
String yacc;
String zacc;
String xgyr;
String ygyr;
String zgyr;

int ledPin = LED_BLUE; 
const int SDpin = 11;
char filename[15];
char c;

void setup() {
  Serial.begin(115200); // Open serial communications baud rate 115200
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
  if (!GPS.available()) {
    leadString += "\t GPS not available (**Ignore following line**) \n";
  }
  leadString += " \t GPS Online! \n";
  GPS.write("$PGCMD,33,0*6D");
  GPS.write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  // GPS.write(PMTK_SET_BAUD_115200);
  // GPS.write(PMTK_AWAKE);
  delay(500);

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
  Serial.println("Gyro data rate set to: ");
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
  myFile.println("__________________________________________ F l i g h t  -  D a t a __________________________________________");
  myFile.println("   time \t GPS Alt \t Bar Alt \t z-acc \t y-acc \t x-acc \t  heading  \t Avg. temp \t Lat \t Long");
  myFile.println("   UTC \t \t   (m) \t \t  (m) \t \t (m/s) \t (m/s) \t (m/s) \t  (deg)  \t   (C) \t  \t ");
  myFile.println("--------------------------------------------------------------------------------------------------------- ");
  myFile.close();
  Serial.println("done.");
}

void loop () {
  myFile = SD.open(filename, FILE_WRITE);
  readsox();
  if (GPS.available()){
    c = GPS.read();
    while (c!=36) {
      if (GPS.available()){
        c = GPS.read();
        NMEA1 += c;
      }
    }  NMEAParse();
  }
  String out = "";
  String bmpalt = String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  out += tgga + "\t" + altggas + "\t" + bmpalt + "\t" + zacc  + "\t"  + yacc  + "\t" + xacc  + "\t" + zgyr + " " + ygyr + " " + xgyr + " "  + "\t" + bmp.temperature   + "\t" + latgga + "  "  + longgga; 
  myFile.println(out);
  myFile.close();

}

void NMEAParse (){
  int des = 0;
  int j[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  if (NMEA1.startsWith("PGGA") || NMEA1.startsWith("$PGGA") || NMEA1.startsWith("GGA") ){
    des --;
    int com = 0;
    int l = NMEA1.length();
    int jc = 0;
   for (int i = 1; i < l; i++){
      if (NMEA1.charAt(i) == ',') {
        jc++;
        j[jc] = i;
      }
    }
  } if (NMEA1.startsWith("PRMC") || NMEA1.startsWith("$PRMC") || NMEA1.startsWith("RMC") ){
      des ++;
      int com = 0;
      int l = NMEA1.length();
      int jc = 0;
      for (int i = 1; i < l; i++){
       if (NMEA1.charAt(i) == ',') {
         jc++;
         j[jc] = i;
        }
      }
  } if (des < 0) {
      tgga = NMEA1.substring(j[1]+1, j[2]);
      latgga = NMEA1.substring(j[2]+1,j[4]); 
      longgga = NMEA1.substring(j[4]+1,j[6]);
      altggas = NMEA1.substring(j[9]+1,j[10]); 
      altgga = altggas.toInt();
  } if (des >0) {
      trmc = NMEA1.substring(j[1]+1, j[2]);
      latrmc = NMEA1.substring(j[3]+1,j[5]); 
      longrmc = NMEA1.substring(j[5]+1,j[7]);
      speedrmc = NMEA1.substring(j[7]+1,j[8]);
      date = NMEA1.substring(j[9]+1,j[10]);
    }
   NMEA1 = "";
}

void readsox(){
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);
  xacc = String(accel.acceleration.x);
  yacc = String(accel.acceleration.y);
  zacc = String(accel.acceleration.z);
  xgyr = String(gyro.gyro.x);
  ygyr = String(gyro.gyro.y);
  zgyr = String(gyro.gyro.z);
}

String buildstring(String in){
  String out = "";
  return out;
}