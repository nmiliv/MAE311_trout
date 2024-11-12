/*

Description:
  -State 0: 
            Do nothing, stay silent, Sleep
  -State 1: 
            View Data on LCD
  -State 2: 
            Record Data on SD Card

  -If the button is pressed for longer than 1 seconds 'cancel' the change in state

*/

//////////////////////////// IMPORT LIBRARIES ////////////////////////////////////////

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_GPS.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <LiquidCrystal.h>
#include <math.h>

//////////////////////////// SENSOR SETUP ////////////////////////////////////////

#define BNO055_SAMPLERATE_DELAY_MS (100) // delay between samples
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire); // create sensor object and connect i2c address

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

#define GPSECHO false
Adafruit_GPS GPS(&Wire);

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

File myFile;

//////////////////////////// INITIALIZE PUBLIC VARIABLES  /////////////////////////////
const int rs = 17, en = 16, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int switchPin1 = 14;                     // switch monitors pin 14
const int switchPin2 = 15;
const int Red = 6;                            // the Red led is on pin 6
const int Green = 7;                          // the green led is on pin 7
const int Blue = 8;                           // the blue led is on pin 8

int buttonState1;                              // check the state of the button
int buttonState2;
int State = 0;                                // stores state of system
int subState = 0;
int caliState = 0;
                                                  // 0 = state0
                                                  // 1 = state1
                                                  // 2 = state2

unsigned long buttonPrevious = 0; // [ms]     // store the time when the button was initially pressed
const int buttonInterval = 1000; // [ms]      // interval for button being pressed to 'cancel' the change in state
unsigned long buttonDiff = 0; // [ms]         // store the difference in time between button press and release

unsigned long startState0 = 0; // [ms]        // Initialize state timers
unsigned long startState1 = 0; // [ms]
unsigned long startState2 = 0; // [ms]

double BMP_temp = 0; // [C]                   // Initialize BMP variables
double BMP_pres = 0 / 100.0; // [hPa]
float BMP_alt = 0; // [m] above specified sealevel

double BNO_X = 0; // [m]                      // Initialize BNO variables
double BNO_Y = 0; // [m]
double BNO_Z = 0; // [m]

double BNO_X_accl = 0;
double BNO_Y_accl = 0;
double BNO_Z_accl = 0;

double BNO_X_gyro = 0;
double BNO_Y_gyro = 0;
double BNO_Z_gyro = 0;

double BNO_X_magn = 0;
double BNO_Y_magn = 0;
double BNO_Z_magn = 0;

uint32_t timer = millis(); // [ms]            // initialize GPS timer

double GPS_latitude = 0; // [deg.min]         // initialize GPS variables
char GPS_lat = 'X'; // [N / S]
double GPS_longitude = 0; // [deg.min]
char GPS_lon = 'X'; // [E / W]
double GPS_speed = 0; // [knotts] speed over ground
double GPS_angle = 0; // [deg] from true North
double GPS_alt = 0; // [m] above MSL
int GPS_sats = 0; // [#] of GPS satellites
bool fix = 0; // [TRUE / FALSE] does GPS have a fix

double ICM_temp1 = 0; // [degC]                // initialize ICM variables
double ICM_accel_x = 0; // [m/s]
double ICM_accel_y = 0; // [m/s]
double ICM_accel_z = 0; // [m/s]
double ICM_gyro_x = 0; // [rad/s]
double ICM_gyro_y = 0; // [rad/s]
double ICM_gyro_z = 0; // [rad/s]
double ICM_mag_x = 0; // [uT]
double ICM_mag_y = 0; // [uT]
double ICM_mag_z = 0; // [uT]

bool LCD = FALSE;
bool serial = TRUE;
bool save = FALSE;
bool headerStateSerial = FALSE;
bool headerState = FALSE;
String header = "time [ms], "
                "BMP temp [degC], BMP pres [mbar], BMP alt [m], "
                "BNO x [deg], BNO y [deg], BNO z [deg], "
                "BNO xa [m/s^2], BNO ya [], BNO za [], "
                "BNO xg [rad/s], BNO yg [], BNO zg [], "
                "BNO xm [uT], BNO ym [], BNO zm [], "
                "GPS lat [deg.min], GPS lon [deg.min], GPS speed [m/s], GPS angle [deg], GPS alt [m], GPS sat [#]"
                "ICM temp [degC], ICM accel x [m/s^2], ICM accel y [m/s^2], ICM accel z [m/s^2], "
                "ICM gyro x [rad/s], ICM gyro y [rad/s], ICM gyro z [rad/s], "
                "ICM mag x [], ICM mag y [], ICM mag z []";
elapsedMillis timestamp = 0;

void setup() {
  pinMode(switchPin1, INPUT);                  // set the switch pin as the input 
  pinMode(switchPin2, INPUT);                  // set the switch pin as the input 
  pinMode(Red, OUTPUT);                        // set the led pin as the output
  pinMode(Green, OUTPUT);                      // set the led pin as the output
  pinMode(Blue, OUTPUT);                       // set the led pin as the output

  if(serial){
      Serial.begin(115200);                        // set up the serial communication
      delay(500);
  }
  buttonState1 = digitalRead(switchPin1);      // read the initial button state
  
//////////////////////////// INITIALIZE SENSORS  ////////////////////////////////////
  if(!bno.begin()){
    //Serial.print("Could not find a valid BNO055 sensor, check wiring!");
    while(1);
  }
  if(!bmp.begin_I2C()) {
    //Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // if(!GPS.begin(0x10)){
  //   //Serial.println("Could not find a valid GPS sensor, check wiring!");
  //   while(1);
  // }
  if (!icm.begin_I2C()) {
    //Serial.println("Could not find a valid ICM sensor, check wiring!");
    while(1);
  }
  if (!SD.begin(10)) {
    //Serial.println("Could not find a valid SD writer, check wiring!");
    while (1);
  }
  
  //Serial.println("initialization done; all sensors detected"); // All systems are a-go!

  bno.setExtCrystalUse(true);

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // include both minimum and fix data and altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // set update rate of GPS
  GPS.sendCommand(PGCMD_ANTENNA);                 // turn on antenna status updates

  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  // //  icm.setAccelRateDivisor(4095);
  // uint16_t accel_divisor = icm.getAccelRateDivisor();
  // float accel_rate = 1125 / (1.0 + accel_divisor);
  // //  icm.setGyroRateDivisor(255);
  // uint8_t gyro_divisor = icm.getGyroRateDivisor();
  // float gyro_rate = 1100 / (1.0 + gyro_divisor);

  delay(1000);                                    // wait 1 second before executing loop function
  //Serial.println();
  //Serial.println("System start up, State 0");

  lcd.begin(16, 2);                            // set up the LCD's number of columns and rows

  //getCalibration();                           // load in the calibration values from the SD card (correction coeff. and offset)
}

void loop() {
  int val = digitalRead(switchPin1);           // read the input of the switch (1/0)
  delay(10);                                  // wait 10 ms to read switch state again
  int val2 = digitalRead(switchPin1);          // read the input of the switch (1/0)

  if (val == val2) {                          // compare the 2 readings for bounce
    buttonDuration(val);                      // check the button press duration
    if (val != buttonState1 && buttonDiff < buttonInterval) { // check if the state has changed AND if the button has been pressed longer than the interval value
      if (val == LOW) {                       // check if the button is being pressed
        if (State == 0) {                     // check if previously in state 0
          State = 1;                          // change the state to 1
          //Serial.println("Button just pressed, State 1");
        }else if (State == 1){                // check if previously in state 1 
          State = 2;                          // change to 2
          //Serial.println("Button just pressed, State 2");
          timestamp = 0;
        }else{                                // check if reviously in state 2 
          State = 0;                          // change state to 0
          //Serial.println("Button just pressed, State 0");
        }
      }
    }
  }
  //////////////////////////// EXECUTE STATE FUNCTIONS  /////////////////////////
  State0();                                   // Mode 0: do not record any sensor data, stay silent and sleep
  State1();                                   // Mode 1: read data from all sensors and display to LCD
  State2();                                   // Mode 2: Log Data to SD Card

  //printSerial(serial);                        // print data to the serial monitor
  //printSerial(0);
  printSerial(serial);
  saveSD(save);
  printLCD(LCD);

  buttonState1 = val;                          // update the button state
}

long buttonDuration(int val){                 // checking button press duration
  if(val != 1){                               // if the button has been pressed and has passed the debounce filter
    buttonPrevious = millis();                // if the button has NOT been pressed keep the previous button time updated
  }else{
    buttonDiff = millis() - buttonPrevious;   // if the button HAS been pressed log the duration of the press
  }
  return buttonDiff;
}

void State0(){                                // execute state 0
  if(State != 0){
    startState0 = millis();
  }else if(State == 0){
    analogWrite(Red, 255);
    analogWrite(Green, 0);
    analogWrite(Blue, 0);
    
    save = FALSE;
    serial = FALSE;
    LCD = FALSE;

    lcd.clear();
    lcd.print("MODE: SLEEP");
  }
}

void State1(){                                // execute state 1
  if(State != 1){
    startState1 = millis();
  }else if (State == 1) {                  
    analogWrite(Red, 0);
    analogWrite(Green, 255);
    analogWrite(Blue, 0);

    save = FALSE;
    serial = TRUE;
    LCD = TRUE;

    // BMP readings
    bmp.performReading();
    BMP_temp = bmp.temperature; // [C]
    BMP_pres = bmp.pressure / 100.0; // [hPa]
    BMP_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); // [m] above defined sea level

    // BNO readings
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    uint8_t calBNO_system, calBNO_gyro, calBNO_accel, calBNO_mag = 0;
    bno.getCalibration(&calBNO_system, &calBNO_gyro, &calBNO_accel, &calBNO_mag);
    BNO_X = euler.x(); // [deg]                  // angle about x
    BNO_Y = euler.y(); // [deg]                  // angle about y
    BNO_Z = euler.z(); // [deg]                  // angle about z
    
    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    BNO_X_accl = accl.x();
    BNO_Y_accl = accl.y();
    BNO_Z_accl = accl.z();
    BNO_X_gyro = gyro.x();
    BNO_Y_gyro = gyro.y();
    BNO_Z_gyro = gyro.z();
    BNO_X_magn = magn.x();
    BNO_Y_magn = magn.y();
    BNO_Z_magn = magn.z();

    // GPS readings
    GPS.read();
    if(GPS.newNMEAreceived()){
      if(!GPS.parse(GPS.lastNMEA())){ // sets the newNMEAreceived() flag to false
        return;                       // wait for another response if parsing fails
      }
    }
    if(millis()-timer>2000){          // only update data every 2 seconds to minimize junk readings
      timer = millis();               // update timer
      fix = GPS.fix;             // check if GPS has a fix
      if(fix){                        // Only get data if GPS has a fix
        GPS_latitude = GPS.latitude; // [deg.min]
        GPS_lat = GPS.lat; // [N / S]
        GPS_longitude = GPS.longitude; // [deg.min]
        GPS_lon = GPS.lon; // [E / W]
        GPS_speed = GPS.speed; // [knotts] speed over ground
        GPS_angle = GPS.angle; // [deg] from true North
        GPS_alt = GPS.altitude; // [m] above MSL
        GPS_sats = GPS.satellites; // [#] number of satellites found
      }else{
        // What to do with data if GPS loses fix
        GPS_latitude = 0; // [deg.min]
        GPS_lat = 'X'; // [N / S]
        GPS_longitude = 0; // [deg.min]
        GPS_lon = 'X'; // [E / W]
        GPS_speed = 0; // [knotts] speed over ground
        GPS_angle = 0; // [deg] from true North
        GPS_alt = 0; // [m] above MSL
        GPS_sats = 0; // [#] number of satellites found
      }
    }

    // ICM readings
    sensors_event_t ICM_accel;
    sensors_event_t ICM_gyro;
    sensors_event_t ICM_mag;
    sensors_event_t ICM_temp;
    icm.getEvent(&ICM_accel, &ICM_gyro, &ICM_temp, &ICM_mag);
    ICM_temp1 = ICM_temp.temperature; // [degC]
    ICM_accel_x = ICM_accel.acceleration.x; // [m/s]
    ICM_accel_y = ICM_accel.acceleration.y; // [m/s]
    ICM_accel_z = ICM_accel.acceleration.z; // [m/s]
    ICM_gyro_x = ICM_gyro.gyro.x; // [rad/s]
    ICM_gyro_y = ICM_gyro.gyro.y; // [rad/s]
    ICM_gyro_z = ICM_gyro.gyro.z; // [rad/s]
    ICM_mag_x = ICM_mag.magnetic.x; // [uT]
    ICM_mag_y = ICM_mag.magnetic.y; // [uT]
    ICM_mag_z = ICM_mag.magnetic.z; // [uT]
    
  }
}

void State2(){                                // execute state 2
  if(State != 2){
    startState2 = millis();
  }else if (State == 2) {
    analogWrite(Red, 0);
    analogWrite(Green, 0);
    analogWrite(Blue, 255);
    lcd.clear();
    lcd.print("MODE: LOGGING...");

    save = TRUE;
    serial = TRUE;
    LCD = FALSE;
    
    // BMP readings
    bmp.performReading();
    BMP_temp = bmp.temperature; // [C]
    BMP_pres = bmp.pressure / 100.0; // [hPa]
    BMP_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); // [m] above defined sea level

    // BNO readings
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    uint8_t calBNO_system, calBNO_gyro, calBNO_accel, calBNO_mag = 0;
    bno.getCalibration(&calBNO_system, &calBNO_gyro, &calBNO_accel, &calBNO_mag);
    BNO_X = euler.x(); // [deg]                  // angle about x
    BNO_Y = euler.y(); // [deg]                  // angle about y
    BNO_Z = euler.z(); // [deg]                  // angle about z

    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> magn = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    BNO_X_accl = accl.x();
    BNO_Y_accl = accl.y();
    BNO_Z_accl = accl.z();
    BNO_X_gyro = gyro.x();
    BNO_Y_gyro = gyro.y();
    BNO_Z_gyro = gyro.z();
    BNO_X_magn = magn.x();
    BNO_Y_magn = magn.y();
    BNO_Z_magn = magn.z();

    // GPS readings
    GPS.read();
    if(GPS.newNMEAreceived()){
      if(!GPS.parse(GPS.lastNMEA())){ // sets the newNMEAreceived() flag to false
        return;                       // wait for another response if parsing fails
      }
    }
    if(millis()-timer>2000){          // only update data every 2 seconds to minimize junk readings
      timer = millis();               // update timer
      fix = GPS.fix;             // check if GPS has a fix
      if(fix){                        // Only get data if GPS has a fix
        GPS_latitude = GPS.latitude; // [deg.min]
        GPS_lat = GPS.lat; // [N / S]
        GPS_longitude = GPS.longitude; // [deg.min]
        GPS_lon = GPS.lon; // [E / W]
        GPS_speed = GPS.speed; // [knotts] speed over ground
        GPS_angle = GPS.angle; // [deg] from true North
        GPS_alt = GPS.altitude; // [m] above MSL
        GPS_sats = GPS.satellites; // [#] number of satellites found
      }else{
        // What to do with data if GPS loses fix
        GPS_latitude = 0; // [deg.min]
        GPS_lat = 'X'; // [N / S]
        GPS_longitude = 0; // [deg.min]
        GPS_lon = 'X'; // [E / W]
        GPS_speed = 0; // [knotts] speed over ground
        GPS_angle = 0; // [deg] from true North
        GPS_alt = 0; // [m] above MSL
        GPS_sats = 0; // [#] number of satellites found
      }
    }

    // ICM readings
    sensors_event_t ICM_accel;
    sensors_event_t ICM_gyro;
    sensors_event_t ICM_mag;
    sensors_event_t ICM_temp;
    icm.getEvent(&ICM_accel, &ICM_gyro, &ICM_temp, &ICM_mag);
    ICM_temp1 = ICM_temp.temperature; // [degC]
    ICM_accel_x = ICM_accel.acceleration.x; // [m/s]
    ICM_accel_y = ICM_accel.acceleration.y; // [m/s]
    ICM_accel_z = ICM_accel.acceleration.z; // [m/s]
    ICM_gyro_x = ICM_gyro.gyro.x; // [rad/s]
    ICM_gyro_y = ICM_gyro.gyro.y; // [rad/s]
    ICM_gyro_z = ICM_gyro.gyro.z; // [rad/s]
    ICM_mag_x = ICM_mag.magnetic.x; // [uT]
    ICM_mag_y = ICM_mag.magnetic.y; // [uT]
    ICM_mag_z = ICM_mag.magnetic.z; // [uT]
  }
}

void saveSD(bool save){

  if(save){ // check if save to SD is turned on and the file is open
    if (!headerState){
      char filename[] = "soy000.csv";
      for(int i = 1; i < 1000; i++){
        //make a new file name
        filename[3] = i/100 + '0';
        filename[4] = i/10  + '0';
        filename[5] = i%10  + '0';
        //test the name
        if(!SD.exists(filename)) break; // no need to continue
      }
      myFile = SD.open(filename, FILE_WRITE); // name file here
      if(myFile){
        //Serial.println("File Open");
        myFile.println(header);                       // Write the header for the data
        headerState = TRUE;
      }else{
        Serial.println("File NOT Open");
      }
    }else{
      // BMP Readings
      myFile.print(timestamp);
      myFile.print(", ");
      myFile.print(BMP_temp);
      myFile.print(", ");
      myFile.print(BMP_pres);
      myFile.print(", ");
      myFile.print(BMP_alt);
      myFile.print(", ");
      // BNO Readings
      myFile.print(BNO_X);
      myFile.print(", ");
      myFile.print(BNO_Y);
      myFile.print(", ");
      myFile.print(BNO_Z);
      myFile.print(", ");

      myFile.print(BNO_X_accl);
      myFile.print(", ");
      myFile.print(BNO_Y_accl);
      myFile.print(", ");
      myFile.print(BNO_Z_accl);
      myFile.print(", ");

      myFile.print(BNO_X_gyro);
      myFile.print(", ");
      myFile.print(BNO_Y_gyro);
      myFile.print(", ");
      myFile.print(BNO_Z_gyro);
      myFile.print(", ");

      myFile.print(BNO_X_magn);
      myFile.print(", ");
      myFile.print(BNO_Y_magn);
      myFile.print(", ");
      myFile.print(BNO_Z_magn);
      myFile.print(", ");
      // GPS Readings
      myFile.print(GPS_latitude);
      myFile.print(", ");
      myFile.print(GPS_longitude);
      myFile.print(", ");
      myFile.print(GPS_speed);
      myFile.print(", ");
      myFile.print(GPS_angle);
      myFile.print(", ");
      myFile.print(GPS_alt);
      myFile.print(", ");
      myFile.print(GPS_sats);
      myFile.print(", ");
      // ICM Readings
      myFile.print(ICM_temp1);
      myFile.print(", ");
      myFile.print(ICM_accel_x);
      myFile.print(", ");
      myFile.print(ICM_accel_y);
      myFile.print(", ");
      myFile.print(ICM_accel_z);
      myFile.print(", ");
      myFile.print(ICM_gyro_x);
      myFile.print(", ");
      myFile.print(ICM_gyro_y);
      myFile.print(", ");
      myFile.print(ICM_gyro_z);
      myFile.print(", ");
      myFile.print(ICM_mag_x);
      myFile.print(", ");
      myFile.print(ICM_mag_y);
      myFile.print(", ");
      myFile.println(ICM_mag_z);
      myFile.flush();
    }
  }else{
    if(myFile){
      myFile.close();                         // close the data logging file if its open
      headerState = FALSE;
      //Serial.println("File Closed");
    }
  }
}

void printLCD(bool LCD){
  if(LCD){
    int subVal = digitalRead(switchPin2);           // read the input of the switch (1/0)
    delay(10);                                  // wait 10 ms to read switch state again
    int subVal2 = digitalRead(switchPin2);          // read the input of the switch (1/0)
    
    // Serial.println(subVal); // DEBUG

    if (subVal == subVal2) {                          // compare the 2 readings for bounce
      buttonDuration(subVal);                      // check the button press duration
      if (subVal != buttonState2 && buttonDiff < buttonInterval) { // check if the state has changed AND if the button has been pressed longer than the interval value
        if (subVal == LOW) {                       // check if the button is being pressed
          if (subState == 0) {                     // check if previously in state 0
            subState = 1;                          // change the state to 1
            Serial.println("Button just pressed, Substate 1");
          }else if (subState == 1){                // check if previously in state 1 
            subState = 2;                          // change to 2
            Serial.println("Button just pressed, substate 2");
          }else{                                // check if reviously in state 2 
            subState = 0;                          // change state to 0
            Serial.println("Button just pressed, substate 0");
          }
        }
      }
    }
    
    buttonState2 = subVal;

    if(subState == 0){
      lcd.clear();
      lcd.home();
      lcd.print("Alt: ");
      lcd.print(BMP_alt);
      lcd.print(" m");
    }else if(subState == 1){
      lcd.clear();
      lcd.home();
      lcd.print("Angle: ");
      lcd.print(BNO_X);
      lcd.print((char)223); // degree symbol code from ASCII Table
    }else{
      lcd.clear();
      lcd.home();
      lcd.print("Pos: ");
      lcd.print(GPS_lat);
      lcd.setCursor(0,1);
      lcd.print(GPS_latitude);
    }
  }
}

void printSerial(bool serial){
  if(serial){
    if (!headerStateSerial){
      Serial.println(header);                       // Write the header for the data
      headerStateSerial = TRUE;
    }else{
      // BMP Readings
      Serial.print(timestamp);
      Serial.print(", ");
      Serial.print(BMP_temp);
      Serial.print(", ");
      Serial.print(BMP_pres);
      Serial.print(", ");
      Serial.print(BMP_alt);
      Serial.print(", ");
      // BNO Readings
      Serial.print(BNO_X);
      Serial.print(", ");
      Serial.print(BNO_Y);
      Serial.print(", ");
      Serial.print(BNO_Z);
      Serial.print(", ");

      Serial.print(BNO_X_accl);
      Serial.print(", ");
      Serial.print(BNO_Y_accl);
      Serial.print(", ");
      Serial.print(BNO_Z_accl);
      Serial.print(", ");

      Serial.print(BNO_X_gyro);
      Serial.print(", ");
      Serial.print(BNO_Y_gyro);
      Serial.print(", ");
      Serial.print(BNO_Z_gyro);
      Serial.print(", ");

      Serial.print(BNO_X_magn);
      Serial.print(", ");
      Serial.print(BNO_Y_magn);
      Serial.print(", ");
      Serial.print(BNO_Z_magn);
      Serial.print(", ");
      // GPS Readings
      Serial.print(GPS_latitude);
      Serial.print(", ");
      Serial.print(GPS_longitude);
      Serial.print(", ");
      Serial.print(GPS_speed);
      Serial.print(", ");
      Serial.print(GPS_angle);
      Serial.print(", ");
      Serial.print(GPS_alt);
      Serial.print(", ");
      Serial.print(GPS_sats);
      Serial.print(", ");
      // ICM Readings
      Serial.print(ICM_temp1);
      Serial.print(", ");
      Serial.print(ICM_accel_x);
      Serial.print(", ");
      Serial.print(ICM_accel_y);
      Serial.print(", ");
      Serial.print(ICM_accel_z);
      Serial.print(", ");
      Serial.print(ICM_gyro_x);
      Serial.print(", ");
      Serial.print(ICM_gyro_y);
      Serial.print(", ");
      Serial.print(ICM_gyro_z);
      Serial.print(", ");
      Serial.print(ICM_mag_x);
      Serial.print(", ");
      Serial.print(ICM_mag_y);
      Serial.print(", ");
      Serial.println(ICM_mag_z);
    }
  }
}

/*
float getCalibration(){
  // get calibration values from SD card file
  kMagX
  kMagY
  kMagZ
  kGyroX
  kGyroY
  kGyroZ
  kAccX
  kAccy
  kAccz
  kAlt

  bMagX
  bMagY
  bMagZ
  bGyroX
  bkGyroY
  bGyroZ
  bAccX
  bAccy
  bAccz
  bAlt
}
*/