/*

Adalogger, Relay latching, RTC (DS3231) (0x68), TCA9548A(I2C Multiplexer)(0x70), Monochrome 0.96" 128x64 OLED Graphic Display - STEMMA QT(0x3D)
TCA9548A(I2C Multiplexer): 
Address0->MS8607(0x76)
Address1->Monochrome 0.96" 128x64 OLED Graphic Display - STEMMA QT(0x3D)
Address2->SCD30A (0x61)
Address3->SCD30B (0x61)
Address4->SCD30C (0x61)
Address5->SCD30D (0x61)
Address6->SCD30E (0x61)
Address7->SCD30F (0x61)
BASED ON THE CODE FROM THE FORUM "https://forums.adafruit.com/viewtopic.php?f=25&t=138929", an excellent example how to connect two (or more) different sensors on one multiplexer (if I understand it correctly)
2*KE25 (O2 sensors) connected to the ADS1115 (KE2A to A0-A1, KE25B to A2-A3)
*/

#define TCAADDR 0x70
#include "Wire.h"
// extern "C" {
// #include "utility/twi.h"  // from Wire library, so we can do bus scanning 
// }
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h" // the I2C address is 0x68. For the real time clock commands, please read the LICENSE and README files before downloading the library at https://github.com/adafruit/RTClib
#include <SD.h>
#include <Adafruit_GFX.h>//OLED
#include <Adafruit_SSD1306.h>//OLED
#include <Adafruit_SleepyDog.h> // for sleep mode between readings, please read the LICENSE and README files before downloading the library at https://github.com/adafruit/Adafruit_SleepyDog
#include <Arduino.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MS8607.h>

int Interval=10;// [min] logging interval
int i = 0;//flag to make sure tat the it will not go into sleep mode during the first time after setup
int SLEEP_TIME = 9*60; // //go into sleep mode for..[seconds]
int cycleIndex=1;
int MinutesPerDay = 1440;

//RTC//
char buffer [24];
uint8_t secq, minq, hourq, dayq, monthq, yearq ;
RTC_DS3231 rtc;
//RTC_DS1307 rtc;
float rtcT; 
////


//SD card//
char FileName[]="26032023.txt";// change the file name
// char FileName[]="26032023.csv";// change the file name
const int chipSelect = 4;//SD input
//String LoggerHeader = "Date [hh:mm:ss dd/mm/yyyy],CO2 [ppm] (K30), CO2 [ppm] (SCD30),T [C] (SCD30),RH [%] (
String LoggerHeader = "CO2O2Soils_Mashas1";//insert the heaters for the SD logging
// #define LED 8 // to be used as datalogging indicator
////

//Power relay (latching type)///
int RelayPinOn = 13;//SET in relay connected to pin#13
int RelayPinOff = 12;//UNSET in relay connected to pin#12
////

//Measuring battery voltage///
#define VBATPIN A7
float measuredvbat = 0;
////

////128x64 OLED////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
////////

//ADS1115//
Adafruit_ADS1115 ads1115;//
float bitValue2 = 0.0078125;// how much 1 bit equals
float O2A = 0;
float O2B = 0;
float O2A_percentage = 0;
float O2B_percentage = 0;
////

//SCD30 section
SCD30 airSensor; //The SCD30 has data ready every two seconds
int CO2SCD30A = 0;
float TemperatureSCD30A = 0;
float RHSCD30A = 0;
int CO2SCD30B = 0;
float TemperatureSCD30B = 0;
float RHSCD30B = 0;
int CO2SCD30C = 0;
float TemperatureSCD30C = 0;
float RHSCD30C = 0;
int CO2SCD30D = 0;
float TemperatureSCD30D = 0;
float RHSCD30D = 0;
int CO2SCD30E = 0;
float TemperatureSCD30E = 0;
float RHSCD30E = 0;
int CO2SCD30F = 0;
float TemperatureSCD30F = 0;
float RHSCD30F = 0;
////



//MS8607//
Adafruit_MS8607 ms8607;
double ms8607T = 0;//[C]
double ms8607P = 0;//[hPa]
double ms8607RH = 0;//[%]
////


void setup() {
  Serial.begin(9600);
    // delay(100);//Some of the sensors need a few seconds between power on with the relay and taking a reading or initiating the SD


  pinMode(RelayPinOn, OUTPUT);// initialize digital pin 13 as an output.
  pinMode(RelayPinOff, OUTPUT);// initialize digital pin 12 as an output.
    
    Serial.println("Test1 ");

  digitalWrite(RelayPinOn,HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);//pulse each pin high for 10ms to latch the relay open or closed. 
  digitalWrite(RelayPinOn, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(4000);//Some of the sensors need a few seconds between power on with the relay and taking a reading or initiating the SD
  Serial.println("Test2 ");

  
  //RTC//
  #ifndef ESP8266
    //while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    // abort();
  }
    Serial.println("Test3 ");

//Calibraet the rtc acording to UTC+2 (winter time in Israel)//
// rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // USE ONLY IF WINTER TIME IS SET IN THE CONNECTED COMPUTER
// rtc.adjust(DateTime(F(__DATE__), F(__TIME__))- TimeSpan(0,1,0,0)); // USE ONLY IF SUMMER TIME IS SET IN THE CONNECTED COMPUTER (*)e.g., minus 1 hour)
 ////

 //SD card//
 delay(10);
 Serial.print(F("Initializing SD card..."));
 // see if the card is present and can be initialized:
 SD.begin(chipSelect);
 if (!SD.begin(chipSelect)) {
   Serial.println(F("Card failed, or not present"));
  // don't do anything more:
  // while (1);
  }
  Serial.println(F("card initialized."));
  Serial.print("File name: ");
  Serial.print(FileName);
  File dataFile = SD.open(FileName, FILE_WRITE);// testing if this is a new file then add heaters
  Serial.print("File size: ");
  Serial.println(dataFile.size());
  if (dataFile.size()==0){
    dataFile.println(LoggerHeader);
    // print to the serial port too:
    Serial.println(LoggerHeader);
  }
  dataFile.close();
  delay(10);
 ////

 //ADS1115//
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  // tcaselect(1);
  ads1115.begin();// Construct an ads1015 at the default
  ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
   ////
    // Serial.println("Test3a ");







  digitalWrite(RelayPinOff,HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(20);//pulse each pin high for 10ms to latch the relay open or closed. 
  digitalWrite(RelayPinOff, LOW);   // turn the LED on (HIGH is the voltage level)
  
}

void loop() {

// delay(1*60*1000);




int loopTimeSec = round(22660*0.001);//[sec] this is the duration of the sensors and datalogging loop - SHOULD BE CAHNGED IF THE CODE IS CHANGED
//instead of sleep mode//
if (i==1){ 
// delay((Interval-1)*60*1000);
delay(((Interval-1)*60*1000)+((loopTimeSec*1000)-5000));

// delay((Interval-(loopTimeSec*1000)-5000)*60*1000);

i = 0;
}

////
    //go into sleep mode//

  // if (i==1){   
  //   for(int q= 0; q < SLEEP_TIME / 8; q++) { // this section was adjusted from "https://tum-gis-sensor-nodes.readthedocs.io/en/latest/adafruit_32u4_lora/README.html"
  //     Watchdog.sleep(8000); // in milliseconds - the watchdog timer max sleep time is 8 seconds so an adjustment is needed (see the link above for additional details)
  //   }
  //   if (SLEEP_TIME % 8) {
  //     Watchdog.sleep((SLEEP_TIME % 8)*1000);
  //    }
  //   //  delay (9000-33.333);//add this to the SLEEP_TIME to set interval
  //   i=0;
  // }

//delay (5000-33.333);  
  //////////////////////

  int StartOfLoop = millis();
  Serial.print("start of ?-min interval loop in milis:");
  Serial.println(StartOfLoop);
      
      Serial.println("Relay pin on start");       
  digitalWrite(RelayPinOn,HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(20);//pulse each pin high for 10ms to latch the relay open or closed. 
  digitalWrite(RelayPinOn, LOW);   // turn the LED on (HIGH is the voltage level)
  Serial.println("Relay pin on end"); 
  //sensor warm up time should be ~10 sec from relay on to first CO2 measurement


   //RTC//
  // rtc.begin(); // this line should be uncomment only if using the relay
  rtcT = rtc.getTemperature();
  DateTime now = rtc.now();
  secq = now.second();
  minq = now.minute();
  hourq = now.hour();
  dayq = now.day();
 monthq = now.month(); 
  sprintf (buffer, "%02u:%02u:%02u %02u/%02u", hourq, minq, secq, dayq, monthq);//this is to zero in single nreadings (e.g., 5 sec to 05 sec)
  

  Serial.print ("Minutes:");
  Serial.print (minq);
  Serial.print (" Seconds:");
  Serial.println (secq);

  // if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //     display.clearDisplay();
  //     display.setTextSize(1);
  //     display.setTextColor(WHITE);
  //     display.setCursor(0,0);
  //     display.print("");
  //     display.print("Second:");display.print(secq);display.print("minutes");display.print(minq);
  //     display.display();     

  //    }//part of the OLED

  int Mod_A = (hourq*60+minq) % Interval ;
  if (Mod_A == 0 && secq == 0){//Comment if ?-min interval is not needed
    int countdownMS = Watchdog.enable(60000); // start a 60 seconds timer - this will reset the microcontroller in case it will get stuck in the loop (e.g., a problematic sensor)




     //ADS1115//
    O2A = ads1115.readADC_Differential_0_1()*bitValue2;//[mV]
    O2B = ads1115.readADC_Differential_2_3()*bitValue2;//[mV]
    O2A_percentage = 1.79031686955191*O2A-0.427551020408163;//[%]
    O2B_percentage = 1.74752514028074*O2B-0.427551020408163;//[%]
    ////

      //Measuring battery voltage///
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    ////

  //SCD30 section - start//
    tcaselect(2);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
    if (airSensor.dataAvailable())
    {
      CO2SCD30A = airSensor.getCO2();
      TemperatureSCD30A = airSensor.getTemperature(), 1;
      RHSCD30A = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      Serial.println("CO2_A-V ");

    }
          // Serial.println("Test5: ");
    }
    tcaselect(3);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
      if (airSensor.dataAvailable())
    {
      CO2SCD30B = airSensor.getCO2();
      TemperatureSCD30B = airSensor.getTemperature(), 1;
      RHSCD30B = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30B10cm);
      Serial.println("CO2_B-V ");

    }
    }

        tcaselect(4);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
      if (airSensor.dataAvailable())
    {
      CO2SCD30C = airSensor.getCO2();
      TemperatureSCD30C = airSensor.getTemperature(), 1;
      RHSCD30C = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30B10cm);
      Serial.println("CO2_C-V ");

    }
    }

        tcaselect(5);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
      if (airSensor.dataAvailable())
    {
      CO2SCD30D = airSensor.getCO2();
      TemperatureSCD30D = airSensor.getTemperature(), 1;
      RHSCD30D = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30B10cm);
      Serial.println("CO2_D-V ");

    }
    }

        tcaselect(6);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
      if (airSensor.dataAvailable())
    {
      CO2SCD30E = airSensor.getCO2();
      TemperatureSCD30E = airSensor.getTemperature(), 1;
      RHSCD30E = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30B10cm);
      Serial.println("CO2_E-V ");

    }
    }

        tcaselect(7);
    if (airSensor.begin()){;// this line should be uncomment only if using the relay
    delay(100);
      if (airSensor.dataAvailable())
    {
      CO2SCD30F = airSensor.getCO2();
      TemperatureSCD30F = airSensor.getTemperature(), 1;
      RHSCD30F = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30B10cm);
      Serial.println("CO2_F-V ");

    }
    }

 //SCD30 section - end//
    
      //MS8607//
    tcaselect(0);
    if (ms8607.begin()) {
      delay(100);

    //setup lines - these lines are here due to the use of the relay
    ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_8b);
    switch (ms8607.getHumidityResolution()){
    // case MS8607_HUMIDITY_RESOLUTION_OSR_12b: Serial.println("12-bit"); break;
    // case MS8607_HUMIDITY_RESOLUTION_OSR_11b: Serial.println("11-bit"); break;
    // case MS8607_HUMIDITY_RESOLUTION_OSR_10b: Serial.println("10-bit"); break;
    // case MS8607_HUMIDITY_RESOLUTION_OSR_8b:  Serial.println("8-bit"); break;
  }
  // ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096);
  //Serial.print("Pressure and Temperature resolution set to ");
  switch (ms8607.getPressureResolution()){
    // case MS8607_PRESSURE_RESOLUTION_OSR_256: Serial.println("256"); break;
    // case MS8607_PRESSURE_RESOLUTION_OSR_512: Serial.println("512"); break;
    // case MS8607_PRESSURE_RESOLUTION_OSR_1024: Serial.println("1024"); break;
    // case MS8607_PRESSURE_RESOLUTION_OSR_2048: Serial.println("2048"); break;
    // case MS8607_PRESSURE_RESOLUTION_OSR_4096: Serial.println("4096"); break;
    // case MS8607_PRESSURE_RESOLUTION_OSR_8192: Serial.println("8192"); break;
  }
  ////end of setup lines
      sensors_event_t temp, pressure, humidity;
      ms8607.getEvent(&pressure, &temp, &humidity);
      ms8607T = temp.temperature;//[C]
      ms8607P = pressure.pressure;//[hPa]
      ms8607RH = humidity.relative_humidity;//[%]
    }
    Serial.println("Sensor ms8607T - V");
    ////

  
    //SD logging//
    // make a string for assembling the data to log:
    String dataString = ""; 
    dataString += String(buffer);dataString += "/";dataString += String(now.year());  //HH:MM:SS DD/MM/YY
      String timeString = ""; 
      timeString += dataString;
    dataString += ",";
    dataString += String(CO2SCD30A);
    dataString += ",";
    dataString += String(TemperatureSCD30A);
    dataString += ",";
    dataString += String(RHSCD30A);
    dataString += ",";
    dataString += String(CO2SCD30B);
    dataString += ",";
    dataString += String(TemperatureSCD30B);
    dataString += ",";
    dataString += String(RHSCD30B);
        dataString += ",";
    dataString += String(CO2SCD30C);
    dataString += ",";
    dataString += String(TemperatureSCD30C);
    dataString += ",";
    dataString += String(RHSCD30C);
        dataString += ",";
    dataString += String(CO2SCD30D);
    dataString += ",";
    dataString += String(TemperatureSCD30D);
    dataString += ",";
    dataString += String(RHSCD30D);
        dataString += ",";
    dataString += String(CO2SCD30E);
    dataString += ",";
    dataString += String(TemperatureSCD30E);
    dataString += ",";
    dataString += String(RHSCD30E);
        dataString += ",";
    dataString += String(CO2SCD30F);
    dataString += ",";
    dataString += String(TemperatureSCD30F);
    dataString += ",";
    dataString += String(RHSCD30F);
        dataString += ",";        
    dataString += String(O2A);
    dataString += ",";   
    dataString += String(O2B);
    dataString += ",";
    dataString += String(O2A_percentage);
    dataString += ",";
    dataString += String(O2B_percentage);
    dataString += ",";       
    dataString += String(ms8607T);
    dataString += ",";
    dataString += String(ms8607P);
    dataString += ",";
    dataString += String(ms8607RH);
    dataString += ",";
    dataString += String(measuredvbat);//[V]
    dataString += ",";
    dataString += String(cycleIndex);//how many sensors measurement cycles were conducted from the last reset
    



    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // digitalWrite(LED, HIGH);
    delay(500);
    File dataFile = SD.open(FileName, FILE_WRITE);
    delay(500);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
       ////128x64 OLED////
       tcaselect(1);
       delay(100);
      if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(0,0);
          display.print(dataString);
          // display.println(timeString);
          // display.print("CO2:");display.print(CO2SCD30A);display.print(",");display.print(CO2SCD30B);display.print(",");display.print(CO2SCD41C);display.print(",");display.print(CO2SCD41D);display.print(";");
          // display.print("O2:");display.print(O2A_percentage);display.print(",");display.print(O2B_percentage);display.print(";");
          // display.print("B[V]:");display.print(measuredvbat);display.print(";");
          display.display();
          // delay(5000); 
      }//part of the OLED

    // digitalWrite(LED, LOW);  
    }
       // if the file isn't open, pop up an error:
    else {
      Serial.println(F("error opening datalog file"));

      ////128x64 OLED////
      tcaselect(1);
      delay(50);

  if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println(timeString);
      display.print("NO SD!");
      display.print(dataString);
      display.display();

     }//part of the OLED
      Watchdog.enable(5*1000);//reset the adalogger after 5sec if no SD wad logged
      delay(7*1000); 
      
    }
    
    delay(10000);
    digitalWrite(RelayPinOff,HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(20);//pulse each pin high for 10ms to latch the relay open or closed. 
    digitalWrite(RelayPinOff, LOW);   // turn the LED on (HIGH is the voltage level)

  //Reset the device once a day//
  cycleIndex++;
  if (cycleIndex> (MinutesPerDay/Interval)){
      Serial.println("testReset");
      cycleIndex=1;
      Watchdog.enable(1000);// reset the device
      delay(2000);
  } 
  ////



  Watchdog.disable(); // end the 60 seconds timer

  i=1;
  } //end of 1-min interval
  int EndOfLoop = millis();
  Serial.print("end of ?-min interval loop in millis:");
  Serial.println(EndOfLoop);
  Serial.print("Toal ?-min interval loop in millis:");
  Serial.println(EndOfLoop-StartOfLoop); 
  Serial.println("----------------------------------------------------------------------");

  // delay(3000);

}


void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

