//Soil CO2 sensor system includes:
//Notecard Cellular + Notecarrier F        Serial1
//Feather M0 adalogger with SD card
//RTC DS3231 featherwing                  (I2C - 0x68)
//Multiplexer TCA9548A  (DFrobot)         (I2C - 0x70)
// 6 CO2 sensors (on mux ports 0-5)
// 1 microclimate sensors MS8607 (on mux port: 6)
//1 soil moisture sensor TDR teros12
//Li ion battery 3.7 volt 
//Solar panel 6V 6W


// notecard and hub:       using sleepy sensor example, the feather samples every fixed interval (that can be changed in notehub) 
//                         and sends data to notehub depending on the system voltage
//                         In order to allow the notecarrier to do this the Notecard's `ATTN`* pin is wired to the enable pin 'EN' of the adalogger
//                         Sync to notehub every reading

//Import libraries
#include <Wire.h>  //include arduinos i2c library
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Notecard.h>                        // Include the Arduino library for the Notecard
#include "SparkFun_SCD30_Arduino_Library.h"  //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SDI12_ARM.h"                       //include library for the moisture sensor  GS3 https://github.com/ManuelJimenezBuendia/Arduino-SDI-12. here we use Teros 12 but we can use the same library
#include <Adafruit_MS8607.h>                 //include library for microclimate sensor MS8607


#define TCAADDR 0x70 //address of I2C mux


//define variables for 6 CO2 sensors (each include CO2 concentration, Temperature, and RH), labeled from A-F
SCD30 airSensor;           //The SCD30 has data ready every two seconds
float CO2SCD30A, TemperatureSCD30A, RHSCD30A, CO2SCD30B, TemperatureSCD30B, RHSCD30B, CO2SCD30C, TemperatureSCD30C, RHSCD30C;  //float var used to hold the values of sensors
float CO2SCD30D, TemperatureSCD30D, RHSCD30D, CO2SCD30E, TemperatureSCD30E, RHSCD30E, CO2SCD30F, TemperatureSCD30F, RHSCD30F;  //float var used to hold the values of sensors

//define variable for microlimate sensor MS8607//
Adafruit_MS8607 ms8607;
double ms8607T = 0;//[C]
double ms8607P = 0;//[hPa]
double ms8607RH = 0;//[%]

// Moisture sensor constants Teros12//
static const uint16_t pin_GS3_SDI12 = 6;  // Set the pins used on main control board (connect the sensor's data wire to port "6" or change the port and update it here)
SDI12 GS3_SDI12(pin_GS3_SDI12);           // Create an SDI-12 object and commands;
static const String myCommandInfo = "?I!"; //"?I!" = get info; // returns address and unit specs 
static const String myCommandMeas = "0M!";   //"aM!" = take measurement; 
static const String myCommandData = "0D0!";  //"aD0!" = read data; // returns "address+calibratedcountsVWC+temp+EC". To convert calibratedcountsSWC or TDR_RAW to VWC use the general equation provided in manual guide of TEROS12
char* samples;
// Teros12 values
float TDR_RAW; //this is termed calibrated counts as in the manual guide. This value is not SWC or dielectric, it has to be calibrated to convert to SWC or dielectric. The manufacturer provided an equation to convert for general minineral soil
float TDR_degC; //for soil temperature
float TDR_bulkEC; //for bulk elecetric conductivity

//logging file and SD card define. Measurement readings are sent to notehub but still SD card is used as a backup
char FileName[] = "soilCO2.txt";                     // logging file name
const int chipSelect = 4;                             // SD input - feather M0
String LoggerHeader = "Time , Date ,  Battery_Volt";  //headers- will be printed on the first line of the file


//define variable for RTC temp
float RTCtemp;
RTC_DS3231 rtc;                           //RTC define
char buffer[24];                          //float var used to hold time and date
uint8_t secq, minq, hourq, dayq, monthq;  //float var used to hold the sec, min, hour, day and month

//Battery volt
#define VBATPIN A7       //Bat pin feather
float Batt_V = 0;             //float variable for adalogger voltage
float BluesVoltage = 0;       //float variable for Notecarrier voltage

int getInterval();  // calls function that returns the reading_interval from the environment variables



//note hub define
#define productUID "il.ac.bgu.levintal:co2o2soils"  //the unique Product Identifier for your device - set at notehub
#define myProductID productUID
Notecard notecard;
//#define txRxPinsSerial Serial1                                  //uncomment when using UART protocol
#define usbSerial Serial


void setup() {
  //while (!Serial); delay(1000);           //uncomment if you want to debug with serial
  Serial.begin(115200);
  notecard.setDebugOutputStream(usbSerial);  // enables to watch JSON objects being transferred to and from the Notecard for each request on the serial terminal window
  notecard.begin(Serial1, 9600);               //uncomment when using UART protocol
  // notecard.begin();  //start notecard with I2C protocol
  Wire.begin();      //start the I2C

  // Provide visual signal when the Host MCU is powered
  ::pinMode(LED_BUILTIN, OUTPUT);
  ::digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("CO2 soil system");
  Serial.println("system setup, please wait");
  delay(100);


  //Note card setup
  {
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "periodic");
    JAddStringToObject(req, "vinbound", "usb:1;high:120;normal:240;low:480;dead:0");  //The max wait time (in minutes) to sync inbound data from Notehub - with a voltage-variable value
    JAddStringToObject(req, "voutbound", "usb:1;high:60;normal:90;low:120;dead:0");   //The max wait time (minutes) to sync outbound data from the Notecard - with a voltage-variable value
    JAddBoolToObject(req, "sync", true);
    notecard.sendRequestWithRetry(req, 5);
  }
  //Optimize voltage variable behaviors for LiPo battery
  {
    J *req = notecard.newRequest("card.voltage");
    JAddStringToObject(req, "mode", "lipo");  // "lipo" for LiPo batteries. Equivalent to "usb:4.6; high:4.0; normal:3.5; low:3.2; dead:0"
    notecard.sendRequest(req);
  }
  //Establish a template to optimize queue size and data usage
  ///*
  {
  J * req = notecard.newRequest("note.template");
  JAddStringToObject(req, "file", "readings.qo");
  J * body = JAddObjectToObject(req, "body");
  JAddStringToObject(body, "Time", "10:24:00 30/04/2024");
  JAddNumberToObject(body, "CO2SCD30A", 12.1);
  JAddNumberToObject(body, "CO2SCD30B", 12.1);
  JAddNumberToObject(body, "CO2SCD30C", 12.1);
  JAddNumberToObject(body, "CO2SCD30D", 12.1);
  JAddNumberToObject(body, "CO2SCD30E", 12.1);
  JAddNumberToObject(body, "CO2SCD30F", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30A", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30B", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30C", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30D", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30E", 12.1);
  JAddNumberToObject(body, "TemperatureSCD30F", 12.1);
  JAddNumberToObject(body, "RHSCD30A", 12.1);
  JAddNumberToObject(body, "RHSCD30B", 12.1);
  JAddNumberToObject(body, "RHSCD30C", 12.1);
  JAddNumberToObject(body, "RHSCD30D", 12.1);
  JAddNumberToObject(body, "RHSCD30E", 12.1);
  JAddNumberToObject(body, "RHSCD30F", 12.1);
  JAddNumberToObject(body, "ms8607T", 12.1);
  JAddNumberToObject(body, "ms8607P", 12.1);
  JAddNumberToObject(body, "ms8607RH", 12.1);
  JAddNumberToObject(body, "TDR_RAW", 12.1);
  JAddNumberToObject(body, "TDR_degC", 12.1);
  JAddNumberToObject(body, "TDR_bulkEC", 12.1);
  JAddNumberToObject(body, "RTCtemp", 12.1);
  JAddNumberToObject(body, "Batt_V", 12.1);
  JAddNumberToObject(body, "BluesVoltage", 12.1);
  notecard.sendRequest(req);
  }
//*/


  //SD card setup
  delay(10);
  Serial.print(F("Initializing SD card... "));
  delay(100);
  SD.begin(chipSelect);  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F(" SD card error/No card"));
    delay(100);
    //while (1);
  }  // don't do anything more
  Serial.print(F(" SD card initialized"));
  delay(100);  //  card is present and can be initialized
  Serial.print("   ,Logging File name: ");
  Serial.print(FileName);
  delay(100);
  File dataFile = SD.open(FileName, FILE_WRITE);  // testing if this is a new file then add headers
  Serial.print(" ,File size: ");
  Serial.print(dataFile.size() / 1024);
  Serial.println(" KB");
  delay(100);
  if (dataFile.size() == 0) {
    dataFile.println(LoggerHeader);  // print headers in the SD logging file
    Serial.print("logging headers:");
    Serial.println(LoggerHeader);  // print headers to the serial port
  }
  dataFile.close();
  delay(10);


  Serial.println("starting measurments!");

  step1();  //step 1: read RTC, CO2 sensors, microclimate sensor, moisture sensor and measure battary voltager
  step2();  //step 2: print results on serial monitor
  step3();  //step 3: Make a string with Time Date and the data, and Log it on SD card
  step4();  //step 4:Notecard Requests
  delay(1000); //milisecond

}  //end of setup


void loop() {
  // int IntervalSeconds = getInterval();
  int IntervalSeconds = 573;  //[sec]
  Serial.print("entering sleep mode for ");
  Serial.print(IntervalSeconds);
  Serial.println(" seconds");
  delay(3000);
  J *req = NoteNewCommand("card.attn");
  JAddStringToObject(req, "mode", "sleep");
  JAddNumberToObject(req, "seconds", IntervalSeconds);
  notecard.sendRequest(req);
  // Wait 1s before retrying
  ::delay(1000);
}



//step 1: read RTC, CO2 sensors, microclimate sensor, moisture sensor and measure battary voltager
void step1() {
  Serial.println("starting step 1");
  delay(10000);

  //RTC setup
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    //while (1) delay(10);
  }
  Serial.println("RTC connected");
  delay(100);
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // USE IF WINTER TIME IS SET IN THE CONNECTED COMPUTER
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))- TimeSpan(0,1,0,0));           // USE IF SUMMER TIME IS SET IN THE CONNECTED COMPUTER (*)e.g., minus 1 hour)
    Serial.println("set the time on RTC!");
    delay(500);
  }
  
  DateTime now = rtc.now();
  secq = now.second();
  minq = now.minute();
  hourq = now.hour();
  dayq = now.day();
  monthq = now.month();
  sprintf(buffer, "%02u:%02u:%02u %02u/%02u/%04u", hourq, minq, secq, dayq, monthq, now.year());


  // read CO2 sensors //
  // read CO2A data //
  tcaselect(0);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30A = airSensor.getCO2();
      TemperatureSCD30A = airSensor.getTemperature(), 1;
      RHSCD30A = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }


  // read CO2B data //
  tcaselect(1);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30B = airSensor.getCO2();
      TemperatureSCD30B = airSensor.getTemperature(), 1;
      RHSCD30B = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }

  // read CO2C data //
  tcaselect(2);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30C = airSensor.getCO2();
      TemperatureSCD30C = airSensor.getTemperature(), 1;
      RHSCD30C = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }

  // read CO2D data //
  tcaselect(3);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30D = airSensor.getCO2();
      TemperatureSCD30D = airSensor.getTemperature(), 1;
      RHSCD30D = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }

  // read CO2E data //
  tcaselect(4);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30E = airSensor.getCO2();
      TemperatureSCD30E = airSensor.getTemperature(), 1;
      RHSCD30E = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }

  // read CO2F data //
  tcaselect(5);
  if (airSensor.begin()) {
      // this line should be uncomment only if using the relay
    //delay(10000);
    delay(100);
    if (airSensor.dataAvailable()) {
      CO2SCD30F = airSensor.getCO2();
      TemperatureSCD30F = airSensor.getTemperature(), 1;
      RHSCD30F = airSensor.getHumidity(), 1;
      // Serial.println(TemperatureSCD30A10cm);
      //Serial.println("CO2_A-V ");
    }
  }
  //MS8607//
    tcaselect(6);
    if (ms8607.begin()) {
    delay(100);
    //setup lines - these lines are here due to the use of the relay
    ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_8b);
    sensors_event_t temp, pressure, humidity;
    ms8607.getEvent(&pressure, &temp, &humidity);
    ms8607T = temp.temperature;//[C]
    ms8607P = pressure.pressure;//[hPa]
    ms8607RH = humidity.relative_humidity;//[%]
    }
  

  delay(200);


// read from Teros12 moisture
    String GS3data;
    GS3_SDI12.begin();
    GS3_SDI12.sendCommand(myCommandMeas);  // start meas; 
    delay(100);                            //was 100 at first instaltion
    while (GS3_SDI12.available()) GS3_SDI12.read();
    delay(1000);  // was 2000 at first instaltion //wait a "001" second to measure
    GS3_SDI12.sendCommand(myCommandData);
    delay(100);  //was 100 at first instaltion
    while (GS3_SDI12.available()) {
      uint8_t data = GS3_SDI12.read();
      if (data != '\r' && data != '\n')
        GS3data += (char)data;
    }
    GS3_SDI12.end();
    if (GS3data.length() < 4) {
      //Serial.print("GS3 error");
    }
    //Serial.println(GS3data);
    //Convert string to char for further analysis";
    int str_len = GS3data.length() + 1;
    char samples[str_len];
    GS3data.toCharArray(samples, str_len);
    // first term is the sensor address (irrelevant to me)
    char* term1 = strtok(samples, "+");
    // second term is RAW
    term1 = strtok(NULL, "+");
    TDR_RAW = atof(term1);
    // int RAW = RAW;
    term1 = strtok(NULL, "+");
    TDR_degC = atof(term1);
    term1 = strtok(NULL, "+");
    TDR_bulkEC = atof(term1);
    // term1 = strtok(NULL, "+");
    // porewaterEC = atof(term1);

// read from voltage from battery
  Batt_V = analogRead(VBATPIN);  Batt_V *= 2;  Batt_V *= 3.3; Batt_V /= 1024; // multiply by 3.3V, our reference voltage Batt_V /= 1024; // convert to voltage  
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));
  if (rsp != NULL) {
      BluesVoltage = JGetNumber(rsp, "value");
      notecard.deleteResponse(rsp);
  }
//read temperature from RTC
  RTCtemp = rtc.getTemperature();

}

//step 2: print results on serial monitor
void step2() {
  Serial.println(String(buffer));
  Serial.print(" ,  CO2_A [ppm]: ");
  Serial.print(CO2SCD30A);
  Serial.print(" , Temperature_A [C]: ");
  Serial.print(TemperatureSCD30A);
  Serial.print(" , RH_A [%]: ");
  Serial.println(RHSCD30A);
  Serial.print(" ,  CO2_B [ppm]: ");
  Serial.print(CO2SCD30B);
  Serial.print(" , Temperature_B [C]: ");
  Serial.print(TemperatureSCD30B);
  Serial.print(" , RH_B [%]: ");
  Serial.println(RHSCD30B);
  Serial.print(" ,  CO2_C [ppm]: ");
  Serial.print(CO2SCD30C);
  Serial.print(" , Temperature_C [C]: ");
  Serial.print(TemperatureSCD30C);
  Serial.print(" , RH_C [%]: ");
  Serial.println(RHSCD30C);
  Serial.print(" ,  CO2_D [ppm]: ");
  Serial.print(CO2SCD30D);
  Serial.print(" , Temperature_D [C]: ");
  Serial.print(TemperatureSCD30D);
  Serial.print(" , RH_D [%]: ");
  Serial.println(RHSCD30D);
  Serial.print(" ,  CO2_E [ppm]: ");
  Serial.print(CO2SCD30E);
  Serial.print(" , Temperature_E [C]: ");
  Serial.print(TemperatureSCD30E);
  Serial.print(" , RH_E [%]: ");
  Serial.println(RHSCD30E);
  Serial.print(" ,  CO2_F [ppm]: ");
  Serial.print(CO2SCD30F);
  Serial.print(" , Temperature_F [C]: ");
  Serial.print(TemperatureSCD30F);
  Serial.print(" , RH_F [%]: ");
  Serial.println(RHSCD30F);
  Serial.print(" airTemp:");
  Serial.print(ms8607T);
  Serial.print(" , air pressure: ");
  Serial.print(ms8607P);
  Serial.print(" , air RH: ");
  Serial.println(ms8607RH);
  Serial.print(" , TDR_RAW: ");
  Serial.print(TDR_RAW);
  Serial.print(" , TDR_degC: ");
  Serial.print(TDR_degC);
  Serial.print(" , TDR_bulkEC: ");
  Serial.print(TDR_bulkEC);
  // Serial.print(" , Pore Water EC: ");
  // Serial.println(porewaterEC);
  Serial.print(" RTCtemp:");
  Serial.print(RTCtemp);
  Serial.print(",  Batt_V");
  Serial.println(Batt_V);
  Serial.print(",  BluesVoltage");
  Serial.println(BluesVoltage);
  //Serial.println("finishing step 4");
}

//step 3: Make a string with Time Date and the data, and Log it on SD card
void step3() {
  //Serial.println("starting step 5");
  String dataString = "";
  dataString += String(buffer);  // HH:MM:SS DD/MM/YYYY
  //String timeString = "";
  //timeString += dataString;
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
  dataString += String(ms8607T);
  dataString += ",";
  dataString += String(ms8607P);
  dataString += ",";
  dataString += String(ms8607RH);
  dataString += ",";
  dataString += String(TDR_RAW);
  dataString += ",";
  dataString += String(TDR_degC);
  dataString += ",";
  dataString += String(TDR_bulkEC);
  dataString += ",";
  dataString += String(Batt_V);
  dataString += ",";
  dataString += String(BluesVoltage);

  File dataFile = SD.open(FileName, FILE_WRITE);  // open logging file. note that only one file can be open at a time, so you have to close this one before opening another.
  delay(100);
  if (dataFile) {  // if the file is available, write to it
    dataFile.println(dataString);
    dataFile.close();
    Serial.print("SD string:");
    Serial.print(dataString);  //prints on serial the string that os logged on the SD card
    Serial.println("Data logged to SD!");
    //display.setCursor(116, 0); display.println("SD"); display.display();
  } else {
    Serial.println(F("error with SD card - no SD logging!"));
    //display.setCursor(116, 0); display.println("XX"); display.display();
  }
  Serial.println("finishing step 3");
}

//step 4: Notecard Requests
void step4() {
  const char *time = buffer;
  J *req = notecard.newRequest("note.add");
  if (req != NULL) {
    JAddStringToObject(req, "file", "readings.qo");
    JAddBoolToObject(req, "sync", true);
    J *body = JAddObjectToObject(req, "body");
    if (body) {
      JAddStringToObject(body, "Time", time);
      JAddNumberToObject(body, "CO2SCD30A", CO2SCD30A);
      JAddNumberToObject(body, "TemperatureSCD30A", TemperatureSCD30A);
      JAddNumberToObject(body, "RHSCD30A", RHSCD30A);
      JAddNumberToObject(body, "CO2SCD30B", CO2SCD30B);
      JAddNumberToObject(body, "TemperatureSCD30B", TemperatureSCD30B);
      JAddNumberToObject(body, "RHSCD30B", RHSCD30B);
      JAddNumberToObject(body, "CO2SCD30C", CO2SCD30C);
      JAddNumberToObject(body, "TemperatureSCD30C", TemperatureSCD30C);
      JAddNumberToObject(body, "RHSCD30C", RHSCD30C);
      JAddNumberToObject(body, "CO2SCD30D", CO2SCD30D);
      JAddNumberToObject(body, "TemperatureSCD30D", TemperatureSCD30D);
      JAddNumberToObject(body, "RHSCD30D", RHSCD30D);
      JAddNumberToObject(body, "CO2SCD30E", CO2SCD30E);
      JAddNumberToObject(body, "TemperatureSCD30E", TemperatureSCD30E);
      JAddNumberToObject(body, "RHSCD30E", RHSCD30E);
      JAddNumberToObject(body, "CO2SCD30F", CO2SCD30F);
      JAddNumberToObject(body, "TemperatureSCD30F", TemperatureSCD30F);
      JAddNumberToObject(body, "RHSCD30F", RHSCD30F);
      JAddNumberToObject(body, "ms8607T", ms8607T);
      JAddNumberToObject(body, "ms8607P", ms8607P);
      JAddNumberToObject(body, "ms8607RH", ms8607RH);
      JAddNumberToObject(body, "TDR_RAW", TDR_RAW);
      JAddNumberToObject(body, "TDR_degC", TDR_degC);
      JAddNumberToObject(body, "TDR_bulkEC", TDR_bulkEC);
      JAddNumberToObject(body, "RTCtemp", RTCtemp);
      JAddNumberToObject(body, "Batt_V", Batt_V);
      JAddNumberToObject(body, "BluesVoltage", BluesVoltage);
    }

    notecard.sendRequest(req);
    // display.setCursor(116, 48); display.println("NH"); display.display();
  }
  Serial.println("finishing step 4 ");
}


// This function returns the reading_interval from the environment variables
int getInterval() {
  int IntervalSeconds = 80;                               //a default value (in seconds)- If the variable is not set, set to 0, or set to an invalid type
  J *req = notecard.newRequest("env.get");
  if (req != NULL) {
      JAddStringToObject(req, "name", "reading_interval");
      J* rsp = notecard.requestAndResponse(req);
      int readingIntervalEnvVar = atoi(JGetString(rsp, "text"));
      if (readingIntervalEnvVar > 0) {
        IntervalSeconds = readingIntervalEnvVar;
      }
      notecard.deleteResponse(rsp);
  }
  return IntervalSeconds;
}

//this function controls the I2C multiplexet
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
