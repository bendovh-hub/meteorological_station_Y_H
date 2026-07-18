/* This code is the final project for the Envirotech course (DIY sensors for environmental research), 
and is part of an open-source hardware and software meteorological station. It works as follows:

Powers on
Checks connection to the S
D card and to each sensor via the I2C multiplexer
Waits for the start of a round minute
Takes measurements
Saves the readings to the SD card
Saves the readings to the Blues Notecard (syncs to the cloud roughly every 30 minutes)
Goes to sleep for about two minutes, then wakes up and repeats
*/
#include <RTClib.h>   // includes relevant librerys
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Arduino.h>                               
#include <Notecard.h>                              
#include <Adafruit_SleepyDog.h>                    
#include <Adafruit_SCD30.h>
#include "DFRobot_RainfallSensor.h"

// Sensor Enable/Disable Switches (for code flexabilitie)
#define ENABLE_BME280    true   // BME280 (I2C via Multiplexer CH 7)
#define ENABLE_RAIN      true   // Rain Sensor (I2C via Multiplexer CH 0)
#define ENABLE_BMP581    true   // BMP581 (I2C via Multiplexer CH 6)
#define ENABLE_SHT31     true   // SHT31 (I2C via Multiplexer CH 3)

#define SENSOR_TIMEOUT_MS 3000 // if sensor doesnt respond the system will continue (prevant crushing)
#define NOTE_FILE_NAME "Logreadings5.qo" // file name target (NOtECARD)
#define WAKE_BEFORE_MINUTE_SEC 14   // wake up -- seconds before the next round minute

#if ENABLE_BMP581            // includes relevant librerys
  #include "Adafruit_BMP5xx.h"
#endif



#if ENABLE_BME280
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
#endif

#if ENABLE_SHT31
  #include "Adafruit_SHT31.h"
#endif

char buffer [24];                           // temporary box for time delay reading
uint8_t secq, minq, hourq, dayq, monthq;    // 8 bytes up to 254 only +
uint16_t yearq; // 16 bytes up to 65535 only +
int delayq;   // save delayq as intiger

// --- Multiplexer Setup ---
#define TCAADDR 0x70 // mux adress

void tcaSelect(uint8_t i) {  // define wich port on multiplexer
  if (i > 7) return; //(only 8 ports in this mux)
  Wire.beginTransmission(TCAADDR); // open comunication with mux
  Wire.write(1 << i); //define port
  Wire.endTransmission();  // send data and close communication
}

void tcaDisable() {   //close ports in mux
  Wire.beginTransmission(TCAADDR); //open comunication with mux
  Wire.write(0); // close all ports (so no port will stay opan when reading is finished)
  Wire.endTransmission();  //close communication
  delay(10); // delay between ports fo power managmant
}

// Global Sensor Objects
#if ENABLE_BME280 
  Adafruit_BME280 bme; // creating object
#endif //

#if ENABLE_BMP581
  Adafruit_BMP5xx bmp; // creating object
#endif //

#if ENABLE_SHT31
  Adafruit_SHT31 sht31 = Adafruit_SHT31(); // creating object
#endif //

#if ENABLE_RAIN
  DFRobot_RainfallSensor_I2C rain(&Wire); // creating object
#endif //
// --- Global Sensor Flags & Data Variables ---(if sensor not responding the code will skip it and wont stop)
bool BME280_flag = false; // true if sensor found els skip
bool rain_flag = false;
bool Pressurebmp_flag = false;
bool SHT31_flag = false;

// defining parmeters type (int/float)
float BME_Temp = 0, BME_Hum = 0, BME_Pre = 0; 
float SHT_Temp = 0, SHT_Hum = 0;
float BMP_Temp = 0, BMP_Pre = 0;
//int   BMP_Alt = 0;
float rain_1m = 0.0, rain_total = 0.0;


// for altitude measurmants
//#define SEALEVELPRESSURE_HPA (1013.25)

//System monitoring 
#define VBATPIN A7     // define where voltage is measured          
float Feather_V = 0;   // battery voltage, read by Feather          
float NoteCard_V = 0;  // battery voltage, read by Notecard          
float Notecard_temp = 0; // Notecard internal temperature        

//logging file and SD card define
File dataFile; // creating object for file
const char FileName[]="proY&H1.txt";  // file name  
const int chipSelect = 4;   // defining SD port in feathar     
String LoggerHeader = "Time/Date, BME_Temp, BME_Hum, BME_Pre, Rain_1m, Rain_Total, BMP_Pre, BMP_Temp, SHT_Temp, SHT_Hum, NoteCard_Temp, Feather_V, NoteCard_V, delay_q";
String dataString = ""; 

//Adalogger red led
const int RLED_Pin = 13; 

//notehub define
#define productUID "il.ac.bgu.levintal:meteorological_yh" 
//#define myProductID productUID 
Notecard notecard; // create Notecard object

// --- Function Declarations  and order ---
void step1();
void step2();
void step3();
void step4();
void step5();
int getInterval();
unsigned long nc_time();
float Get_NoteCard_Temp();


void setup() {
  Serial.begin(115200); // defining serial
  notecard.setDebugOutputStream(Serial); // conecting notecard to serial
  Wire.begin();  // start I2C protocol
  notecard.begin();  

  pinMode(RLED_Pin, OUTPUT);    // set LED pin
  digitalWrite(RLED_Pin, HIGH);    // turn on LED
  
  //Note card setup (connect to account-poroject)
  {
    J * req = notecard.newRequest("hub.set"); // create hub.set request
    JAddStringToObject(req, "product", productUID);  // set Notehub project ID                                
    JAddStringToObject(req, "mode", "periodic");  // periodic connaction (not always connected)                                      
    JAddStringToObject(req, "vinbound", "usb:2;high:60;normal:120;low:240;dead:0");  // conect to cloud acording to bat mode   
    JAddStringToObject(req, "voutbound", "usb:2;high:30;normal:30;low:60;dead:0");     
    notecard.sendRequestWithRetry(req, 5);  // try 5 time befor moving on
  }
  {
    J * req = notecard.newRequest("card.voltage"); // create card.voltage request
    JAddStringToObject(req, "mode", "lipo"); // define lipo as bat source            
    notecard.sendRequest(req);  // send request
  }
  {
    J * req = notecard.newRequest("note.template"); // define templet ( target box for each parmeter)  
    JAddStringToObject(req, "file", NOTE_FILE_NAME);
    J * body = JAddObjectToObject(req, "body");
    JAddStringToObject(body, "Time", "15:12:24 17/08/2023"); 
    JAddNumberToObject(body, "BME_Temp", 12.1);  // nombers are specific dont chang them
    JAddNumberToObject(body, "BME_Hum", 12.1);  
    JAddNumberToObject(body, "BME_Pre", 14.1);
    JAddNumberToObject(body, "Rain_1m", 12.1);
    JAddNumberToObject(body, "Rain_Total", 12.1);
    JAddNumberToObject(body, "BMP_Pre", 14.1);
    JAddNumberToObject(body, "BMP_Temp", 12.1);
    JAddNumberToObject(body, "SHT_Temp", 12.1);
    JAddNumberToObject(body, "SHT_Hum", 12.1);
    JAddNumberToObject(body, "Notecard_temp", 12.1);
    JAddNumberToObject(body, "Feather_V", 12.1);
    JAddNumberToObject(body, "NoteCard_V", 12.1);
    JAddNumberToObject(body, "delayq", 21);
    notecard.sendRequest(req); 
    Serial.print("built template!,   "); 
  }

  //SD card setup
  delay(10);
  Serial.print(F("Initializing SD card... ")); delay(10); 
  
  if (!SD.begin(chipSelect)) {  // try to init SD card
    Serial.println(F(" SD card error/No card - Skipping SD logging")); 
    delay(100);          
  } else {                                                     
    Serial.println(F(" SD card initialized"));  delay(100);  
    Serial.print("   ,Logging File name: "); Serial.print(FileName);  delay(100);   
    
    dataFile = SD.open(FileName, FILE_WRITE);  // open file    
    if (dataFile) {
      Serial.print(" ,File size: "); Serial.print(dataFile.size()/1024); Serial.println(" KB");   delay(100); 
      if (dataFile.size() == 0){   // if data = 0 than the file is new so headers must ne named
        dataFile.println(LoggerHeader);        
        Serial.print("logging headers:"); Serial.println(LoggerHeader);       
      }
      dataFile.close();
    } else {
      Serial.println(F(" Error opening file on setup"));
    }
  }
  delay(10);

  
  // --- Sensor Setup with Multiplexer ---
  
  #if ENABLE_BME280
    Serial.print("[BME280] ");
    tcaSelect(7); // open port 7 of mux
    delay(10);
    if (!bme.begin(0x77)) { // if sensor doesnt found
      Serial.println("Not detected");
    } else {
      Serial.println("✓ Found and initialized");
      
      // library defanitions for sensor aqurecy
      bme.setSampling(Adafruit_BME280::MODE_FORCED,     //// single reading, then sleep
                      Adafruit_BME280::SAMPLING_X16,    // average 16 samples
                      Adafruit_BME280::SAMPLING_X16,    // 
                      Adafruit_BME280::SAMPLING_X16,    // 
                      Adafruit_BME280::FILTER_X16,     // filter outliers
                      Adafruit_BME280::STANDBY_MS_500); // not relevant but nececery for code to compiled
                      
      BME280_flag = true;
    }
    tcaDisable(); // close ports
  #endif
  
 #if ENABLE_RAIN
    Serial.print("[RAIN] ");
    tcaSelect(0); //open port o
    delay(10);    // 
    
    if (!rain.begin()) {
      Serial.println("Not detected");
    } else {
      Serial.println("✓ Found and initialized");
      rain_flag = true;
    }
    
    tcaDisable(); // close port
  #endif
  
  #if ENABLE_BMP581
    Serial.print("[BMP581] ");
    tcaSelect(6); // opan port 6
    delay(10);
    if (!bmp.begin(0x47, &Wire)) { 
      Serial.println("Not detected");
      delay(200);
    } else {
      Serial.println("✓ Found and initialized");
      bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X); // taking number of fast meassurmantse and take average
      bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_128X);
      bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3); // smooth noise between readings
      bmp.setOutputDataRate((bmp5xx_odr_t)BMP5_ODR_50_HZ);  // 50Hz data rate, not used in single-read mode

      Pressurebmp_flag = true;  
      delay(50); 
    }
    tcaDisable(); // close ports
  #endif

  #if ENABLE_SHT31
    Serial.print("[SHT31] ");
    tcaSelect(3); //opan port 3
    delay(10);
    if (!sht31.begin(0x44)) {
      if (!sht31.begin(0x45)) {
        Serial.println("Not detected");
      } else {
        Serial.println("✓ Found and initialized at 0x45");
        SHT31_flag = true;
      }
    } else {
      Serial.println("✓ Found and initialized at 0x44");
      SHT31_flag = true;
    }
    tcaDisable(); // close port
  #else
    Serial.println("[SHT31] Disabled in configuration");
  #endif
  
  Serial.println("end of setup");
  
  // synchronize to start of minute (Using real time instead of delay)
  DateTime a(nc_time()); // chcks time from notecard
  secq = a.second(); // current seconds
  delayq = 60 - secq; // calcolate time until round minute
  Serial.print("waiting ");       
  Serial.print(delayq);          
  Serial.println(" seconds..."); 

  unsigned long startWait = millis();  // mark start time
  unsigned long waitDuration = delayq * 1000UL; 
  while (millis() - startWait < waitDuration) {  
    yield(); // prevent watchdog reboot while waiting
  }

  DateTime b(nc_time()); // get updated time from Notecard
  DateTime bfixed = b + TimeSpan(0, 3, 0, 0); // fix timezone
  secq = bfixed.second(); // gathering time
  minq = bfixed.minute();
  hourq = bfixed.hour();
  dayq = bfixed.day();
  monthq = bfixed.month(); 
  yearq = bfixed.year(); 
  sprintf (buffer, "%02u:%02u:%02u %02u/%02u/%04u", hourq, minq, secq, dayq, monthq, yearq); 
  Serial.print(buffer);
  
  Serial.println(" - start of sensor loop");
  
  step1();       
  step2();       
  step3();       
  step4();       
  step5();       
  delay(50); 
  digitalWrite(RLED_Pin, LOW);

  Serial.println("end of sensor loop");
}

void step1()  { // chacking system status begore runing
  Feather_V = analogRead(VBATPIN); Feather_V *= 2;  Feather_V *= 3.3; Feather_V /= 1024;    // get V
 
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));  // get Notecard V
  if (rsp != NULL) {
      NoteCard_V = JGetNumber(rsp, "value"); // converting data to number
      notecard.deleteResponse(rsp); // delete after to mange ram memory
  }
  Notecard_temp = Get_NoteCard_Temp(); //get Temp
}

void step2()  { // gathering data from sensores
  // 
  #if ENABLE_BME280
    if (BME280_flag) {
      tcaSelect(7); // open port 7
      delay(50);

      float t = bme.readTemperature(); //get data
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0F;

      BME_Temp = isnan(t) ? -9.0 : t;  // -9 if reading invalid
      BME_Hum  = (isnan(h) || h == 0) ? -9.0 : h; // -9 if invalid or impossible value
      BME_Pre  = (isnan(p) || p == 0) ? -9.0 : p;
      
      tcaDisable(); // close port
    } else {
      BME_Temp = -9.0; BME_Hum = -9.0; BME_Pre = -9.0;
    }
  #endif


  #if ENABLE_RAIN
    if (rain_flag) {
      tcaSelect(0); 
      delay(50);    
      
      float r = rain.getRainfall();
      rain_1m = (r < 0) ? -9.0 : r;
      
      tcaDisable();  
    } else {
      rain_1m = -9.0;
    }
  #endif

  
  #if ENABLE_SHT31
    if (SHT31_flag) {
      tcaSelect(3); 
      delay(50);
      
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();
      SHT_Temp = isnan(t) ? -9.0 : t;
      SHT_Hum  = isnan(h) ? -9.0 : h;
      
      tcaDisable(); 
    } else {
      SHT_Temp = -9.0; SHT_Hum = -9.0;
    }
  #endif


  #if ENABLE_BMP581
    if (Pressurebmp_flag) {
      tcaSelect(6); 
      delay(50);
      
      if (bmp.performReading()) {
        BMP_Temp = bmp.temperature;
        BMP_Pre  = bmp.pressure; 
      } else {
        BMP_Temp = -9.0; BMP_Pre = -9.0;
      }
      
      tcaDisable(); 
    } else {
      BMP_Temp = -9.0; BMP_Pre = -9.0;
    }
  #endif

  Serial.println(" step2 finished");
}

void step3()  { // packing parameters into templet
  dataString = ""; 
  dataString += String(buffer);   
  dataString += ","; dataString += String(BME_Temp);
  dataString += ","; dataString += String(BME_Hum);
  dataString += ","; dataString += String(BME_Pre);
  dataString += ","; dataString += String(rain_1m);
  dataString += ","; dataString += String(rain_total);
  
  
  dataString += ","; dataString += String(BMP_Pre);
  dataString += ","; dataString += String(BMP_Temp);

  dataString += ","; dataString += String(SHT_Temp);
  dataString += ","; dataString += String(SHT_Hum);

  dataString += ","; dataString += String(Notecard_temp);
  dataString += ","; dataString += String(Feather_V);
  dataString += ","; dataString += String(NoteCard_V);
  dataString += ","; dataString += String(delayq);
  Serial.println(LoggerHeader);
  Serial.println(dataString);
}    

void step4()  { // saving data in SD card
  dataFile = SD.open(FileName, FILE_WRITE);      
  delay(50);
  if (dataFile)  {             
    dataFile.println(dataString);
    dataFile.close();
    Serial.print("SD string: ");    
    Serial.print(dataString);          
    Serial.println(" -  Data logged to SD!");
  }
  else {
    Serial.println(F("error with SD card - no SD logging!"));
  }   
}

void step5() { // adding data to queue intil data is sand to notehub
  const char* time = buffer; // pointer to formatted time string
  J *req = notecard.newRequest("note.add"); // create note.add request
  if (req != NULL)  { // check request created ok
    JAddStringToObject(req, "file", NOTE_FILE_NAME); // set target file/queue 
    J *body = JAddObjectToObject(req, "body");  // create body object
    
    // forcing data in case no rain because when parameter is 0 the NOTEHUB woמt show it
    float rain_to_send = rain_1m;
    if (rain_to_send == 0.0) {
        rain_to_send = -1.0; // negative number so wont effect total rain
    }
    if (body)  {
      JAddStringToObject(body, "Time", time);
      JAddNumberToObject(body, "BME_Temp", BME_Temp);
      JAddNumberToObject(body, "BME_Hum", BME_Hum);
      JAddNumberToObject(body, "BME_Pre", BME_Pre);
      JAddNumberToObject(body, "Rain_1m", rain_to_send);        
      JAddNumberToObject(body, "Rain_Total", rain_total);  
      JAddNumberToObject(body, "BMP_Pre", BMP_Pre);        // אין כאן BMP_Alt!
      JAddNumberToObject(body, "BMP_Temp", BMP_Temp);  
      JAddNumberToObject(body, "SHT_Temp", SHT_Temp);
      JAddNumberToObject(body, "SHT_Hum", SHT_Hum);
      JAddNumberToObject(body, "Notecard_temp", Notecard_temp);
      JAddNumberToObject(body, "Feather_V", Feather_V);
      JAddNumberToObject(body, "NoteCard_V", NoteCard_V);
      JAddNumberToObject(body, "delayq", delayq);
    }
    notecard.sendRequest(req); 
  }
}

int getInterval() { // enabeling to chang itrevals from far
  int IntervalSeconds = 120; // seting default
  J *req = notecard.newRequest("env.get"); //reading reques from claude
  if (req != NULL) {
      JAddStringToObject(req, "name", "reading_interval"); // variable name to fetch
      J* rsp = notecard.requestAndResponse(req); // send and get response
      
      if (rsp != NULL) {  // check response received
          if (JHasObjectItem(rsp, "text")) {  // check variable exists
              int readingIntervalEnvVar = atoi(JGetString(rsp, "text")); // parse string to in
              if (readingIntervalEnvVar > 0) { // check variable exists
                  IntervalSeconds = readingIntervalEnvVar; // override default
              }
          }
          notecard.deleteResponse(rsp);  // free response memory
      }
  }
  return IntervalSeconds; // return interval in seconds
}

unsigned long nc_time() { // defing real time
  uint32_t t = 0; // defining object
  J *req, *rsp;

  for (int i = 0; i < 3; i++) {  // creating box in memory for time (3 tryes)
    if ((req = notecard.newRequest("card.time"))) {
      rsp = notecard.requestAndResponse(req);
      if (rsp != NULL) {
        t = JGetNumber(rsp, "time");
        notecard.deleteResponse(rsp); // deleting after sanding to clean ram memory
        if (t > 1600000000) { // cacking if value make sense (after 9/2020)
          return t;
        }
      }
    }
    delay(250); 
  }
  return 0; 
}

float Get_NoteCard_Temp() {  // make surק temp not to high
    float result = 0.0f;    // default value
    J * req = notecard.newRequest("card.temp");  // create  request
    J * rsp = notecard.requestAndResponse(req);   // send and get response

    if (rsp != NULL) {   // check response received
        if (JHasObjectItem(rsp,"value")) {   // check field exists
            J * temp_obj = JGetObjectItem(rsp,"value");   // get value
            if (JIsNumber(temp_obj)) {  // check it's a number
                result = static_cast<float>(JNumberValue(temp_obj));  // convert value to float and save
            }
        }
        notecard.deleteResponse(rsp); // free response memory
    } else {
        Serial.println("Warning: Notecard did not respond to card.temp");
    }
    return result;
}

void loop()  {
  // calculate sleep time and send adalogger to sleep.
Watchdog.reset();  // reset watchdog timer
  int IntervalSeconds = getInterval();
  int TimeOfSensorsWarmingUp = 15;     // warming time
  int IntervalSecondsCorrected = max(IntervalSeconds - TimeOfSensorsWarmingUp, 10);  // min 10 sec to avoid zero/negative sleep
  Serial.print("entering sleep mode for "); Serial.print(IntervalSecondsCorrected); Serial.println(" seconds");
  //delay(100);
  J * req = NoteNewCommand("card.attn"); // create sleep command
  JAddStringToObject(req, "mode", "sleep");  // set mode to sleep
  JAddNumberToObject(req, "seconds", IntervalSecondsCorrected); // set sleep duration
  notecard.sendRequest(req);  // send sleep command
  ::delay(1000);         // Wait 1s for ATTN pin to assert; if sleep triggers we will not reach here  



}