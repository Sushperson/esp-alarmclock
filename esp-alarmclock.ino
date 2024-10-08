

//#include <RtcDS1302.h>

#include <functional>

#include <FS.h>
#include <LittleFS.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <time.h>

#include <ArduinoJson.h>

#include <SPI.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include "Font_tight.h"

#include <OneWire.h>
#include <DallasTemperature.h>

/**
 * calculate the length of an array in terms of elements
 * this apparently cannot really be done with a function...
 */
#define countof(a) (sizeof(a) / sizeof(a[0]))

// some definitions for the temp sensor
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// array to hold device address
DeviceAddress therm;


// Define matrix display hardware type, size, and output pins:
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CS_PIN 15

// Create a new instance of the MD_Parola class with hardware SPI connection:
MD_Parola ledMatrix = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Instances for the RTC
// ThreeWire myWire(4, 5, 0);  // IO, SCLK, CE
// RtcDS1302<ThreeWire> Rtc(myWire);

// Define input pins
struct Button {
  const uint8_t PIN;
  unsigned long debounceTimer;
  bool pressed;
};

Button r_button = {0, 0, false};
Button l_button = {4, 0, false};
// Button renc_button = {12, 0, false};

//Setup rotary encoder
struct RotaryEncoder {
  const uint8_t CLK_PIN;
  const uint8_t INP_PIN;
  unsigned long debounceTimer;
  int8_t rotation;
};

RotaryEncoder renc = {5, 16, 0};

// Function to set the NTP polling interval to more than 1h, apparently...
uint32_t sntp_update_delay_MS_rfc_not_less_than_15000() {
  return 12 * 60 * 60 * 1000UL;  // 12 hours
}

// Global time variables.
// TODO: evaluate if these have to be global
time_t now;  // these are the seconds since Epoch (1970) - UTC
tm tm;       // the structure tm holds time information in a more convenient way

// Global config an state-variables
struct SystemConfiguration {
  const char* wifi_ssid;
  const char* wifi_password;
  const char* weather_api_key;
  float lat;
  float lon;
  const char* timezone;
};
SystemConfiguration config;
uint8_t menuLevel = 0;

struct MenuItem {
  const char* title;
  std::function<void()> setValFctn;
};

/**
 * Structure to hold information about different modes for the clock's main display
 */
struct DisplayMode {
  std::function<void()> updateFctn;
  int updateInterval;
  //Constructor that enables initializing the object after declaration
  DisplayMode(std::function<void()> fctn, int updtInt): updateFctn(fctn), updateInterval(updtInt) {}
  //Default constructor that just sets the mode to be a time-display mode.
  DisplayMode(): updateFctn(displayCurrentTime), updateInterval(1000) {}
};
DisplayMode dispModes[3];
uint8_t dispMode = 0;

/**
 * Connect to wifi using specified ssid and password
 */
void setupWifi(const char* ssid, const char* password) {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


/**
 * Setup the led-Matrix Display
 */
void setupDisplay() {
  // Intialize the object:
  ledMatrix.begin();
  // Set the intensity (brightness) of the display (0-15):
  ledMatrix.setIntensity(5);
  // Clear the display:
  ledMatrix.displayClear();
  // Set the font
  ledMatrix.setFont(tightFont);
}


/**
 * Setup the thermometer
 */
void setupThermometer() {
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // assigns the first address found to therm
  if (!sensors.getAddress(therm, 0)) Serial.println("Unable to find address for Device 0");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(therm);
  Serial.println();

  // set the resolution to 9 bit for fastest possible reading with 0.5° pecision
  // (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(therm, 9);
}


/**
 * Show ambient temprerature as measured by the onboard
 * temp sensor on the led matrix display
 */
void displayInsideTemperature() {
  Serial.end();
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(therm);
  ledMatrix.setTextAlignment(PA_CENTER);
  if (tempC == DEVICE_DISCONNECTED_C) {
    ledMatrix.print("Temp Err.");
    return;
  }
  ledMatrix.printf("%.1f\x90\x43", tempC);
  Serial.begin(115200);
}


/**
 * Show outside temprerature at the spcified location
 * on the led matrix display
 */
void displayOutsideTemperature(float lat, float lon) {
  ledMatrix.setTextAlignment(PA_CENTER);
  //Display the temp by converting the Kelvin-value to °C
  ledMatrix.printf("%.1f\x90\x43", getOutsideTemp(lat, lon) - 273.15);
}


/**
 * Get outside temperature from the openweathermap-API
 * @return Temperature in Kelvin, or -1.0 if an error occurred.
 */
float getOutsideTemp(float lat, float lon) {
  WiFiClient client;
  HTTPClient http;
  char url[160];
  snprintf(url, 160, "http://api.openweathermap.org/data/2.5/weather?lat=%f&lon=%f&appid=%s", lat, lon, config.weather_api_key);

  float returnVal = -1.0;
  if (http.begin(client, url)) {  // HTTP
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      // Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String payload = http.getString();
        JsonDocument weatherResp;
        DeserializationError error = deserializeJson(weatherResp, payload);
        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
        } else {
          returnVal = weatherResp["main"]["temp"];
        }
      }
    } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  return returnVal;
}


/**
 * Display the current time off the local clock
 * with or without seconds on the led matrix display 
 */
void displayCurrentTime() {
  getLocalTime(&tm, 5000);
  ledMatrix.setTextAlignment(PA_LEFT);
  if (tm.tm_hour >= 20) {
    ledMatrix.printf("\x95%d:%02d:%02d", tm.tm_hour % 10, tm.tm_min, tm.tm_sec);
  } else {
    ledMatrix.printf("%2d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
  }
}


void setupTime() {
  //Configure Timezone and servers
  configTime(config.timezone, "de.pool.ntp.org", "0.pool.ntp.org", "1.pool.ntp.org");
  delay(2000);

  // Rtc.Begin();

  // if (Rtc.GetIsWriteProtected()) {
  //   Serial.println("RTC was write protected, enabling writing now");
  //   Rtc.SetIsWriteProtected(false);
  // }

  // Serial.println("Setting RTC to fetched Time");
  // RtcDateTime rtcNow = RtcDateTime();
  // time(&now);
  // rtcNow.InitWithUnix64Time(now);
  // Rtc.SetDateTime(rtcNow);

  // if (!Rtc.GetIsRunning()) {
  //   Serial.println("RTC was not actively running, starting now");
  //   Rtc.SetIsRunning(true);
  // }
}


/**
 * read config-options from the configfile in the FS
 */
void readConfig() {
  Serial.println("Mount LittleFS");
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }

  //Read config file and deserialize json
  File configFile = LittleFS.open("config.json", "r");
  String configJsonStr = configFile.readString();
  Serial.println(configJsonStr);
  configFile.close();

  JsonDocument jsonConfig;

  DeserializationError error = deserializeJson(jsonConfig, configJsonStr);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  //Copy the char* from the JsonDocument to new static char*
  static char ssid[50];
  strcpy(ssid, jsonConfig["ssid"].as<const char*>());
  static char password[50];
  strcpy(password, jsonConfig["password"].as<const char*>());
  static char weather_api_key[50];
  strcpy(weather_api_key, jsonConfig["weather_api_key"].as<const char*>());
  static char timezone[50];
  strcpy(timezone, jsonConfig["timezone"].as<const char*>());

  //Now inizialize the config structs with our new static char*
  config = {
    ssid,
    password,
    weather_api_key,
    jsonConfig["lat"],
    jsonConfig["lon"],
    timezone
  };
}


/**
 * Setup the different modes for the main clock display
 */
void setupDisplayModes() {
  static auto dispOutsideTempLambda = [=]() {
    return displayOutsideTemperature(config.lat, config.lon);
  };

  dispModes[0] = {displayCurrentTime, 1000};
  dispModes[1] = {displayInsideTemperature, 10000};
  dispModes[2] = {dispOutsideTempLambda, 60000};
}


/**
 * Setup the Input/Interrupt Pins
 */
void setupInputs() {
  pinMode(r_button.PIN, INPUT_PULLUP);
  pinMode(l_button.PIN, INPUT_PULLUP);
  pinMode(renc.CLK_PIN, INPUT);
  pinMode(renc.INP_PIN, INPUT);
  //pinMode(renc_button.PIN, INPUT_PULLUP);
  attachInterrupt(r_button.PIN, rBtnDown, FALLING);
  attachInterrupt(l_button.PIN, lBtnDown, FALLING);
  attachInterrupt(renc.CLK_PIN, rencTurned, RISING);
}


////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

bool interrupted = false;

ICACHE_RAM_ATTR void rBtnDown(){
  unsigned long now = millis();
  if(now - r_button.debounceTimer > 250){
    r_button.pressed = true;
    r_button.debounceTimer = now;
    interrupted = true;
  }
}


ICACHE_RAM_ATTR void lBtnDown(){
  unsigned long now = millis();
  if(now - l_button.debounceTimer > 250){
    l_button.pressed = true;
    l_button.debounceTimer = now;
    interrupted = true;
  }
}


ICACHE_RAM_ATTR void rencTurned(){
  unsigned long now = millis();
  if(now - renc.debounceTimer > 50){
    if(digitalRead(renc.CLK_PIN) == digitalRead(renc.INP_PIN)){
      renc.rotation = -1;
    } else {
      renc.rotation = 1;
    }
    renc.debounceTimer = now;
    interrupted = true;
  }
}

/////////////////////////
// MAIN SETUP AND LOOP //
/////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.println();

  //Read config file
  readConfig();

  //Setup the display modes
  setupDisplayModes();

  //Start Wifi
  setupWifi(config.wifi_ssid, config.wifi_password);

  //Setup Time stuff
  setupTime();

  //Set esp-time after RTC-time
  // time_t epoch_t = Rtc.GetDateTime().Unix64Time();
  // timeval tv = { epoch_t, 0 };
  // settimeofday(&tv, nullptr);

  //Setup Temp sensor
  setupThermometer();

  //Setup the Matrix Display
  setupDisplay();

  //Setup user input pins
  setupInputs();
}


unsigned long lastSerialPrint = 0;
unsigned long lastUpdate = 0;

void loop() {
  unsigned long now = millis();

  //Update display according to current displayMode
  if (now - lastUpdate > dispModes[dispMode].updateInterval) {
    lastUpdate = now;
    dispModes[dispMode].updateFctn();
  }
  
  //If the system was interrupted, handle the input received.
  if(interrupted) {
    Serial.println("An interrupt seems to have occurred!");

    // Determine the input and handle it.
    if(r_button.pressed) {
      Serial.println("handling r button press");
      dispMode = (dispMode + 1) % countof(dispModes);
      r_button.pressed = false;
    } 
    if(l_button.pressed) {
      Serial.println("handling l button press");
      dispMode = positive_modulo(dispMode - 1, countof(dispModes));
      l_button.pressed = false;
    }
    if(renc.rotation != 0) {
      Serial.print("Handling Rotary Input, reporting: ");
      Serial.println(renc.rotation);
      dispMode = positive_modulo(dispMode + renc.rotation, countof(dispModes));
      renc.rotation = 0;
    }

    Serial.printf("Handled Interrupt. Current mode: %d\n", dispMode);
    // Clear interrupt flag and update display.
    interrupted = false;
    lastUpdate = now;
    dispModes[dispMode].updateFctn();
  }


  // if (now - lastSerialPrint > 10000 - 10) {
  //   lastSerialPrint = now;
  //   serialPrintTimes();
  // }

  delay(10);
}


////////////////////////////
// Small helper functions //
////////////////////////////

inline int positive_modulo(int i, int n) {
  return (i % n + n) % n;
}


/////////////////////////////
// Functions for debugging //
/////////////////////////////

void serialPrintTimes() {
  // RtcDateTime RtcNow = Rtc.GetDateTime();

  // Serial.print("Rtc Time: ");
  // printDateTime(RtcNow);
  // Serial.println();

  getLocalTime(&tm, 5000);
  Serial.printf("Local Time : %d-%02d-%02d %02d:%02d:%02d\n", (tm.tm_year) + 1900, (tm.tm_mon) + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}



// void printDateTime(const RtcDateTime& dt) {
//   char datestring[26];

//   snprintf_P(datestring,
//              countof(datestring),
//              PSTR("%02u.%02u.%04u %02u:%02u:%02u"),
//              dt.Day(),
//              dt.Month(),
//              dt.Year(),
//              dt.Hour(),
//              dt.Minute(),
//              dt.Second());
//   Serial.print(datestring);
// }

/**
 * Print a single temperature sensor's temperature reading
 */
void serialPrintTemperature(DeviceAddress deviceAddress) {
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.println(tempC);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
