

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

#include "webserver.h"


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
#define CS_PIN D6

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
  int rotation;
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

// Global config set in config file
// cannot be changed on the clock directly
struct SystemConfiguration {
  const char* wifi_ssid;
  const char* wifi_password;
  const char* wifi_hostname;
  const char* web_user;
  const char* web_pass;
  const char* weather_api_key;
  float lat;
  float lon;
  const char* timezone;
};
SystemConfiguration config;

// Configuration for the clock operation
// can be changed on the clock directly
struct OpConfiguration {
  bool displaySeconds;
  int displayBrightness;
  bool wifiConnection;
};
OpConfiguration opConfig = {true, 5, false};

struct MenuItem {
  const char* title;
  std::function<bool()> setValFctn;
};
MenuItem menuItems[2];

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


/////////////////////
// Setup Functions //
/////////////////////


/**
 * Connect to wifi using specified ssid and password
 */
bool setupWifi(const char* ssid, const char* password) {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.hostname(config.wifi_hostname);
  WiFi.begin(ssid, password);

  if ((WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
    // delay(500);
    // Serial.print(".");
    Serial.println("Connection failed!");
    WiFi.mode(WIFI_OFF);
    return false;
  } else {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  }
}


void setupAP(){
  const char ssid[] = "ESP-Clock";

  Serial.println();
  Serial.print("Setting up AP as");
  Serial.println(ssid);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
}


void setupTime() {
  //Configure Timezone and servers
  configTime(config.timezone, "de.pool.ntp.org", "0.pool.ntp.org", "1.pool.ntp.org");
  delay(2000);

  if(!opConfig.wifiConnection){
    struct tm timeinfo;
    timeinfo.tm_year = 2024 - 1900; // Year since 1900
    timeinfo.tm_mon = 10 - 1;        // Month (0-11)
    timeinfo.tm_mday = 12;            // Day of the month (1-31)
    timeinfo.tm_hour = 1;           // Hour (0-23)
    timeinfo.tm_min = 32;             // Minute (0-59)
    timeinfo.tm_sec = 0;             // Second (0-59)

    // Convert to time_t
    time_t t = mktime(&timeinfo);
    struct timeval tv = {t, 0};
    // tv.tv_sec = t; // Set this to the desired epoch time (seconds since Jan 1, 1970)
    // tv.tv_usec = 0;         // Microseconds
    // Set the time
    settimeofday(&tv, NULL);
  }

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
  strcpy(ssid, jsonConfig["wifi-ssid"].as<const char*>());
  static char password[50];
  strcpy(password, jsonConfig["wifi-password"].as<const char*>());
  static char hostname[50];
  strcpy(hostname, jsonConfig["hostname"].as<const char*>());
  static char web_user[50];
  strcpy(web_user, jsonConfig["web-username"].as<const char*>());
  static char web_pass[50];
  strcpy(web_pass, jsonConfig["web-password"].as<const char*>());
  static char weather_api_key[50];
  strcpy(weather_api_key, jsonConfig["weather_api_key"].as<const char*>());
  static char timezone[50];
  strcpy(timezone, jsonConfig["timezone"].as<const char*>());

  //Now inizialize the config structs with our new static char*
  config = {
    ssid,
    password,
    hostname,
    web_user,
    web_pass,
    weather_api_key,
    jsonConfig["latitude"],
    jsonConfig["longitude"],
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
 * Setup the different menu items to configure settings
 */
void setupMenuItems() {
  //Set Brightness
  static auto setBrightnessLambda = [&]() {
    bool callerStayInLoop = setIntegerConfigVal(&opConfig.displayBrightness, 0, 16);
    ledMatrix.setIntensity(opConfig.displayBrightness);
    return callerStayInLoop;
  };
  menuItems[0] = {"Bright", setBrightnessLambda};

  //Set wether to display seconds
  static auto setDisplaySecondsLambda = [&]() {
    return setBooleanConfigVal(&opConfig.displaySeconds);
  };
  menuItems[1] = {"disp secs", setDisplaySecondsLambda};


}


/**
 * Setup the led-Matrix Display
 */
void setupDisplay() {
  Serial.println("Settings up matrix display");
  // Intialize the object:
  ledMatrix.begin();
  // Set the intensity (brightness) of the display (0-15):
  ledMatrix.setIntensity(opConfig.displayBrightness);
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
  Serial.print("Locating OneWire-Bus devices...");
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

  Serial.println();

  // set the resolution to 9 bit for fastest possible reading with 0.5° pecision
  // (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(therm, 9);
}


/**
 * Setup the Input/Interrupt Pins
 */
void setupInputs() {
  Serial.println("Setting up input pins");
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

IRAM_ATTR void rBtnDown(){
  unsigned long now = millis();
  if(now - r_button.debounceTimer > 250){
    r_button.pressed = true;
    r_button.debounceTimer = now;
    interrupted = true;
  }
}


IRAM_ATTR void lBtnDown(){
  unsigned long now = millis();
  if(now - l_button.debounceTimer > 250){
    l_button.pressed = true;
    l_button.debounceTimer = now;
    interrupted = true;
  }
}


IRAM_ATTR void rencTurned(){
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


///////////////////////////////////
// General and Display Functions //
///////////////////////////////////



// Idea for some kind of more abstact inputbehaviour to reduce 
// redundancies between the different functions that handle input in some way.
// It turned out to be something really kinda whaky...
template <typename T, typename F1, typename F2, typename F3, typename F4>
bool inputBehaviour(T* pVal, F1 rencHandler, F2 rBtnHandler, F3 lBtnHandler, F4 displayFctn, bool callerStaysInLoop) {
  interrupted = false;
  T currentVal = *pVal;

  ledMatrix.setTextAlignment(PA_CENTER);
  ledMatrix.print(displayFctn(currentVal));

  bool stayInLoop = true;
  while(stayInLoop) {
    //Wait for user input, in which case an interrupt will have occurred
    if(interrupted) {
      // Determine the input and handle it.
      if(r_button.pressed) {
        r_button.pressed = false;
        stayInLoop = rBtnHandler(pVal, currentVal);
      }
      if(l_button.pressed) {
        l_button.pressed = false;
        stayInLoop = lBtnHandler(pVal, currentVal);
      }
      if(renc.rotation != 0) {
        currentVal = rencHandler(currentVal, renc.rotation);
        renc.rotation = 0;
      }
      // Clear interrupt flag and update display.
      interrupted = false;
      ledMatrix.print(displayFctn(currentVal));
    }
    delay(20);
  }
  interrupted = false;
  return callerStaysInLoop;
}


/**
 * display a screen for setting an arbitrary config-value
 * using the rotary encoder
 */
template <typename T, typename F1, typename F2>
T setConfigVal(T* pVal, F1 rencHandler, F2 displayFctn) {
  auto doNothing = [](T* pVal, T currentVal) {
    return false;
  };
  auto setpVal = [](T* pVal, T currentVal) {
    *pVal = currentVal;
    return false;
  };

  return inputBehaviour(pVal, rencHandler, setpVal, doNothing, displayFctn, true);
}


/**
 * display menu and let user scroll through items
 * using the rotary encoder
 */
bool displayMenu() {
  auto selectConfigItem = [&](int* pVal, int currentItem){
    return menuItems[currentItem].setValFctn();
  };
  auto doNothing = [](int* pVal, int currentVal) {
    return false;
  };
  auto displayMenuItem = [&](int currentItem){
    return menuItems[currentItem].title;
  };
  auto rencHandler = [&](int currentItem, int rotation){
    return positive_modulo(currentItem + rotation, countof(menuItems));
  };

  int val = 0;
  return inputBehaviour(&val, rencHandler, selectConfigItem, doNothing, displayMenuItem, true);
}


/**
 * display menu and let user scroll through items
 * using the rotary encoder
 */
// void displayMenu() {
//   interrupted = false;
//   int currentItem = 0;

//   ledMatrix.setTextAlignment(PA_CENTER);
//   ledMatrix.print(menuItems[currentItem].title);

//   while(true) {
//     //Wait for user input, in which case an interrupt will have occurred
//     if(interrupted) {
//       // Determine the input and handle it.
//       if(r_button.pressed) {
//         r_button.pressed = false;
//         menuItems[currentItem].setValFctn();
//       } 
//       if(l_button.pressed) {
//         l_button.pressed = false;
//         break;
//       }
//       if(renc.rotation != 0) {
//         currentItem = positive_modulo(currentItem + renc.rotation, countof(menuItems));
//         renc.rotation = 0;
//       }
//       // Clear interrupt flag and update display.
//       interrupted = false;
//       ledMatrix.print(menuItems[currentItem].title);
//     }
//     delay(10);
//   }
//   interrupted = false;
// }



/**
 * special version of setConfigVal for integer values
 * specify lower and upper bound for integer value instead
 * of rotary encoder input handler function
 */
bool setIntegerConfigVal(int* pVal, int lower, int upper) {
  auto intRencHandler = [=](int currentVal, int rotation) {
    return std::clamp(currentVal + rotation, lower, upper);
  };
  auto id = [](int val) {
    return val;
  };
  return setConfigVal(pVal, intRencHandler, id);
}


/**
 * special version of setConfigVal for boolean values
 * only the initial boolean value is required.
 */
bool setBooleanConfigVal(bool *pVal) {
  auto boolRencHandler = [=](bool currentVal, int rotation) {
    return !currentVal;
  };
  auto displayBoolean = [](bool val) {
    if(val){
      return "on";
    } else {
      return "off";
    }
  };
  return setConfigVal(pVal, boolRencHandler, displayBoolean);
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
  struct tm tm_local;
  struct tm* tm = &tm_local;
  if(opConfig.wifiConnection) {
    getLocalTime(tm, 5000);
  } else {
    time_t now = time(NULL);
    tm = localtime(&now);
  }

  if(opConfig.displaySeconds){
    ledMatrix.setTextAlignment(PA_LEFT);
    if (tm->tm_hour >= 20) {
      ledMatrix.printf("\x95%d:%02d:%02d", tm->tm_hour % 10, tm->tm_min, tm->tm_sec);
    } else {
      ledMatrix.printf("%2d:%02d:%02d", tm->tm_hour, tm->tm_min, tm->tm_sec);
    }
  } else {
    ledMatrix.setTextAlignment(PA_CENTER);
    ledMatrix.printf("%2d:%02d", tm->tm_hour, tm->tm_min);
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

  //Setup the config menu items
  setupMenuItems();

  //Try to connect to Wifi
  opConfig.wifiConnection = setupWifi(config.wifi_ssid, config.wifi_password);

  if(!opConfig.wifiConnection){
    setupAP();
  }

  //Setup Time stuff
  setupTime();

  //Setup the webserver for config and OTA updates
  setupWebServer(config.web_user, config.web_pass);

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
      r_button.pressed = false;
      displayMenu();
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


  if (now - lastSerialPrint > 2000 - 10) {
    lastSerialPrint = now;
    serialPrintAnalogReading();
    system_print_meminfo();
  }

  handleWebServer();

  delay(20);
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
  struct tm* tm;

  getLocalTime(tm, 5000);
  Serial.printf("Local Time : %d-%02d-%02d %02d:%02d:%02d\n", (tm->tm_year) + 1900, (tm->tm_mon) + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
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

void serialPrintAnalogReading() {
  Serial.println(analogRead(A0));
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
