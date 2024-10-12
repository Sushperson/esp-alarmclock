
#include "webserver.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

const char* update_path = "/update/";
const char* user;
const char* pass;

static const char siteIndex[] PROGMEM =
  R"(<!DOCTYPE html>
      <html lang='en'>
      <head>
          <meta charset='utf-8'>
          <meta name='viewport' content='width=device-width,initial-scale=1'/>
		  <style>
				form  { display: table;
						    border-spacing: 5px; }
				p     { display: table-row;  }
				label { display: table-cell; }
				input { display: table-cell; }
		  </style>
      </head>
      <body>
      <form method='POST' id="form" action='' enctype='multipart/form-data'>
          <h3>Settings:</h3>
          <div id="form_elems">
            placeholder
          </div>
          <input type='submit' value='Submit Settings'>
      </form>

      <script type="text/javascript">
        var xmlhttp = new XMLHttpRequest();
        var url = "/getConfig/";

        xmlhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            var settings = JSON.parse(this.responseText);
            myFunction(settings);
          }
        };
        xmlhttp.open("GET", url, true);
        xmlhttp.send();

        function myFunction(settings) {
            var form_str = "";
            for(var key in settings) {
              var type;
              
              if(typeof settings[key] == "number") {
                type = "number";
              } else {
                type = "text";
              }

              form_str += ("<p><label>" + key + ": </label> <input type=\"" + type + "\" name=\"" + key + "\"required value=\"" + settings[key] + "\"></p>");
            }
            document.getElementById("form_elems").innerHTML = form_str;
        }
      </script>

      </body>
      </html>)";

    

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;


String readConfigFile() {
  File configFile = LittleFS.open("config.json", "r");
  String configJsonStr = configFile.readString();
  configFile.close();
  return configJsonStr;
}


void returnConfig() {
  if (!httpServer.authenticate(user, pass)) {
    return httpServer.requestAuthentication();
  }

  httpServer.send(200, "application/json", readConfigFile());
}


void handleRoot() {
  Serial.println("handling request to root");
  if (!httpServer.authenticate(user, pass)) {
    return httpServer.requestAuthentication();
  }
  if (httpServer.method() == HTTP_POST) {
    String configJsonStr = readConfigFile();

    JsonDocument configJson;
    DeserializationError error = deserializeJson(configJson, configJsonStr);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      httpServer.send(500, "text/plain", error.c_str());
      return;
    }


    for (uint8_t i = 0; i < httpServer.args(); i++) {
      String currentArg = httpServer.argName(i);
      if(!configJson[currentArg].isNull()) {
        configJson[currentArg] = httpServer.arg(i);
      }
    }

    //std::string jsonOutputStream();
    serializeJson(configJson, configJsonStr);
    File configFile = LittleFS.open("config.json", "w");
    configFile.print(configJsonStr);
    configFile.close();
  }
  httpServer.send(200, "text/html", siteIndex);
}


void setupWebServer(const char* web_user, const char* web_pass){
  user = web_user;
  pass = web_pass;
  httpUpdater.setup(&httpServer, update_path, user, pass);

  httpServer.on("/", handleRoot);
  httpServer.on("/getConfig/", returnConfig);

  httpServer.begin();
}


void handleWebServer() {
  httpServer.handleClient();
}