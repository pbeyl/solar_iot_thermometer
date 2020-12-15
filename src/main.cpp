/**
 *
 * Solar Powered IOT Thermometer
 * Paul Beyleveld
 * paul (dot) beyleveld at gmail (dot) com
 * 
 * Example Thingspeak Channel: https://thingspeak.com/channels/1257024
 * Author: https://github.com/pbeyl
 * 
 * This is a temperature sensor based on Wemos D1 Mini with DHT22 sensor.
 * I tried to make it portable and easy including the wifi manager and leveraging Thingspeak MQTT 
 * there is no requirement on an MQTT broker.
 * The only requiement is a wifi access point internet connection and a channel on Thingspeak.
 * 
 * On first boot Wifimanager will start a random wifi access point on the ESP8266 chip. Connect
 * to this ap and the captive portal will allow the user to configure wifi SSID, password, 
 * thingspeak mqtt API key, MQTT server, port, channel number and channel Write API Key.
 * 
 * When configuration is completed, the ESP8266 chip will connect to the configured wifi SSID,
 * once connected it will establish an mqtt session with thingspeak mqtt service.
 * It will read the values from the DHT22 sensor and publish field 1-6 to the configured channel,
 * after a successful publish a deep sleep will be triggered for 5 minutes.
 * 
 * Mapping of the thingspeak channel fields are as follows
 * -------------------------------------------------------
 * field1 = Temperature in Celsius
 * field2 = Temperature in Fahrenheit
 * field3 = Humidity
 * field4 = Heat Index Calculated in Celsius
 * field5 = Battery Level (V)
 * field6 = Wifi Signal RSSI (dBm)
 * 
 * It is possible to reset the wifi settings in order to reconfigure the wifi or thingspeak settings,
 * simply press the D1 mini reset switch twice within a 3 second period will trigger configuration reset
 * using the DoubleResetDetector library.
 * 
 */


#include <FS.h>                   // Include the LittleFS library
#include <LittleFS.h>
#include <ESP8266WiFi.h>          // https://www.arduino.cc/en/Reference/WiFi
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Arduino.h>

#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>

#include <DHT.h>                  // Required to read the temperature and humidity
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson


#define DRD_TIMEOUT 3             // Number of seconds after reset during which a
                                    // subseqent reset will be considered a double reset.
#define DRD_ADDRESS 0             // RTC Memory Address for the DoubleResetDetector to use

#define ESP8266_DRD_USE_RTC     false   //true
#define ESP_DRD_USE_LITTLEFS    true

#include <ESP_DoubleResetDetector.h>  // Allows you to reset the ESP8266 to factory defaults

#define MQTT_USER "user"          //can be anything, thingspeak does not validate user

#define VOLTAGE_PIN A0
#define DHT_PIN D2
#define DHT_TYPE DHT22
#define DEEP_SLEEP_SECONDS 300

struct Config {
  // WifiManager Settings - define default settings
  char MQTT_HOST[24] = "mqtt.thingspeak.com"; // variable to contain NTP server address
  char channelID[8];
  char writeAPIKey[18];                       // write API key for your ThingSpeak Channel
  char MQTT_PASS[18]; 
  char MQTT_PORT[6] = "1883"; 
};

const char *filename = "/config.txt";         // <- Configuration file name
Config config;                                // <- global configuration object

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
Ticker ticker;                                // for led feedback
AsyncWebServer server(80);
DNSServer dns;
DHT dht(DHT_PIN, DHT_TYPE);
//DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);  // Initialize the double reset detector
DoubleResetDetector* drd;

bool sensor_isset = false;                    //flag if dht22 was read successfully
bool shouldSaveConfig = false;                //flag for saving data
bool gotosleep = false;
unsigned long lasPolltime = 0; 
const unsigned long dhtInterval = 2L * 1000L; // read DHT data every 2 seconds.
int try_count = 0;                    //keep track of mqtt tries, if > 10 then sleep before trying again next wake

//sensor value variables
float voltage = 0;
float h = 0;
float t = 0;
float f = 0;
float hif = 0;
float hic = 0;
long rssi;
int wifi_signal;

// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  if (LittleFS.exists(filename)) { 
    // Open file for reading
    File file = LittleFS.open(filename, "r");

    // Allocate a temporary JsonDocument
    // Don't forget to change the capacity to match your requirements.
    // Use arduinojson.org/v6/assistant to compute the capacity.
    StaticJsonDocument<512> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, file);
    if (error)
      Serial.println(F("Failed to read file, using default configuration"));

    // Copy values from the JsonDocument to the Config
    strlcpy(config.MQTT_PORT, doc["MQTT_PORT"], sizeof(config.MQTT_PORT));
    strlcpy(config.MQTT_HOST, doc["MQTT_HOST"], sizeof(config.MQTT_HOST));
    strlcpy(config.MQTT_PASS, doc["MQTT_PASS"], sizeof(config.MQTT_PASS));
    strlcpy(config.channelID, doc["channelID"], sizeof(config.channelID));
    strlcpy(config.writeAPIKey, doc["writeAPIKey"], sizeof(config.writeAPIKey));

    // Close the file (Curiously, File's destructor doesn't close the file)
    file.close();
  } 
}

// Saves the configuration to a file
void saveConfiguration(const char *filename, const Config &config) {
  
  if (LittleFS.exists(filename)) {
    // Delete existing file, otherwise the configuration is appended to the file
    LittleFS.remove(filename);
  }

  // Open file for writing
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc["MQTT_HOST"] = config.MQTT_HOST;
  doc["MQTT_PORT"] = config.MQTT_PORT;
  doc["MQTT_PASS"] = config.MQTT_PASS;
  doc["channelID"] = config.channelID;
  doc["writeAPIKey"] = config.writeAPIKey;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

// Prints the content of a file to the Serial
void printFile(const char *filename) {
  // Open file for reading
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}


void tick()
{
  //toggle state
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (AsyncWiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

/**
 * Report the battery voltage.
 */
float readVoltage()
{
    int voltageCount = analogRead(VOLTAGE_PIN);
    return 0.005080906 * voltageCount;
}

/**
 * Report the Wifi Signal Strenght
 */
int readWifiSignal() {
  rssi = WiFi.RSSI();  // eg. -63

      // Convert to scale -48 to 0 eg. map(rssi, -100, 0, 0, -48);
      // I used -100 instead of -120 because <= -95 is unusable
      // Negative number so we can use percentage
  //return (1.0 - (rssi / -100.0)) * -48.0;
  return rssi;
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Info: Should save config");
  shouldSaveConfig = true;
}


void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  //TODO: add code to read DHT22 sensor values
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  
}

/**
 * Callback function on mqtt conntection
 */
void onMqttConnect(bool sessionPresent) {

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  
  // if we have temperature values then publish to thingspeak.
  if( sensor_isset ) {
      //this is where we publish the values to mqtt

      String data = String("field1=") + String(t, DEC) + "&field2=" + String(f, DEC) + "&field3=" + String(h, DEC) + "&field4=" + String(hic, DEC) + "&field5=" + String(voltage, DEC) + "&field6=" + String(wifi_signal, DEC);
      const char *msgBuffer;
      msgBuffer=data.c_str();
      //Serial.println(msgBuffer);

      // Create a topic string and publish data to ThingSpeak channel feed. 
      String topicString = "channels/" + String( config.channelID ) + "/publish/"+String(config.writeAPIKey);
      const char *topicBuffer;
      topicBuffer = topicString.c_str();
      //Serial.println(topicBuffer);

      mqttClient.publish( topicBuffer, 0, false, msgBuffer );

      Serial.println("Publishing at QoS 0");

      //publish a packet on qos1 otherwise the publish callback is not triggered.
      //uint16_t packetIdPub1 = mqttClient.publish("EOL", 1, false, "x");
      uint16_t packetIdPub1 = mqttClient.publish( topicBuffer, 1, false, "x");
      Serial.print("Publishing at QoS 1, packetId: ");
      Serial.println(packetIdPub1);

  } else {
    mqttClient.disconnect();
    return;
  }

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected() && !gotosleep) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/**
 * this callback will trigger on successful mqtt publish
 */ 
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  
  gotosleep = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  // Initialize SD library
  while (!LittleFS.begin()) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  // Should load default config if run for the first time
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);


  String port = config.MQTT_PORT;
  //setup mqtt client
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setCleanSession(true);   // enable cleansession, required for thingspeak
  mqttClient.setServer(config.MQTT_HOST, port.toInt());
  mqttClient.setCredentials(MQTT_USER, config.MQTT_PASS);

  //register some wifi callbacks
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  //set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VOLTAGE_PIN,INPUT);
  pinMode(DHT_PIN,INPUT_PULLUP);

  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  AsyncWiFiManagerParameter custom_thingspeak("<br>Thingspeak Settings");
  AsyncWiFiManagerParameter custom_mqtt_api_key("MQTT_PASS", "MQTT API Key", config.MQTT_PASS, 18);
  AsyncWiFiManagerParameter custom_mqtt_server("MQTT_HOST", "MQTT Server", config.MQTT_HOST, 24);
  AsyncWiFiManagerParameter custom_mqtt_port("MQTT_PORT", "MQTT Port", config.MQTT_PORT, 6);
  AsyncWiFiManagerParameter custom_chanel_id("channelID", "Channel ID", config.channelID, 8);
  AsyncWiFiManagerParameter custom_write_api_key("writeAPIKey", "Channel Write API Key", config.writeAPIKey, 18);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  AsyncWiFiManager mywifiManager(&server,&dns);
  //reset settings - for testing
  //wifiManager.resetSettings();

  // disable debug logging for WifiManager
  mywifiManager.setDebugOutput(false);

  //detect double reset and reset settings
   if (drd->detectDoubleReset()) {
    Serial.println("INFO: Double Reset Detected - loading factory defauls");

    ticker.detach();

    //clean FS, for testing
    //LittleFS.format();
    mywifiManager.resetSettings();
    //if (LittleFS.exists(filename)) {
    //  LittleFS.remove(filename);
    //}

    for (int counter = 0; counter < 20; counter++) {
      Serial.print(".");
      delay(100);
    }
    Serial.print("\n");

    ESP.reset();

  }

  //set config save notify callback
  mywifiManager.setSaveConfigCallback(saveConfigCallback);

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  mywifiManager.setAPCallback(configModeCallback);

  //add all wifimanager custom parameters here
  mywifiManager.addParameter(&custom_thingspeak);
  mywifiManager.addParameter(&custom_mqtt_api_key);
  mywifiManager.addParameter(&custom_mqtt_server);
  mywifiManager.addParameter(&custom_mqtt_port);
  mywifiManager.addParameter(&custom_chanel_id);
  mywifiManager.addParameter(&custom_write_api_key);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!mywifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("Wifi connected...yeey :)");
  ticker.detach();

  //keep LED on
  digitalWrite(LED_BUILTIN, LOW);

  if (shouldSaveConfig) {
      //read updated parameters
      strcpy(config.channelID, custom_chanel_id.getValue());
      strcpy(config.writeAPIKey, custom_write_api_key.getValue());
      strcpy(config.MQTT_HOST, custom_mqtt_server.getValue());
      strcpy(config.MQTT_PASS, custom_mqtt_api_key.getValue());
      strcpy(config.MQTT_PORT, custom_mqtt_port.getValue());

      // Create configuration file
      Serial.print(F("Saving configuration... "));
      saveConfiguration(filename, config);
      Serial.println("Done!");
  }

  //printFile(filename);

  // start the dht sensor
  dht.begin();

  //trigger mqtt connect if not connected
  //if (WiFi.isConnected() && !mqttClient.connected()) {
  //  mqttReconnectTimer.once(2, connectToMqtt);
  //}

}

void loop() {

  // Only read DHT sensor every few seconds, this method is non blocking.
  if (millis() - lasPolltime > dhtInterval) 
  {
    // check if we need to deep sleep, goto sleep if failed > 10 times
    // failures will most likely be related to wifi/connectivity issues
    if ( gotosleep || try_count >= 10 ) {
      Serial.println("Entering Deep Sleep...");
      drd->stop();    //ensure drd is off
      ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000000);
      delay(1000); // allow deep sleep to occur
    }

    drd->loop();
    Serial.println("Read DHT Sensor");
  
    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;

    } else {

      // Compute heat index in Fahrenheit (the default)
      hif = dht.computeHeatIndex(f, h);
      // Compute heat index in Celsius (isFahreheit = false)
      hic = dht.computeHeatIndex(t, h, false);

      voltage = readVoltage();
      wifi_signal = readWifiSignal();

      Serial.print(F("Wifi Signal: "));
      Serial.print(wifi_signal);
      Serial.print(F("dBm Voltage: "));
      Serial.print(voltage);
      Serial.print(F("V Humidity: "));
      Serial.print(h);
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.print(F("°C "));
      Serial.print(f);
      Serial.print(F("°F  Heat index: "));
      Serial.print(hic);
      Serial.print(F("°C "));
      Serial.print(hif);
      Serial.println(F("°F"));


      sensor_isset = true;
    }

    try_count++;
    lasPolltime = millis();   //set the last completed run time
  }
  

}
