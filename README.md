## Solar Powered IOT Thermometer
Paul Beyleveld
paul (dot) beyleveld at gmail (dot) com

Example Thingspeak Channel: https://thingspeak.com/channels/1257024
Author: https://github.com/pbeyl

This is a temperature sensor based on Wemos D1 Mini with DHT22 sensor.
I tried to make it portable and easy including the wifi manager and leveraging Thingspeak MQTT 
there is no requirement on an MQTT broker.
The only requiement is a wifi access point internet connection and a channel on Thingspeak.

The Circuit diagram in the circuit folder offers insight into the electronics I used, these 
electronics have also been installed in a Stevenson shield for more accurate readings.
![Circuit Diagram](https://raw.githubusercontent.com/pbeyl/solar_iot_thermometer/master/circuit/dht22_d1mini_solar_bb.png)

On first boot Wifimanager will start a random wifi access point on the ESP8266 chip. Connect
to this ap and the captive portal will allow the user to configure wifi SSID, password, 
thingspeak mqtt API key, MQTT server, port, channel number and channel Write API Key.

When configuration is completed, the ESP8266 chip will connect to the configured wifi SSID,
once connected it will establish an mqtt session with thingspeak mqtt service.
It will read the values from the DHT22 sensor and publish field 1-6 to the configured channel,
after a successful publish a deep sleep will be triggered for 5 minutes.

## Mapping of the thingspeak channel fields are as follows
* field1 = Temperature in Celsius
* field2 = Temperature in Fahrenheit
* field3 = Humidity
* field4 = Heat Index Calculated in Celsius
* field5 = Battery Level (V)
* field6 = Wifi Signal RSSI (dBm)

It is possible to reset the wifi settings in order to reconfigure the wifi or thingspeak settings,
simply press the D1 mini reset switch twice within a 3 second period will trigger configuration reset
using the DoubleResetDetector library.