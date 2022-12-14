> Based on [BresserWeatherSensorReceiver](https://github.com/matthias-bs/Bresser_Weather_Sensor_CC1101_MQTT)
> which is deprecated according to the author.
> This fork removes a lot of functionality (drops Secure MQTT, deep sleep requiring extra wiring for interrupt),
> while improving the basic functionality and code. Adds a HTTP status page. Uses C1101 interrupt-based reception.

# Bresser_Weather_Sensor_CC1101_MQTT

**Bresser 5-in-1 868 MHz Weather Sensor Radio Receiver based on CC1101 and ESP32/ESP8266 - provides data via MQTT, HTTP**

Based on:
- [Bresser5in1-CC1101](https://github.com/seaniefs/Bresser5in1-CC1101) by [Sean Siford](https://github.com/seaniefs)
- [RadioLib](https://github.com/jgromes/RadioLib) by [Jan Gromeš](https://github.com/jgromes)
- [arduino-mqtt](https://github.com/256dpi/arduino-mqtt) by [Joël Gähwiler (256dpi)](https://github.com/256dpi)
- [ArduinoJson](https://arduinojson.org) by [Benoit Blanchon](https://github.com/bblanchon) 

## Weather Stations

* [BRESSER Weather Center 5-in-1](https://www.bresser.de/en/Weather-Time/Weather-Center/BRESSER-Weather-Center-5-in-1-black.html)
* [BRESSER Professional WIFI colour Weather Center 5-in-1 V](https://www.bresser.de/en/Weather-Time/WLAN-Weather-Stations-Centers/BRESSER-Professional-WIFI-colour-Weather-Center-5-in-1-V.html)

The Bresser 5-in-1 Weather Stations seem to use two different protocols. In the original matthias-bs version, you could select the appropriate decoder by (un-)commenting `#define BRESSER_6_IN_1` in the source code. In this fork of matthias-bs code, however, support has been reduced to just the 5In1.

| Model         | Decoder Function                |
| ------------- | ------------------------------- |
| 7002510..12   | decodeBresser**5In1**Payload()  |
| 7002585       | decodeBresser**6In1**Payload()  |

## MQTT Topics

MQTT publications:

`<base_topic>/data`    sensor data as JSON string - see `publishWeatherdata()`
     
`<base_topic>/radio`   CC1101 radio transceiver info as JSON string - see `publishRadio()`
     
`<base_topic>/status`  "online"|"offline"|"dead"$

Example messages

`pi@raspberrypi:~ $ mosquitto_sub -t "bresserproxy/#" -v`
`bresserproxy/radio {"rssi": -79.500000, "lqi": 17 }`
`bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.2,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}`
`bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.1,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}`
`bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.1,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}`

## HTTP

The ESP serves a very simple text-based status page, e.g., http://192.168.0.100/

## Hardware 

(ESP8266 D1-Mini)
![Bresser5in1_CC1101_D1-Mini](https://user-images.githubusercontent.com/83612361/158458191-b5cabad3-3515-45d0-98e3-94b0fa13b8ef.jpg)

### CC1101

[Texas Instruments CC1101 Product Page](https://www.ti.com/product/CC1101)

**Note: CC1101 Module Connector Pitch is 2.0mm!!!**

Unlike most modules/breakout boards, most (if not all) CC1101 modules sold on common e-commerce platforms have a pitch (distance between pins) of 2.0mm. To connect it to breadboards or jumper wires with 2.54mm/100mil pitch (standard), the following options exist:

* solder wires directly to the module
* use a 2.0mm pin header and make/buy jumper wires with 2.54mm at one end and 2.0mm at the other (e.g. [Adafruit Female-Female 2.54 to 2.0mm Jumper Wires](https://www.adafruit.com/product/1919))
* use a [2.0mm to 2.54 adapter PCB](https://www.amazon.de/Lazmin-1-27MM-2-54MM-Adapter-Platten-Brett-drahtlose-default/dp/B07V873N52)

**Note 2: Make sure to use the 868MHz version!**

## Dashboard with [IoT MQTT Panel](https://snrlab.in/iot/iot-mqtt-panel-user-guide) (Example)
![IoTMQTTPanel_Bresser_5-in-1](https://user-images.githubusercontent.com/83612361/158457786-516467f9-2eec-4726-a9bd-36e9dc9eec5c.png)

