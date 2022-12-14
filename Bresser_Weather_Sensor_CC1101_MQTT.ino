////////////////////////////////////////////////////////////////////////////////////////////////
// Bresser_Weather_Sensor_CC1101_MQTT.ino
//
// Bresser 5-in-1 868 MHz Weather Sensor Radio Receiver based on CC1101 and ESP32/ESP8266 -
// provides data via unsecure MQTT
//
// Modified from original project
// Original project:
//    https://github.com/matthias-bs/Bresser_Weather_Sensor_CC1101_MQTT
// Based on:
//    Bresser5in1-CC1101 by Sean Siford (https://github.com/seaniefs/Bresser5in1-CC1101)
//    RadioLib by Jan Gromeš (https://github.com/jgromes/RadioLib)
//    arduino-mqtt Joël Gähwiler (256dpi) (https://github.com/256dpi/arduino-mqtt)
//    ArduinoJson by Benoit Blanchon (https://arduinojson.org)
//
// MQTT publications:
//     <base_topic>/data    sensor data as JSON string - see publishWeatherdata()
//     <base_topic>/radio   CC1101 radion transceiver info as JSON string - see publishRadio()
//     <base_topic>/status  "online"|"offline"|"dead"$
//
//   Example messages
//     pi@raspberrypi:~ $ mosquitto_sub -t "bresserproxy/#" -v
//     bresserproxy/radio {"rssi": -79.500000, "lqi": 17 }
//     bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.2,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}
//     bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.1,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}
//     bresserproxy/data {"sensor_id": "F9","battery_ok":1,"temp_c":6.1,"humidity":98,"wind_gust":0.0,"wind_avg":0.0,"wind_dir":225.0,"rain":81.6}
//     ...
//
// Cabling CC1101 to ESP as per https://docs.openmqttgateway.com/setitup/rf.html#pinout
//   Board   Receiver Pin(GDO2)  Emitter Pin(GDO0)   SCK   VCC   MOSI  MISO  CSN   GND
//   ESP8266   D2/D3/D1/D8            RX/D2          D5    3V3   D7    D6    D8    GND
//   ESP32         D27                 D12           D18   3V3   D23   D19   D5    GND
//
// Notes:
// (- To enable wakeup from deep sleep on ESP8266, GPIO16 (D0) must be connected to RST!
//    Add a jumper to remove this connection for programming!) -- disabled for now, JanW.
//
// ChangeLog wrt Matthias BS project:
// - removed ESP32 for now
// - removed HTTPS MQTT, back to unsecure traffic..
// - removed deep sleep for now...
// - restructured and tidied code, cut down to Bresser 5-in-1 only
// - revised setup() and loop() to be more dynamic
// - switched CC1101 rx to use an interrupt
// - added simple HTTP with an info dump page that does not look good yet
//
////////////////////////////////////////////////////////////////////////////////////////////////

// User specific options
#define MQTT_PAYLOAD_SIZE 200     // maximum MQTT message size
#define STATUS_INTERVAL   30000   // MQTT status message interval [ms]
//  unused:
#define SLEEP_EN          false   // enable sleep mode (see notes above!)
#define AWAKE_TIMEOUT     300000  // maximum time until sketch is forced to sleep [ms]
#define SLEEP_INTERVAL    10000   // sleep interval [ms]

// Board wiring to CC1101
#if defined(ESP32)
    #define PIN_CC1101_CS   5
    #define PIN_CC1101_GDO0 27
    #define PIN_CC1101_GDO2 4
#elif defined(ESP8266)
    #define PIN_CC1101_CS   15  // D8 = gpio15 =  CS
    #define PIN_CC1101_GDO0  0  // D3 = gpio0
    #define PIN_CC1101_GDO2  4  // D2 = gpio4
#endif

// Firmware behaviour
#define USE_RX_INTERRUPT    // Enable interrupt-driven reception rather than default polling
#define DEBUG_DUMP_C1101    // Be more verbose in Serial about C1101 e.g. do hex dump of received data
#define LED_EN              // Enable LED flash indicating successful data reception
#define LED_GPIO 2          // -"- pin to use
//#define FAKE_WX_DATA      // Enable to debug MQTT connection; will generate synthetic sensor data

////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#define RADIOLIB_BUILD_ARDUINO
#include <RadioLib.h>

#if defined(ESP32)
    #include <WiFi.h>
#elif defined(ESP8266)
    #include <ESP8266WiFi.h>
#endif

#include <PubSubClient.h>
  #include <ESP8266WebServer.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SECRET
    const char ssid[] = "your-wifi-ssid";
    const char pass[] = "your-wifi-passwd";

    #define MY_HOSTNAME "bresserproxy"
    #define MY_HTTP_PORT 80

    const char MQTT_HOST[] = "192.168.0.185"; // destination for pushing MQTT
    const int  MQTT_PORT   = 1883; // 1883 default, or 8883 if net is (disabled for now) WiFiClientSecure
    const char MQTT_USER[] = "";   // leave blank if no credentials used
    const char MQTT_PASS[] = "";   // leave blank if no credentials used
#endif

// MQTT topics
const char MQTT_PUB_STATUS[] = MY_HOSTNAME "/status";
const char MQTT_PUB_RADIO[]  = MY_HOSTNAME "/radio";
const char MQTT_PUB_DATA[]   = MY_HOSTNAME "/data";

////////////////////////////////////////////////////////////////////////////////////////////////

// Decoder return codes
typedef enum DecodeStatus_E {
    DECODE_OK, DECODE_PAR_ERR, DECODE_CHK_ERR, DECODE_DIG_ERR
} DecodeStatus;

// Weather data container
typedef struct WeatherData_S {
    uint8_t  s_type;               // only 6-in1
    uint32_t sensor_id;            // 5-in-1: 1 byte / 6-in-1: 4 bytes
    uint8_t  chan;                 // only 6-in-1
    bool     temp_ok;              // only 6-in-1
    float    temp_c;
    int      humidity;
    bool     uv_ok;                // only 6-in-1
    float    uv;                   // only 6-in-1
    bool     wind_ok;              // only 6-in-1
    float    wind_direction_deg;
    float    wind_gust_meter_sec;
    float    wind_avg_meter_sec;
    bool     rain_ok;              // only 6-in-1
    float    rain_mm;
    bool     battery_ok;
    bool     moisture_ok;          // only 6-in-1
    int      moisture;             // only 6-in-1
    // Local:
    bool     valid;                // 'true' if at least one of above fields is ok
    uint32_t timestamp;
} WeatherData_t;

WeatherData_t weatherData;
WeatherData_t latestGoodWeatherData;

// Bresser data funcs
DecodeStatus decodeBresser5In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *wx);
void clearWeatherdata(WeatherData_t *wx);
void publishWeatherdata(const WeatherData_t *wx);
void printWeatherdata(const WeatherData_t *wx);

////////////////////////////////////////////////////////////////////////////////////////////////

// Generate WiFi network instance
#if defined(ESP32)
    // WiFiClientSecure net;
    WiFiClient wifi;
#elif defined(ESP8266)
    // BearSSL::WiFiClientSecure net;
    WiFiClient wifi;
#endif

// Generate MQTT client instance
// N.B.: Default message buffer size is too small!
PubSubClient mqttClient(wifi);

// Generate HTTP server instance
ESP8266WebServer httpServer(MY_HTTP_PORT);
void http_handleRoot();

// Generate CC1101 radio module instance
CC1101 radio = new Module(PIN_CC1101_CS, PIN_CC1101_GDO0, RADIOLIB_NC, PIN_CC1101_GDO2);

// Time keeping
uint32_t statusPublishPreviousMillis = 0;
time_t now;

////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_RX_INTERRUPT
volatile bool radio_Int_ENA = true;
volatile bool radio_Int_Flag = false;
ICACHE_RAM_ATTR void radio_INT_handler(void)
{
    if (!radio_Int_ENA) {
        return;
    }
    radio_Int_Flag = true;
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////

//
// Set up WiFi in Station Mode
//
void wifi_setup()
{
    Serial.print(F("[WiFi] Attempting to connect to SSID: "));
    Serial.println(ssid);

    WiFi.hostname(MY_HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    WiFi.waitForConnectResult();
    if (WiFi.status()) {
       Serial.print(F("[WiFi] Connected and got IP "));
       Serial.println(WiFi.localIP());
    }
}

/*
void sync_ntp()
{
    Serial.print("Setting time using SNTP ");
    configTime(-5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    now = time(nullptr);
    while (now < 1510592825)
    {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("done!");
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.print(asctime(&timeinfo));
}
*/

//
// Set up MQTT
//
void mqtt_setup()
{
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.connect(MY_HOSTNAME);
}

//
// Set up CC1101 for 868.35 MHz rx
//
bool cc1101_setup()
{
    Serial.println(F("[CC1101] Initializing"));

    // Initialization - starts with presence detection (chip version) and reset cmd
    //int state = radio.begin(868.35, 8.22, 57.136417, 270.0, 10, 32); // original
    int state = radio.begin(868.3, 8.21, 57.136417, 270, 10, 32); // https://github.com/matthias-bs/BresserWeatherSensorReceiver/blob/main/src/WeatherSensor.cpp
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Error initialising: [%d]\n", state);
        return false;
    }

    // Set Tx power very low, we are not transmitting anyway
    radio.setOutputPower(-30);

    // Setup for passive Rx
    state = radio.setCrcFiltering(false);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Error disabling crc filtering: [%d]\n", state);
        return false;
    }
    state = radio.fixedPacketLengthMode(27);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Error setting fixed packet length: [%d]\n", state);
        return false;
    }
    // Preamble: AA AA AA AA AA
    // Sync is: 2D D4
    // Preamble 40 bits but the CC1101 doesn't allow us to set that
    // so we use a preamble of 32 bits and then use the sync as AA 2D
    // which then uses the last byte of the preamble - we recieve the last sync byte
    // as the 1st byte of the payload.
    state = radio.setSyncWord(0xAA, 0x2D, 0, false);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Error setting sync words: [%d]\n", state);
        return false;
    }

#ifdef USE_RX_INTERRUPT
    radio.setGdo0Action(radio_INT_handler);
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Error with startReceive: [%d]\n", state);
        return false;
    }
#endif

    Serial.println(F("[CC1101] Setup complete. Awaiting incoming messages."));

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/util.c
//
uint16_t lfsr_digest16(uint8_t const message[], unsigned bytes, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k) {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i) {
            // fprintf(stderr, "key at bit %d : %04x\n", i, key);
            // if data bit is set then xor with key
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

////////////////////////////////////////////////////////////////////////////////////////////////

// Cribbed from rtl_433 project - but added extra checksum to verify uu
//
// Example input data:
//   EA EC 7F EB 5F EE EF FA FE 76 BB FA FF 15 13 80 14 A0 11 10 05 01 89 44 05 00
//   CC CC CC CC CC CC CC CC CC CC CC CC CC uu II SS GG DG WW  W TT  T HH RR  R Bt
// - C = Check, inverted data of 13 byte further
// - uu = checksum (number/count of set bits within bytes 14-25)
// - I = station ID (maybe)
// - G = wind gust in 1/10 m/s, normal binary coded, GGxG = 0x76D1 => 0x0176 = 256 + 118 = 374 => 37.4 m/s.  MSB is out of sequence.
// - D = wind direction 0..F = N..NNE..E..S..W..NNW
// - W = wind speed in 1/10 m/s, BCD coded, WWxW = 0x7512 => 0x0275 = 275 => 27.5 m/s. MSB is out of sequence.
// - T = temperature in 1/10 °C, BCD coded, TTxT = 1203 => 31.2 °C
// - t = temperature sign, minus if unequal 0
// - H = humidity in percent, BCD coded, HH = 23 => 23 %
// - R = rain in mm, BCD coded, RRxR = 1203 => 31.2 mm
// - B = Battery. 0=Ok, 8=Low.
// - S = sensor type, only low nibble used, 0x9 for Bresser Professional Rain Gauge
//
// Parameters:
//
// msg     - Pointer to message
// msgSize - Size of message
// wx      - Pointer to WeatherData
//
// Returns:
//
// DECODE_OK      - OK - WeatherData will contain the updated information
// DECODE_PAR_ERR - Parity Error
// DECODE_CHK_ERR - Checksum Error
//
DecodeStatus decodeBresser5In1Payload(uint8_t *msg, uint8_t msgSize, WeatherData_t *wx)
{
    // First 13 bytes need to match inverse of last 13 bytes
    for (unsigned col = 0; col < msgSize / 2; ++col) {
        if ((msg[col] ^ msg[col + 13]) != 0xff) {
            Serial.printf("%s: Parity wrong at %u\n", __func__, col);
            return DECODE_PAR_ERR;
        }
    }

    // Verify checksum (number number bits set in bytes 14-25)
    uint8_t bitsSet = 0;
    uint8_t expectedBitsSet = msg[13];
    for(uint8_t p = 14 ; p < msgSize ; p++) {
      uint8_t currentByte = msg[p];
      while(currentByte) {
        bitsSet += (currentByte & 1);
        currentByte >>= 1;
      }
    }
    if (bitsSet != expectedBitsSet) {
       Serial.printf("%s: Checksum wrong actual [%02X] != expected [%02X]\n", __func__, bitsSet, expectedBitsSet);
       return DECODE_CHK_ERR;
    }

    // Parse the data
    wx->sensor_id = msg[14];

    int temp_raw = (msg[20] & 0x0f) + ((msg[20] & 0xf0) >> 4) * 10 + (msg[21] &0x0f) * 100;
    if (msg[25] & 0x0f) {
        temp_raw = -temp_raw;
    }
    wx->temp_c = temp_raw * 0.1f;
    wx->temp_ok = true;

    wx->humidity = (msg[22] & 0x0f) + ((msg[22] & 0xf0) >> 4) * 10;

    wx->wind_direction_deg = ((msg[17] & 0xf0) >> 4) * 22.5f;

    int gust_raw = ((msg[17] & 0x0f) << 8) + msg[16];
    int wind_raw = (msg[18] & 0x0f) + ((msg[18] & 0xf0) >> 4) * 10 + (msg[19] & 0x0f) * 100;
    wx->wind_gust_meter_sec = gust_raw * 0.1f;
    wx->wind_avg_meter_sec = wind_raw * 0.1f;
    wx->wind_ok = true;

    int rain_raw = (msg[23] & 0x0f) + ((msg[23] & 0xf0) >> 4) * 10 + (msg[24] & 0x0f) * 100;
    wx->rain_mm = rain_raw * 0.1f;
    wx->rain_ok = true;

    wx->battery_ok = (msg[25] & 0x80) ? false : true;

    wx->valid = true;
    wx->timestamp = millis();

    return DECODE_OK;
}

//
// Reinitialize weather data struct to 'blank'
//
void clearWeatherdata(WeatherData_t *wx)
{
    wx->valid       = false;
    // 'False'-out fields available on Bresser 5-in-1
    wx->temp_ok     = false;
    wx->wind_ok     = false;
    wx->rain_ok     = false;
    // 'False'-out fields not available on Bresser 5-in-1
    wx->uv_ok       = false;
    wx->moisture_ok = false;
}

//
// Generate fake weather data for simulation
//
void fakeWeatherdata(WeatherData_t *wx)
{
    wx->battery_ok          = true;
    wx->sensor_id           = 0xff;
    wx->temp_c              = 22.2f;
    wx->humidity            = 55;
    wx->wind_direction_deg  = 333;
    wx->wind_gust_meter_sec = 44.4f;
    wx->wind_avg_meter_sec  = 11.1f;
    wx->rain_mm             = 9.9f;

    wx->temp_ok     = true;
    wx->wind_ok     = true;
    wx->rain_ok     = true;
    wx->moisture_ok = false;
    wx->uv_ok       = false;

    wx->valid = true;
    wx->timestamp = millis();
}

//
// Receive radio data if available
//
bool receiveRadioData(uint8_t *recvData)
{
#ifndef USE_RX_INTERRUPT  // polling based appoach
    int state = radio.receive(recvData, 27);
    if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        #ifdef DEBUG_DUMP_C1101
        Serial.print(F("[CC1101] Rx Timeout\n"));
        #endif
        return false;
    }
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[CC1101] Receive failed - error code %d\n", state);
        return false;
    }
    return true;
#else
    // Chip got any data yet?
    bool success = false;
    if (radio_Int_Flag) {

        // Disable interrupt while processing
        radio_Int_ENA = false;
        radio_Int_Flag = false;

        // Grab the data
        int state = radio.readData(recvData, 27);
        if (state == RADIOLIB_ERR_NONE) {
            success = true;
        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            #ifdef DEBUG_DUMP_C1101
            Serial.print(F("[CC1101] Rx Timeout\n"));
            #endif
        } else {
            Serial.printf("[CC1101] Receive failed - error code %d\n", state);
        }

        // Ready to receive more later
        radio.startReceive();
        radio_Int_ENA = true;
    }
    return success;
#endif
}

//
// Get weather data from CC1101 receiver and decode it
//
bool getWeatherdata(WeatherData_t *wx)
{
    uint8_t recvData[27];

    // (Re)Initialize struct
    clearWeatherdata(wx);

    // Check for radio data
    if (!receiveRadioData(recvData)) {
        return false;
    }

    #ifdef DEBUG_DUMP_C1101
    printRadioQuality();
    #endif

    // Verify last syncword is 1st byte of payload (see above) before proceeding with decode
    if (recvData[0] != 0xD4) {
       Serial.printf("[CC1101] Unexpected first byte 0x%02X instead of 0xD4\n", recvData[0]);
       return false;
    }

    #ifdef DEBUG_DUMP_C1101
    Serial.print(F("[CC1101] Rx Data: "));
    for(int i = 0 ; i < sizeof(recvData) ; i++) {
        Serial.printf(" %02X", recvData[i]);
    }
    Serial.println();
    #endif

    // Decode the information - skip the last sync byte we used to check the data is OK
    if (decodeBresser5In1Payload(&recvData[1], sizeof(recvData) - 1, wx) != DECODE_OK) {
        return false;
    }

    return true;
}

//
// Print weather sensor data on Serial
//
void printWeatherdata(const WeatherData_t *wx)
{
    Serial.printf("Id: [%08X] Battery: [%s] ", wx->sensor_id, wx->battery_ok ? "OK " : "Low");

    if (wx->temp_ok) {
        Serial.printf("Temp: [%3.1fC] Hum: [%3d%%] ", wx->temp_c, wx->humidity);
    } else {
        Serial.printf("Temp: [---.-C] Hum: [---%%] ");
    }

    if (wx->wind_ok) {
        Serial.printf("Wind max: [%3.1fm/s] Wind avg: [%3.1fm/s] Wind dir: [%4.1fdeg] ",
             wx->wind_gust_meter_sec, wx->wind_avg_meter_sec, wx->wind_direction_deg);
    } else {
        Serial.printf("Wind max: [--.-m/s] Wind avg: [--.-m/s] ");
    }

    if (wx->rain_ok) {
        Serial.printf("Rain: [%6.1fmm] ", wx->rain_mm);
    } else {
        Serial.printf("Rain: [-----.-mm] ");
    }

    if (wx->moisture_ok) {
        Serial.printf("Moisture: [%2d%%]", wx->moisture);
    }

    Serial.println();
}

//
// Publish weather sensor data as JSON string via MQTT
//
void publishWeatherdata(const WeatherData_t *wx)
{
    char mqtt_payload[MQTT_PAYLOAD_SIZE];
    char *buf = mqtt_payload;

    // Example:
    // {"sensor_id":"0x12345678","ch":0,"battery_ok":true,"humidity":44,"wind_gust":1.2,"wind_avg":1.2,"wind_dir":150,"rain":146}

    buf += sprintf(buf, "{\"sensor_id\": \"%X\"", wx->sensor_id);
    buf += sprintf(buf, ",\"battery_ok\":%d", wx->battery_ok ? 1 : 0);

    if (wx->temp_ok) {
        buf += sprintf(buf, ",\"temp_c\":%.1f", wx->temp_c);
        buf += sprintf(buf, ",\"humidity\":%d", wx->humidity);
    }

    if (wx->wind_ok) {
        buf += sprintf(buf, ",\"wind_gust\":%.1f", wx->wind_gust_meter_sec);
        buf += sprintf(buf, ",\"wind_avg\":%.1f", wx->wind_avg_meter_sec);
        buf += sprintf(buf, ",\"wind_dir\":%.1f", wx->wind_direction_deg);
    }

    if (wx->uv_ok) {
        buf += sprintf(buf, ",\"uv\":%.1f,", wx->uv);
    }

    if (wx->rain_ok) {
        buf += sprintf(buf, ",\"rain\":%.1f", wx->rain_mm);
    }

    if (wx->moisture_ok) {
        buf += sprintf(buf, ",\"moisture\":%d", wx->moisture);
    }

    buf += sprintf(buf, "}");

    mqttClient.publish(MQTT_PUB_DATA, mqtt_payload);
}

// Print CC1101 radio link stats on Serial
// - RSSI: Received Signal Strength Indication
// - LQI:  Link Quality Indicator
void printRadioQuality()
{
    Serial.printf("[CC1101] RSSI: %f LQI: %d\n", radio.getRSSI(), radio.getLQI());
}

// Publish CC1101 radio receiver info as JSON string via MQTT
// - RSSI: Received Signal Strength Indication
// - LQI:  Link Quality Indicator
void publishRadioQuality()
{
    char mqtt_payload[MQTT_PAYLOAD_SIZE];
    sprintf(mqtt_payload, "{\"rssi\": %f, \"lqi\": %d }", radio.getRSSI(), radio.getLQI());

    mqttClient.publish(MQTT_PUB_RADIO, mqtt_payload);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void http_handleRoot()
{
  const WeatherData_t *wx = &latestGoodWeatherData;
  const uint32_t millisSinceWx = millis() - wx->timestamp;
  String msg;

  msg += F("<html><head><meta http-equiv='refresh' content='5' />\n");
  msg += F("<style>div.mid { font-size: 2vw; height: 80vh; display: flex; align-items: center; justify-content: center }</style>");
  msg += F("</head><title>Bresser 5-in-1</title>\n");
  msg += F("<body><div class=mid><center>");
  if (!wx->valid) {
    msg += "No radio data received yet";
  } else {
    char fields[300];
    char *buf = fields;
    buf += sprintf(buf, "<small><ul>[Data from %u sec ago]</ul></small><br>", millisSinceWx/1000);
    buf += sprintf(buf, "Sensor: %04X ", wx->sensor_id);
    buf += sprintf(buf, "Battery: %d <br>", wx->battery_ok ? 1 : 0);
    if (wx->temp_ok) {
        buf += sprintf(buf, "Temp: %.1f &#x2103; ", wx->temp_c);
        buf += sprintf(buf, "Hum: %d %%<br>", wx->humidity);
    }
    if (wx->wind_ok) {
        buf += sprintf(buf, "Wind Gust: %.1f m/s ", wx->wind_gust_meter_sec);
        buf += sprintf(buf, "Avg: %.1f m/s ", wx->wind_avg_meter_sec);
        buf += sprintf(buf, "Dir: %.1f &#x00b0;<br>", wx->wind_direction_deg);
    }
    if (wx->rain_ok) {
        buf += sprintf(buf, "Rain: %.1f mm<br>", wx->rain_mm);
    }
    msg += fields;
  }
  msg += F("</center></div></body></html>\n");
  httpServer.send(200, "text/html", msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////

//
// Main Setup
//
void setup()
{
    bool ok = true;

    #ifdef LED_EN
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, HIGH);
    #endif

    Serial.begin(115200);
    Serial.print(F("\nBresser Weather Sensor CC1101+ESP MQTT - version JW_20221130\n"));

    clearWeatherdata(&latestGoodWeatherData);

    wifi_setup();
    mqtt_setup();
    httpServer.on("/", http_handleRoot);
    httpServer.begin();

    ok = cc1101_setup();
    if (!ok) {
      while(true);
    }

}


//
// Main Loop
//
void loop()
{
    const uint32_t currentMillis = millis();

    mqttClient.loop();
    httpServer.handleClient();

    #ifdef FAKE_WX_DATA
    fakeWeatherdata(&weatherData);
    #else
    getWeatherdata(&weatherData);
    #endif

    #ifdef LED_EN
    if (weatherData.valid) {
      digitalWrite(LED_GPIO, LOW);
    } else {
      digitalWrite(LED_GPIO, HIGH);
    }
    #endif

    if (WiFi.status() != WL_CONNECTED) {
        Serial.print(F("[WiFi] Disconnected, reconnecting."));
        WiFi.waitForConnectResult();
        if (WiFi.status()) {
           Serial.print(F("[WiFi] Reconnected and got IP "));
           Serial.println(WiFi.localIP());
        } else {
            return;
        }
    }

    if (!mqttClient.connected()) {
        Serial.print(F("[MQTT] Disconnected, reconnecting."));
        if (mqttClient.connect(MY_HOSTNAME)) {
            mqttClient.publish(MQTT_PUB_STATUS, "online");
            Serial.print(F("[MQTT] Connected."));
        } else {
            return;
        }
    }

    // publish weather data always upon valid reception
    if (weatherData.valid) {
        latestGoodWeatherData = weatherData;
        publishWeatherdata(&latestGoodWeatherData);
        printWeatherdata(&latestGoodWeatherData);
    }

    // publish data link quality at STATUS_INTERVAL intervals
    if (currentMillis - statusPublishPreviousMillis >= STATUS_INTERVAL) {
        statusPublishPreviousMillis = currentMillis;
        publishRadioQuality();
        printRadioQuality();
    }
}
