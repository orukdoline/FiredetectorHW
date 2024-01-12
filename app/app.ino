#include "RTC.h"
#include <NTPClient.h>

#if defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOWIFIR4)
#include <WiFiS3.h>
#endif

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "WiFiSSLClient.h"
#include <WiFiUdp.h>
#include <DHT.h>

// WiFi credentials
const char *ssid = "JSW iPhone14 Pro";             // Replace with your WiFi name
const char *pass = "0000001151";   // Replace with your WiFi password

// MQTT Broker settings
const int mqtt_port = 8883;  // MQTT port (TLS)
const char *mqtt_broker = "fbb1763c.ala.us-east-1.emqxsl.com";  // EMQX broker endpoint
const char *mqtt_topic = "fire_detector";     // MQTT topic
const char *mqtt_username = "capstone";  // MQTT username for authentication
const char *mqtt_password = "1234";  // MQTT password for authentication

// WiFi and MQTT client initialization
WiFiSSLClient client;
PubSubClient mqtt_client(client);

int wifiStatus = WL_IDLE_STATUS;
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient timeClient(Udp);

static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

const int flameSensorPin = 0; // 불꽃 감지 센서 핀
const int irLedPin = 3; // 적외선 송신 모듈 핀
DHT dht(1, DHT22); // 온습도 센서 핀
const int redLed = 4; // RedLED 핀
const int greenLed = 5; // GreenLED 핀

bool isSensorCheck = true;

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToWiFi(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();
}

void connectToMQTT() {
    client.setCACert(ca_cert);
    while (!mqtt_client.connected()) {
        String client_id = "esp8266-client-1234";
        Serial.println("Connecting to MQTT Broker.....");
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.println(mqtt_client.state());
            delay(5000);
        }
    }
}

//void mqttCallback(char *topic, byte *payload, unsigned int length) {
//    Serial.print("Message received on topic: ");
//    Serial.print(topic);
//    Serial.print("]: ");
//    for (int i = 0; i < length; i++) {
//        Serial.print((char) payload[i]);
//    }
//    Serial.println();
//}

void OnRedLed() {
  analogWrite(greenLed, 0);
  analogWrite(redLed, 800);
  delay(1000);
  analogWrite(redLed, 0);
}

void OnGreenLed() {
  analogWrite(greenLed, 800);
}

bool outputIRSignal() {
  digitalWrite(irLedPin, HIGH); // 적외선 LED 동작
  int flamesensorValue = digitalRead(flameSensorPin); // 불꽃감지 센서 동작
  delay(5000); // 5초 대기
  digitalWrite(irLedPin, LOW); // 적외선 LED 끄기
  //Serial.println("check function operation!");
  return flamesensorValue == 0 ? true : false;
}

bool checkFlame() {
  int flamesensorValue = digitalRead(flameSensorPin); // 불꽃감지 센서 동작
  return flamesensorValue == 0 ? true : false;
}

void alarmCallback() {
  isSensorCheck = outputIRSignal();
}

void setup(){
  Serial.begin(9600);
  while (!Serial);

  connectToWiFi();
  RTC.begin();
  Serial.println("\nStarting connection to server...");
  timeClient.begin();
  timeClient.update();

  auto timeZoneOffsetHours = 9;
  auto unixTime = timeClient.getEpochTime() + (timeZoneOffsetHours * 3600);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);

  // Trigger the alarm every time the seconds are zero
  RTCTime alarmTime;
  alarmTime.setSecond(0);

  // Make sure to only match on the seconds in this example - not on any other parts of the date/time
  AlarmMatch matchTime;
  matchTime.addMatchSecond();

  // sets the alarm callback
  RTC.setAlarmCallback(alarmCallback, alarmTime, matchTime);

  // Retrieve the date and time from the RTC and print them
  RTCTime currentTime;
  RTC.getTime(currentTime); 
  Serial.println("The RTC was just set to: " + String(currentTime));

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  //mqtt_client.setCallback(mqttCallback);
  connectToMQTT();

  pinMode(flameSensorPin, INPUT); // 불꽃 센서 입력모드 설정
  pinMode(irLedPin, OUTPUT); // 적외선 송신 모듈 출력모드 설정
  dht.begin(); // 온습도 센서 동작
}

void loop(){
  timeClient.update();

  if (!mqtt_client.connected()) {
      connectToMQTT();
  }
  mqtt_client.loop();

  float temperature = dht.readTemperature(); // 온도 센서 데이터
  float humidity = dht.readHumidity(); // 습도 센서 데이터
  bool isFlameDetected = checkFlame(); // 불꽃감지 센서 데이터

  //Serial.print("temperature : ");
  //Serial.println(temperature);
  //Serial.print("humidity : ");
  //Serial.println(humidity);

  RTCTime currentTime;
  RTC.getTime(currentTime); 

  // Serial.println(currentTime.getUnixTime() * 1000);

  isFlameDetected == true ? OnRedLed() : OnGreenLed();

  DynamicJsonDocument jsonDocument(200);

  jsonDocument["id"] = 5;
  jsonDocument["name"] = "정보공학관 3층";
  jsonDocument["temperature"] = round(temperature * 10.0) / 10.0;
  jsonDocument["humidity"] = round(humidity * 10.0) / 10.0;
  jsonDocument["fireDetected"] = isFlameDetected;
  jsonDocument["checkResult"] = isSensorCheck;
  jsonDocument["time"] = currentTime.getUnixTime();

  // JSON 문자열을 고정할 버퍼
  char buffer[256];

  // JSON 문서를 문자열로 직렬화
  serializeJson(jsonDocument, buffer);

  // MQTT 브로커에 JSON 문자열 게시
  mqtt_client.publish(mqtt_topic, buffer);
  
  delay(2000);
}
