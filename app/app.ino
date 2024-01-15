#include "RTC.h"
#include "Arduino_LED_Matrix.h"
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
bool isSensorCheck = false;
long randNumber;

ArduinoLEDMatrix matrix;
uint8_t off_frame[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

uint8_t on_frame[8][12] = {
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
};

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
        String client_id = "esp8266-client-" + random(300);
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

void OnRedLed() {
  analogWrite(greenLed, 0);
  analogWrite(redLed, 800);
  delay(1000);
  analogWrite(redLed, 0);
}

void OnGreenLed() {
  analogWrite(greenLed, 800);
}

bool checkFlame(int sec, bool inspect) {
  if(inspect) {
    digitalWrite(irLedPin, HIGH);
  }else{
    digitalWrite(irLedPin, LOW);
  }

  if(digitalRead(irLedPin) == HIGH){
    return false;
  }

  int flamesensorValue; // 불꽃감지 센서 동작
  int startTime = millis();
  int endTime = startTime;

  while((endTime - startTime) <= sec * 1000)
  {
    flamesensorValue = digitalRead(flameSensorPin);
    if(flamesensorValue == 0) break; // 센서가 인식되면 즉시 종료.
    endTime = millis();
  }

  digitalWrite(irLedPin, LOW);

  return flamesensorValue == 0 ? true : false;
}

void inspectFlameSensor() {
  isSensorCheck = checkFlame(5, true);
}

void setup(){
  Serial.begin(9600);
  while (!Serial);
  
  randomSeed(analogRead(0));

  connectToWiFi();
  RTC.begin();
  timeClient.begin();
  timeClient.update();

  // matrix led
  matrix.begin();

  auto unixTime = timeClient.getEpochTime();
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
  RTC.setAlarmCallback(inspectFlameSensor, alarmTime, matchTime);

  // Retrieve the date and time from the RTC and print them
  RTCTime currentTime;
  RTC.getTime(currentTime); 
  Serial.println("The RTC was just set to: " + String(currentTime));

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  connectToMQTT();

  pinMode(flameSensorPin, INPUT); // 불꽃 센서 입력모드 설정
  pinMode(irLedPin, OUTPUT); // 적외선 송신 모듈 출력모드 설정
  dht.begin(); // 온습도 센서 동작

  isSensorCheck = checkFlame(5, true);
}

void loop(){
  // MQTT Broker 연결
  if (!mqtt_client.connected()) {
      connectToMQTT();
  }
  mqtt_client.loop();

  float temperature = dht.readTemperature(); // 온도 센서 데이터
  float humidity = dht.readHumidity(); // 습도 센서 데이터
  // bool isFlameDetected = digitalRead(irLedPin) == LOW ? checkFlame(5, false) : false; // 불꽃감지 센서 데이터
  bool isFlameDetected = checkFlame(5, false);

  // if(digitalRead(irLedPin) == LOW){
  //   isFlameDetected = checkFlame(5, false);
  // }

  if(isFlameDetected) {
    matrix.renderBitmap(on_frame, 8, 12);
  } else {
    matrix.renderBitmap(off_frame, 8, 12);
  }

  // 내장 RTC 에서 시간 값을 가져옴.
  RTCTime currentTime;
  RTC.getTime(currentTime); 

  DynamicJsonDocument jsonDocument(200);

  jsonDocument["id"] = 0;
  jsonDocument["name"] = "정보공학관 1층";
  jsonDocument["temperature"] = round(temperature * 10.0) / 10.0;
  jsonDocument["humidity"] = round(humidity * 10.0) / 10.0;
  jsonDocument["fireDetected"] = isFlameDetected;
  jsonDocument["checkResult"] = isSensorCheck;
  jsonDocument["time"] = currentTime.getUnixTime() * 1000; // sec -> ms 단위 변환

  // JSON 문자열을 고정할 버퍼
  char buffer[256];

  // JSON 문서를 문자열로 직렬화
  serializeJson(jsonDocument, buffer);

  // MQTT 브로커에 JSON 문자열 게시
  mqtt_client.publish(mqtt_topic, buffer);

  delay(4000);
}
