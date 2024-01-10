#include <DHT.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const int flameSensorPin = A0; // 불꽃 감지 센서 핀
const int pirSensorPin = 2; // 인체 감지 센서 핀
const int irLedPin = 3; // 적외선 송신 모듈 핀
DHT dht(1, DHT22); // 온습도 센서 핀

char ssid[] = "";    // 와이파이 아이디
char pass[] = "";    // 와이파이 비밀번호

char mqtt_server[] = "fbb1763c.ala.us-east-1.emqxsl.com"; // MQTT 브로커 주소
char mqtt_port = 8883;
char mqtt_user[] = "capstone"; // MQTT 아이디
char mqtt_pass[] = "1234"; // MQTT 비밀번호

const char* ntpServerName = "pool.ntp.org"; // NTP 서버 정보
const int timeZone = 9;  // 한국 시간대: GMT+9

WiFiClient espClient;
PubSubClient client(espClient);

const char publish_topic[] = "fire_detector"; // 토픽

unsigned long lastIRTime = 0; // 점검이 동작했던 시간정보를 저장하기 위한 변수

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServerName, timeZone * 3600);

void setup() {
  Serial.begin(9600);
  pinMode(flameSensorPin, INPUT); // 불꽃 센서 입력모드 설정
  pinMode(pirSensorPin, INPUT); // 인체감지 센서 입력모드 설정
  pinMode(irLedPin, OUTPUT); // 적외선 송신 모듈 출력모드 설정
  dht.begin(); // 온습도 센서 동작

  while (!Serial); // 시리얼 통신이 정상적으로 되는지 체크

  // 와이파이 연결
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("You're connected to the network");
  Serial.println();

  timeClient.begin();

  // MQTT 브로커 연결
  client.setServer(mqtt_server, mqtt_port);
  // client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ArduinoClient", mqtt_user, mqtt_pass)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

// void callback(char* topic, byte* payload, unsigned int length) {
  // 콜백 함수 구현 (필요한 경우)
// }

void loop() {
  client.loop();

  unsigned long currentTime = millis(); // 현재 시간 정보 저장
  bool isSensorCheck = true;
  timeClient.update();

  // 1시간에 한 번 불꽃감지 센서 점검
  if (currentTime - lastIRTime >= 3600000) {
    isSensorCheck = outputIRSignal(); // 점검 실행
    lastIRTime = currentTime; // 점검을 했던 시간 정보 저장
  }

  unsigned long epochTime = timeClient.getEpochTime(); // 현재시간 정보 데이터
  float temperature = dht.readTemperature(); // 온도 센서 데이터
  float humidity = dht.readHumidity(); // 습도 센서 데이터
  bool isFlameDetected = checkFlame(); // 불꽃감지 센서 데이터
  unsigned long isMotionDetected = checkMotion(); // 인체감지 센서 데이터

  // JSON 문서 생성
  DynamicJsonDocument jsonDocument(200);

  // JSON 문서를 데이터로 채우기
  jsonDocument["id"] = 1;
  jsonDocument["name"] = "정보공학관 2층";
  jsonDocument["temperature"] = temperature;
  jsonDocument["humidity"] = humidity;
  jsonDocument["fireDetected"] = isFlameDetected;
  jsonDocument["movementDetectedTime"] = isMotionDetected;
  jsonDocument["checkResult"] = isSensorCheck;
  jsonDocument["time"] = epochTime;

  // JSON 문자열을 고정할 버퍼
  char buffer[256];

  // JSON 문서를 문자열로 직렬화
  serializeJson(jsonDocument, buffer);

  // MQTT 브로커에 JSON 문자열 게시
  client.publish(publish_topic, buffer);

  delay(10000); // 10초 대기
}

// 화재감지 관련 메소드
bool checkFlame() {
  int flamesensorValue = analogRead(flameSensorPin); // 불꽃감지 센서 동작
  float temperature = dht.readTemperature(); // 온도 센서 동작
  int threshold = 500; // 불꽃 센서가 반응하는 적외선 수치
  if (flamesensorValue > threshold && temperature > 50) return true;  // 불꽃이 감지되고 주변온도가 높으면
  else return false; // 위 조건을 둘 다 만족하지 않으면
}

// 인체감지 관련 메소드
unsigned long checkMotion() {
  int motionValue = digitalRead(pirSensorPin); // 인체 감지 센서 동작
  if (motionValue == HIGH) {
    unsigned long epochTime = timeClient.getEpochTime(); // 현재시간 정보 데이터
    return epochTime;  // 움직임이 감지되면
  }
}

// 불꽃감지 센서 점검 관련 메소드
bool outputIRSignal() {
  int threshold = 500; // 불꽃 센서가 반응하는 적외선 수치

  digitalWrite(irLedPin, HIGH); // 적외선 LED 동작
  int flamesensorValue = analogRead(flameSensorPin); // 불꽃감지 센서 동작
  delay(3000); // 3초 대기
  digitalWrite(irLedPin, LOW); // 적외선 LED 끄기

  if (flamesensorValue > threshold) return true;  // 불꽃 센서가 정상작동하면
  else return false; // 불꽃 센서가 오작동하면
}
