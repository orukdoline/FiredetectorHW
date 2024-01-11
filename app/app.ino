#include <DHT.h>

const int flameSensorPin = 0; // 불꽃 감지 센서 핀
// const int pirSensorPin = 2; // 인체 감지 센서 핀
const int irLedPin = 3; // 적외선 송신 모듈 핀
DHT dht(1, DHT22); // 온습도 센서 핀

unsigned long lastIRTime = 0; // 점검이 동작했던 시간정보를 저장하기 위한 변수

void setup() {
  Serial.begin(9600);
  pinMode(flameSensorPin, INPUT); // 불꽃 센서 입력모드 설정
  // pinMode(pirSensorPin, INPUT); // 인체감지 센서 입력모드 설정
  pinMode(irLedPin, OUTPUT); // 적외선 송신 모듈 출력모드 설정
  dht.begin(); // 온습도 센서 동작

  while (!Serial); // 시리얼 통신이 정상적으로 되는지 체크

  
}

void loop() {

  unsigned long currentTime = millis(); // 현재 시간 정보 저장
  bool isSensorCheck = true;

  // 1시간에 한 번 불꽃감지 센서 점검
  if (currentTime - lastIRTime >= 3600000) {
    isSensorCheck = outputIRSignal(); // 점검 실행
    lastIRTime = currentTime; // 점검을 했던 시간 정보 저장
  }

  
  float temperature = dht.readTemperature(); // 온도 센서 데이터
  float humidity = dht.readHumidity(); // 습도 센서 데이터
  isSensorCheck = outputIRSignal();
  // bool isFlameDetected = checkFlame(); // 불꽃감지 센서 데이터
  //unsigned long isMotionDetected = checkMotion(); // 인체감지 센서 데이터

  Serial.print("온도 : ");
  Serial.println(temperature);
  Serial.print("습도 : ");
  Serial.println(humidity);
  Serial.print("점검결과 : ");
  Serial.println(isSensorCheck);

  delay(5000); // 10초 대기
}

// 화재감지 관련 메소드
bool checkFlame() {
  int flamesensorValue = digitalRead(flameSensorPin); // 불꽃감지 센서 동작
  float temperature = dht.readTemperature(); // 온도 센서 동작
  int threshold = 500; // 불꽃 센서가 반응하는 적외선 수치
  if (flamesensorValue > threshold && temperature > 50) return true;  // 불꽃이 감지되고 주변온도가 높으면
  else return false; // 위 조건을 둘 다 만족하지 않으면
}

// 인체감지 관련 메소드
//unsigned long checkMotion() {
//  int motionValue = digitalRead(pirSensorPin); // 인체 감지 센서 동작
//  if (motionValue == HIGH) {
//    unsigned long epochTime = timeClient.getEpochTime(); // 현재시간 정보 데이터
//    return epochTime;  // 움직임이 감지되면
//  }
// }

// 불꽃감지 센서 점검 관련 메소드
bool outputIRSignal() {

  digitalWrite(irLedPin, HIGH); // 적외선 LED 동작
  int flamesensorValue = digitalRead(flameSensorPin); // 불꽃감지 센서 동작
  delay(3000); // 3초 대기
  //digitalWrite(irLedPin, LOW); // 적외선 LED 끄기

  if (flamesensorValue == true) return true;  // 불꽃 센서가 정상작동하면
  else return false; // 불꽃 센서가 오작동하면
}
