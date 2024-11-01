U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,/*reset=*/U8X8_PIN_NONE);

// #은 매크로 표현으로 컴파일시 DHTPIN에 자동으로 A1이 할당됌
#define DHTPIN A1 // Digital pin connected to the DHT sensor ==A1핀에 접해라
#define DHTTYPE DHT22 // DHT 22 (AM2302), AM2321 //  센서의 타입으로 우리모델은 22모델
#define SOILHUMI A6 

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE); //  dht 클래스 생성

uint32_t DataCaptureDelay = 2000; //ms   // 32비트 인트로 재정의함 2초
uint32_t DataCapture_ST = 0;

int Soilhumi = 0; 
float Temp;
float Humi;

void setup() {
 dht.begin();//센서도 시작해라
 u8g2.begin();
 
 DataCapture_ST = millis();  //시작시간을 ms로 저장
 pinMode(SOILHUMI, INPUT);
}

void loop() {
 if ((millis() - DataCapture_ST ) > DataCaptureDelay) { //2초 간격으로 실행  
 Humi = dht.readHumidity();//습도 읽어오는 함수 (저항값을 읽어서 -> 디지털신호인 플로트로)
 // Read temperature as Celsius (the default)
 Temp = dht.readTemperature(); // 온도를 읽어오는 함수 (저항값을 읽어서 -> 디지털신호인 플로트)
 
 // Check if any reads failed and exit early (to try again).
  if (isnan(Humi) || isnan(Temp)) {  // isnan 함수는 뭐가 들어갔냐 마냐를 확인하는 함수 
  Serial.println(F("Failed to read from DHT sensor!"));
  return;
  }
 Soilhumi = map(analogRead(SOILHUMI), 0, 1023, 100, 0); // 이 코드가 교재에 없음
 OLEDdraw();
 DataCapture_ST = millis();
 }
}

void OLEDdraw(){
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_ncenB08_te);
  u8g2.drawStr(1,15,"SMART FARM");
  
  u8g2.drawStr(15,36,"Temp.");
  u8g2.setCursor(85,36);
  u8g2.print(Temp); 
  u8g2.drawStr(114,36,"\xb0");
  u8g2.drawStr(119,36,"C");  
  
  u8g2.drawStr(15,47,"Humidity");
  u8g2.setCursor(85,47);
  u8g2.print(Humi);
  u8g2.drawStr(116,47,"%");

  u8g2.drawStr(15,58,"Soil Humi.");
  u8g2.setCursor(85,58);
  u8g2.print(Soilhumi);
  u8g2.drawStr(116,58,"%");

  u8g2.sendBuffer();
}

#include <U8g2lib.h>
#include "DHT.h" //교재 참고 
