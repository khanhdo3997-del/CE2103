#include <WiFi.h>
#include "math.h"
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <SPI.h>
#include "Ucglib.h"

/* ------------------------------ MACRO ------------------------------ */
#define ADC_MIN_ERROR 50    
#define ADC_MAX_ERROR 4050  
#define LED_PIN_FIRE 16
#define LED_PIN 17
#define BUZZER_PIN 18
#define ANALOG_PIN_1 35
#define ANALOG_PIN_2 34
#define FALSE 0
#define TRUE 1
#define R_FIXED 10000.0 
#define BETA 3950       
#define T0 298.15       
#define R0 10000.0      
#define NORMALAREA 0
#define HOTAREA 1
#define KITCHENAREA 2

/* ------------------------------ GLOBAL VARIABLE ------------------------------ */
enum deviceStatusType {
  NORMAL = 0,
  SENSOR_1_ERR,
  SENSOR_2_ERR,
  ERROR
};

// Khởi tạo màn hình: CD=2, CS=5, RESET=4
Ucglib_ILI9341_18x240x320_HWSPI ucg(2, 5, 4);

/* WIFI & MQTT SETUP */
const char* ssid = "MERCUSYS_E421";
const char* password = "Goldenage123456";
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

/* Biến hệ thống */
float Temp = 0;
float oneMTempAgo = 0;
bool fireDetected = false;
bool oneTimeFlag = false;
deviceStatusType deviceStatus = NORMAL;
int counter = 0;
char curArea = 0;

const int ADC_RES = 4095;
const float VCC = 3.3;

unsigned long currentTime = 0;
unsigned long lastTimeMQTT = 0;   
unsigned long lastDisplayTime = 0;
const unsigned long intervalMQTT = 5000;
unsigned long lastReconnectAttempt = 0;

// Biến điều khiển non-blocking cho ngoại vi
unsigned long buzzerStartTime = 0;
bool isBuzzing = false;
unsigned long ledBlinkStartTime = 0;
bool isLedBlinking = false;

/* ------------------------------ DISPLAY FUNCTIONS ------------------------------ */
void initDisplay() {
  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.setRotate90();

  // Background ban đầu
  ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
  ucg.setColor(0, 0, 100, 0);
  ucg.setColor(1, 0, 100, 0);
  ucg.setColor(2, 20, 20, 20);
  ucg.setColor(3, 20, 20, 20);
  ucg.drawGradientBox(0, 0, 320, 240);

  // Title
  ucg.setFont(ucg_font_logisoso32_tf);
  ucg.setColor(0, 5, 0); ucg.setPrintPos(50, 120); ucg.print("Fire Detection"); 
  ucg.setColor(0, 255, 0); ucg.setPrintPos(50, 120); ucg.print("Fire Detection");

  ucg.setFont(ucg_font_courB14_tf);
  ucg.setColor(20, 255, 20); ucg.setPrintPos(90, 200); ucg.print("Starting... OK!");

  delay(1500);

  // Xóa màn hình chuẩn bị vào giao diện chính
  ucg.setColor(0, 0, 0); // Đặt màu đen
  ucg.drawBox(0, 0, 320, 240);
  
  ucg.setColor(255, 255, 255);
  ucg.setFont(ucg_font_helvR12_tf);
  ucg.setPrintPos(10, 30);
  ucg.print("SYSTEM MONITORING");
}

void updateDisplay(float t, bool fire, deviceStatusType status) {
  
  //  ĐỌC TRỰC TIẾP 2 SENSOR CHO HIỂN THỊ
  int a1 = analogRead(ANALOG_PIN_1);
  int a2 = analogRead(ANALOG_PIN_2);

  bool s1_ok = !((a1 < ADC_MIN_ERROR) || (a1 > ADC_MAX_ERROR));
  bool s2_ok = !((a2 < ADC_MIN_ERROR) || (a2 > ADC_MAX_ERROR));

  float t1 = s1_ok ? convertToCelsius(a1) : 0;
  float t2 = s2_ok ? convertToCelsius(a2) : 0;

  //  NHIỆT ĐỘ CHUNG
  ucg.setColor(0, 0, 0);
  ucg.drawBox(30, 60, 220, 50);

  ucg.setFont(ucg_font_logisoso32_tf);
  ucg.setPrintPos(40, 100);

  if (fire) ucg.setColor(255, 0, 0);
  else ucg.setColor(0, 255, 255);

  ucg.print(t, 1);
  ucg.print(" C");

  //  TRẠNG THÁI FIRE/SAFE
  ucg.setColor(0, 0, 0);
  ucg.drawBox(30, 130, 260, 40);

  ucg.setFont(ucg_font_courB18_tf);
  ucg.setPrintPos(40, 160);

  if (fire) {
    ucg.setColor(255, 0, 0);
    ucg.print("!!! FIRE !!!");
  } else {
    ucg.setColor(0, 255, 0);
    ucg.print("SAFE");
  }

  //  HIỂN THỊ 2 SENSOR 
  ucg.setColor(0, 0, 0);
  ucg.drawBox(10, 190, 300, 50); // vùng mới thay cho sensor status

  ucg.setFont(ucg_font_helvR08_hr);
  ucg.setColor(200, 200, 200);

  // Sensor 1
  ucg.setPrintPos(10, 205);
  ucg.print("S1: ");
  if (s1_ok) {
    ucg.print(t1, 1);
    ucg.print(" C");
  } else {
    ucg.print("error:");
  }

  // Sensor 2
  ucg.setPrintPos(10, 225);
  ucg.print("S2: ");
  if (s2_ok) {
    ucg.print(t2, 1);
    ucg.print(" C");
  } else {
    ucg.print("error:");
  }
}

/* ------------------------------ LOGIC ------------------------------ */
float convertToCelsius(float analogSensor) {
  float voltage = analogSensor * VCC / ADC_RES;
  if (voltage <= 0.1) return 0;
  float R_ntc = R_FIXED * (VCC / voltage - 1);
  float tempK = 1.0 / ( (1.0 / T0) + (1.0 / BETA) * log(R_ntc / R0) );
  return tempK - 273.15;
}

float readTemp(void) {
  int a1 = analogRead(ANALOG_PIN_1);
  int a2 = analogRead(ANALOG_PIN_2);
  float tResult = 0;

  if ((a1 < ADC_MIN_ERROR) || (a1 > ADC_MAX_ERROR)) {
    deviceStatus = SENSOR_1_ERR;
    if ((a2 < ADC_MIN_ERROR) || (a2 > ADC_MAX_ERROR)) {
      deviceStatus = ERROR;
      tResult = 0; 
    } else {
      tResult = convertToCelsius(a2);
    }
  } else {
    deviceStatus = ((a2 < ADC_MIN_ERROR) || (a2 > ADC_MAX_ERROR)) ? SENSOR_2_ERR : NORMAL;
    tResult = convertToCelsius(a1);
  }
  return tResult;
}

char preArea(float t) {
  if(t >= 30 && t <= 40) return HOTAREA;
  if(t > 40 && t <= 55) return KITCHENAREA;
  return NORMALAREA;
}

bool fireDetect(float oldT, float curT, char area) {
  if((curT - oldT) >= 8) return TRUE;
  if(area == NORMALAREA && curT > 54) return TRUE;
  if(area == HOTAREA && curT > 69) return TRUE;
  if(area == KITCHENAREA && curT > 84) return TRUE;
  return FALSE;
}

/* ------------------------------ NETWORK ------------------------------ */
void setupWiFi() {
  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500); Serial.print("."); retry++;
  }
  if(WiFi.status() == WL_CONNECTED) Serial.println("\nConnected!");
}

bool reconnect() {
  bool ret = false;
  static unsigned long lastReconnect = 0;
  if (millis() - lastReconnect > 5000) {
    lastReconnect = millis();
    String clientId = "ESP32-Fire-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT Connected");
      ret = true;
    }
  }

  return ret;
}

void sendDataMQTT(float t, bool fire, deviceStatusType status) {
  if (!client.connected()) return;
  String payload = "{\"node_id\":\"NODE_02\",\"temperature\":" + String(t, 2) + 
                   ",\"fire_status\":\"" + String(fire ? "FIRE" : "NORMAL") + 
                   "\",\"device_status\":" + String(status) + "}";
  client.publish("CE2103/firedetect", payload.c_str());
  Serial.println("MQTT Sent: " + payload);
}

/* ------------------------------ PERIPHERALS ------------------------------ */
void handleBuzzer(bool fire) {
  if (!fire) {
    digitalWrite(BUZZER_PIN, LOW);
    isBuzzing = false;
    return;
  }
  unsigned long now = millis();
  static unsigned long lastBuzzCycle = 0;
  if (!isBuzzing && (now - lastBuzzCycle >= 2000)) { // Kêu mỗi 2s khi có cháy
    lastBuzzCycle = now;
    buzzerStartTime = now;
    isBuzzing = true;
    digitalWrite(BUZZER_PIN, HIGH);
  }
  if (isBuzzing && (now - buzzerStartTime >= 1000)) { // Kêu dài 1s
    digitalWrite(BUZZER_PIN, LOW);
    isBuzzing = false;
  }
}

void LedBlink5s() {
  if (fireDetected) {
    digitalWrite(LED_PIN_FIRE, HIGH);
    digitalWrite(LED_PIN, LOW);
    return;
  }
  digitalWrite(LED_PIN_FIRE, LOW);
  unsigned long now = millis();
  static unsigned long lastBlink = 0;
  if (!isLedBlinking && (now - lastBlink >= 5000)) {
    lastBlink = now;
    ledBlinkStartTime = now;
    isLedBlinking = true;
    digitalWrite(LED_PIN, HIGH);
  }
  if (isLedBlinking && (now - ledBlinkStartTime >= 200)) {
    digitalWrite(LED_PIN, LOW);
    isLedBlinking = false;
  }
}

/* ------------------------------ MAIN LOOP ------------------------------ */
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_FIRE, OUTPUT);
  analogReadResolution(12);

  initDisplay();
  setupWiFi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  currentTime = millis();

  // Kết nối lại MQTT nếu rớt mạng (Non-blocking)
  // if (WiFi.status() == WL_CONNECTED) {
  //   if (!client.connected()) reconnect();
  //   else client.loop();
  // }


  // Kết nối lại MQTT nếu rớt mạng mỗi 5s
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      unsigned long now = millis();
      // Chỉ thử kết nối lại sau mỗi 5 giây, không đợi kết nối ngay lập tức
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;
        if (reconnect()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      // MQTT đã kết nối, duy trì việc nhận/gửi dữ liệu
      client.loop();
    }
  }

  // Đọc cảm biến & Xử lý logic CHÁY
  Temp = readTemp();
  if(!oneTimeFlag) { oneMTempAgo = Temp; oneTimeFlag = true; }
  if((counter % 20) == 1) oneMTempAgo = Temp;

  curArea = preArea(Temp);
  fireDetected = fireDetect(oneMTempAgo, Temp, curArea);

  // 1. Cập nhật Màn hình (500ms/lần)
  if (currentTime - lastDisplayTime >= 500) {
    lastDisplayTime = currentTime;
    updateDisplay(Temp, fireDetected, deviceStatus);
  }

  // 2. Gửi MQTT định kỳ hoặc khẩn cấp
  static unsigned long lastMqttSent = 0;
  unsigned long mqttInterval = fireDetected ? 0 : 5000; // Có cháy gửi nhanh hơn
  if (currentTime - lastMqttSent >= mqttInterval) {
    lastMqttSent = currentTime;
    sendDataMQTT(Temp, fireDetected, deviceStatus);
    counter++;
  }

  // 3. Điều khiển thiết bị ngoại vi
  handleBuzzer(fireDetected);
  LedBlink5s();
}