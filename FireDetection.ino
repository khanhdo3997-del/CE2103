#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

/* ------------------------------ MACRO ------------------------------ */
#define ADC_MIN_ERROR 50    // Ngưỡng dưới (ngắn mạch)
#define ADC_MAX_ERROR 4050  // Ngưỡng trên (hở mạch)
#define LED_PIN_FIRE 16
#define LED_PIN 17
#define BUZZER_PIN 18
#define ANALOG_PIN_1 35
#define ANALOG_PIN_2 34
#define FALSE 0
#define TRUE 1
#define R_FIXED 10000.0 // Điện trở 10k trong NTC
#define BETA 3950       // Hằng số nhiệt của NTC
#define T0 298.15       // Nhiệt độ chuẩn Kelvin
#define R0 10000.0      // Điện trở NTC tại 25°C (điểm chuẩn ban đầu với mốc 25 độ nhiệt độ phòng)
#define NORMALAREA 0
#define HOTAREA 1
#define KITCHENAREA 2

/* ------------------------------ GLOBAL VARIABLE ------------------------------ */
/* Type define */
enum deviceStatusType
{
  NORMAL = 0,
  SENSOR_1_ERR,
  SENSOR_2_ERR,
  ERROR
};

/*===== WIFI SETUP=====*/
const char* ssid = "Duy Khanh";
const char* password = "112345678";
const char* mqtt_server = "broker.hivemq.com";
/*====*/

/* Globle variables */
char Area;
char curArea = 0;
static bool fireDetected;
static bool oneTimeFlag = 0;
static int analogSensor_1 = 0;
static int analogSensor_2 = 0;
deviceStatusType deviceStatus = NORMAL;
int counter = 0;
float Temp;
float oneMTempAgo;
const int ADC_RES = 4095;
const float VCC = 3.3;
unsigned long currentTime = 0;
unsigned long lastTime = 0;   // Lưu thời điểm cuối cùng tăng counter
unsigned long interval = 5000; /* 5s - 5,000 milliseconds */

WiFiClient espClient;
PubSubClient client(espClient);
// ===== API =====
const char* serverName = "http://192.168.1.8:3000/api/data";

/* ------------------------------ SET UP ------------------------------ */
void setup() {
  /* initialize serial communication at 115200 bits per second */
  Serial.begin(115200);

  /* CONFIG */
  #define NODE_ID   "NODE_01"
  #define TEMP_THRESHOLD 50.0

  /* WIFI SETUP */
  setupWiFi();
  client.setServer(mqtt_server, 1883);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_FIRE, OUTPUT);
}

void loop() {
  /* Get current time */
  currentTime = millis();

    /* Read current temp */
    Temp = readTemp();

    /* Init */
    if(oneTimeFlag == 0)
    {
      oneMTempAgo = Temp;
      oneTimeFlag = 1;
    }
    else
    {
      /* Do nothing */
    }

    if((counter%12) == 1)
    {
      oneMTempAgo = Temp;
      counter = 0;
    }
    else
    {
      /* Do nothing */
    }

    /* Predict area */
    curArea = preArea(Temp);

    /* Fire detect */
    fireDetected = fireDetect(oneMTempAgo, Temp, curArea);

    /* Fire warning */ 
    if(fireDetected == TRUE)
    {
      Serial.printf("Fire detected\n");

      /* Sent data to server */
      sendDataMQTT(Temp, fireDetected, deviceStatus);

      /* Buzzer ON */
      buzzFireWarning();

      /* FIRE Led - switch state */
      FireLed(TRUE);
    }
    else
    {
      /* FIRE Led - switch state */
      FireLed(FALSE);
    }

    if (!client.connected()) {
      reconnect();
    }
    // client.loop();



  /* Check if 5 seconds have passed */
  if (currentTime - lastTime >= interval)
  {
    /* Update last time */
    lastTime = currentTime;
    /* Sent data to server */
    sendDataMQTT(Temp, fireDetected, deviceStatus);
    /* Incease the counter */
    counter++;
  }
  else
  {
    /* Do nothing */
  }
  
  /* Led blink */
  LedBlink5s(deviceStatus);
}

char preArea(float Temp)
{
  Area = NORMALAREA;
  if((Temp >= 20) && (Temp <= 30))
  {
    /* Do nothing */
  }
  else if((Temp >= 30) && (Temp <= 40))
  {
    Area = HOTAREA;
  }
  else if((Temp >= 40) && (Temp <= 55))
  {
    Area = KITCHENAREA;
  }
  else
  {
    /* Do nothing */
  }

  return Area;
}

float readTemp(void)
{
  float Temp;

  /* Read value of sensor 1 */
  int analogSensor_1 = analogRead(ANALOG_PIN_1);
  /* Read value of sensor 2 */
  int analogSensor_2 = analogRead(ANALOG_PIN_2);

  /* Sensor 1 check */
  if ((analogSensor_1 == 0) || (analogSensor_1 < (-10)) || (analogSensor_1 < ADC_MIN_ERROR) || (analogSensor_1 > ADC_MAX_ERROR))
  {
    /* Set status */
    deviceStatus = SENSOR_1_ERR;
    
    
    /* Sensor 2 check */
    if ((analogSensor_2 == 0) || (analogSensor_2 < (-10)) || (analogSensor_2 < ADC_MIN_ERROR) || (analogSensor_2 > ADC_MAX_ERROR))
    {
      /* Set status */
      deviceStatus = ERROR;
    }
    else 
    {
      Temp = convertToCelsius(analogSensor_2); // Hàm chuyển đổi ADC -> độ C
    }
  } 
  else
  {
    /* Sensor 2 check */
    if ((analogSensor_2 == 0) || (analogSensor_2 < (-10)) || (analogSensor_2 < ADC_MIN_ERROR) || (analogSensor_2 > ADC_MAX_ERROR))
    {
      /* Set status */
      deviceStatus = SENSOR_2_ERR;
    }
    else
    {
      /* Do nothing */
    }

    /* Read value of sensor 1 */
    Temp = convertToCelsius(analogSensor_1);
  }

  return Temp;
}

float convertToCelsius(float analogSensor)
{
  float tempC = 0;

  /* Convert ADC to volt*/
  float voltage = analogSensor * VCC / ADC_RES;

  /* Calculate register*/
  float R_ntc = R_FIXED * (VCC / voltage - 1);

  /* Beta */
  float tempK = 1.0 / ( (1.0 / T0) + (1.0 / BETA) * log(R_ntc / R0) );
  tempC = tempK - 273.15;

  return tempC;
}

bool fireDetect(float oneMTempAgo, float curTemp, char Area)
{
  bool ret;
  float rate;
  ret = FALSE;

  /* Check for input argument */
  if(Area > 2)
  {
    /* Do nothing */
  }
  else
  {
    rate = curTemp - oneMTempAgo;
    if(rate >= 8)
    {
      ret = TRUE;
    }
    else
    {
      switch(Area)
      {
        /* Fire detection for normal area */
        case NORMALAREA:
          if(curTemp > 54)
          {
            ret = TRUE;
          }
          else
          {
            /* Do nothing */
          }
          break;
        /* Fire detection for hot area */
        case HOTAREA:
          if(curTemp > 69)
          {
            ret = TRUE;
          }
          else
          {
            /* Do nothing */
          }
          break;
        /* Fire detection for kitchen area */
        case KITCHENAREA:
          if(curTemp > 84)
          {
            ret = TRUE;
          }
          else
          {
            /* Do nothing */
          }
          break;

        default:
          /* Do nothing */
          break;
      }
    }
  }

  return ret;
}

/* Function to setup Wifi*/
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
}

/* Reconnect */ 
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client"))
    {
      /* Connected */ 
    }
    else
    {
      delay(2000);
    }
  }
}

/* Send data to MQTT */
void sendDataMQTT(float temp, bool fireStatus, deviceStatusType deviceStatus)
{
  String payload = "{";
  payload += "\"node_id\":\"NODE_01\",";
  payload += "\"temperature\":" + String(temp, 2) + ",";
  payload += "\"fire_status\":\"" + String(fireStatus ? "FIRE" : "NORMAL") + "\",";
  payload += "\"device_status\":\"" + String(deviceStatus) + "\",";
  payload += "}";

  Serial.println("Payload gửi:");
  Serial.println(payload);  

  client.publish("CE2103/firedetect", payload.c_str());
}

void buzzFireWarning() {
  static unsigned long lastTime = 0;
  static bool buzzerState = false;

  unsigned long currentTime = millis();

    /* Beeps once every 5 seconds */
    if (currentTime - lastTime >= 5000)
    {
      lastTime = currentTime;
      buzzerState = true;
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else
    {
      /* Do nothing */
    }
    /* Sounds for 500ms */
    if (buzzerState && currentTime - lastTime >= 500)
    {
      buzzerState = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
    else
    {
      /* Do nothing */
    }
}

void LedBlink5s(bool fireDetected)
{
  if(fireDetected != TRUE)
  {
    static unsigned long lastTime = 0;
    static bool ledState = false;

    unsigned long currentTimeLed = millis();

    /* Turn on every 5 seconds */
    if (!ledState && currentTimeLed - lastTime >= 5000) {
      ledState = true;
      lastTime = currentTimeLed;
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      /* Do nothing */
    }

    /* Turn off after 1 seconds */
    if (ledState && currentTimeLed - lastTime >= 1000) {
      ledState = false;
      lastTime = currentTimeLed;
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      /* Do nothing */
    }
  }
}

void FireLed(bool state) {
  if(state == TRUE)
  {
    digitalWrite(LED_PIN_FIRE, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN_FIRE, LOW);
  }
}