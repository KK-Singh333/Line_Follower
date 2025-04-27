#include <L298N.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Motor control pins
#define AIN1 10
#define BIN1 07
#define AIN2 09
#define BIN2 07
#define PWMA 11
#define PWMB 06

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Sensors
const uint8_t SensorCount = 4;
uint8_t sensorPins[SensorCount] = {12, 13, 15, 16};
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

// PID constants
float Kp = 10.0, Ki = 5.0, Kd = 2.0;
float Pvalue, Ivalue, Dvalue;
int previousError = 0, error = 0;

// WiFi and MQTT
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_PC_IP";

WiFiClient espClient;
PubSubClient client(espClient);

boolean onoff = true;   // <-- default ON
boolean wifiAvailable = false;  // <-- new flag
int lsp, rsp;
int lfspeed = 230;

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) {  // wait max 5 sec
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiAvailable = true;
  } else {
    wifiAvailable = false;
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }
  processMessage(msg);
}

void reconnect() {
  if (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe("robot/voice");
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();

  if (wifiAvailable) {
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  } else {
    onoff = true;  // keep robot active if no WiFi
  }

  calibrateSensors();
}

void loop() {
  if (wifiAvailable) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  if (onoff) {
    robot_control();
  } else {
    motor1.stop();
    motor2.stop();
  }
}

void calibrateSensors() {
  for (int i = 0; i < SensorCount; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  for (int i = 0; i < 400; i++) {
    for (int j = 0; j < SensorCount; j++) {
      uint16_t val = analogRead(sensorPins[j]);
      if (val < sensorMin[j]) sensorMin[j] = val;
      if (val > sensorMax[j]) sensorMax[j] = val;
    }
    delay(5);
  }
}

void readSensors() {
  for (int i = 0; i < SensorCount; i++) {
    uint16_t val = analogRead(sensorPins[i]);
    val = constrain(val, sensorMin[i], sensorMax[i]);
    sensorValues[i] = map(val, sensorMin[i], sensorMax[i], 0, 1000);
  }
}

uint16_t readLineBlack() {
  uint32_t avg = 0;
  uint16_t sum = 0;
  readSensors();

  for (int i = 0; i < SensorCount; i++) {
    avg += (uint32_t)sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }
  if (sum == 0) return 1500;
  return avg / sum;
}

void robot_control() {
  uint16_t position = readLineBlack();
  error = 1500 - position;

  while (allSensorsBlack()) {
    if (previousError > 0) {
      motor_drive(-230, 230);
    } else {
      motor_drive(230, -230);
    }
    position = readLineBlack();
  }

  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  static int I = 0;
  int P = error;
  I += error;
  int D = error - previousError;

  Pvalue = (Kp / 10.0) * P;
  Ivalue = (Ki / 10.0) * I;
  Dvalue = (Kd / 10.0) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void processMessage(String msg) {
  if (msg == "start") {
    onoff = true;
  } else if (msg == "stop") {
    onoff = false;
  }
}

void motor_drive(int left, int right) {
  if (right > 0) {
    motor2.setSpeed(right);
    motor2.forward();
  } else {
    motor2.setSpeed(abs(right));
    motor2.backward();
  }

  if (left > 0) {
    motor1.setSpeed(left);
    motor1.forward();
  } else {
    motor1.setSpeed(abs(left));
    motor1.backward();
  }
}

bool allSensorsBlack() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 900)
      return false;
  }
  return true;
}
