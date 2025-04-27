// Modified Code with Custom QTR Sensor Replacement and Full Documentation

#include <L298N.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Motor control pins
#define AIN1 21
#define BIN1 25
#define AIN2 22
#define BIN2 33
#define PWMA 23
#define PWMB 32
#define STBY 19

const int offsetA = 1;
const int offsetB = 1;

// Motor driver setup
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Sensor setup
const uint8_t SensorCount = 5;
uint8_t sensorPins[SensorCount] = {26, 27, 14, 12, 13};
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];


float Kp = 10.0; 
float Ki = 5.0;  
float Kd = 2.0; 
uint8_t multiP = 1, multiI = 1, multiD = 1;
float Pvalue, Ivalue, Dvalue;
int previousError = 0, error = 0;

const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";
const char *mqtt_server = "192.168.1.100"; 

WiFiClient espClient;
PubSubClient client(espClient);

boolean onoff = false;
int lsp, rsp;
int lfspeed = 230;

void setup_wifi()
{
    delay(10);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
}

void callback(char *topic, byte *message, unsigned int length)
{
    String msg;
    for (int i = 0; i < length; i++)
    {
        msg += (char)message[i];
    }
    processMessage(msg);
}


void reconnect()
{
    while (!client.connected())
    {
        if (client.connect("ESP32Client"))
        {
            client.subscribe("robot/voice");
        }
        else
        {
            delay(5000);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);


    calibrateSensors();
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    if (onoff)
    {
        robot_control();
    }
    else
    {
        motor1.stop();
        motor2.stop();
    }
}

void calibrateSensors()
{
    for (int j = 0; j < SensorCount; j++)
    {
        sensorMin[j] = 4095;
        sensorMax[j] = 0;
    }

    for (int i = 0; i < 400; i++)
    {
        for (int j = 0; j < SensorCount; j++)
        {
            uint16_t val = analogRead(sensorPins[j]);
            if (val < sensorMin[j])
                sensorMin[j] = val;
            if (val > sensorMax[j])
                sensorMax[j] = val;
        }
        delay(5);
    }
}

void readSensors()
{
    for (int i = 0; i < SensorCount; i++)
    {
        uint16_t val = analogRead(sensorPins[i]);
        val = constrain(val, sensorMin[i], sensorMax[i]);
        sensorValues[i] = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    }
}

uint16_t readLineBlack()
{
    uint32_t avg = 0;
    uint16_t sum = 0;

    readSensors();

    for (int i = 0; i < SensorCount; i++)
    {
        avg += (uint32_t)sensorValues[i] * (i * 1000);
        sum += sensorValues[i];
    }
    if (sum == 0)
        return 2000;
    return avg / sum;
}

void robot_control()
{
    uint16_t position = readLineBlack();
    error = 2000 - position;

    while (allSensorsBlack())
    {
        if (previousError > 0)
        {
            motor_drive(-230, 230);
        }
        else
        {
            motor_drive(230, -230);
        }
        position = readLineBlack();
    }

    PID_Linefollow(error);
}

void PID_Linefollow(int error)
{
    static int I = 0;

    int P = error;
    I += error;
    int D = error - previousError;

    Pvalue = (Kp / pow(10, multiP)) * P;
    Ivalue = (Ki / pow(10, multiI)) * I;
    Dvalue = (Kd / pow(10, multiD)) * D;

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    lsp = constrain(lsp, -255, 255);
    rsp = constrain(rsp, -255, 255);

    motor_drive(lsp, rsp);
}

void processMessage(String msg)
{
    if (msg == "start")
    {
        onoff = true;
    }
    else if (msg == "stop")
    {
        onoff = false;
    }
}

void motor_drive(int left, int right)
{
    if (right > 0)
    {
        motor2.setSpeed(right);
        motor2.forward();
    }
    else
    {
        motor2.setSpeed(abs(right));
        motor2.backward();
    }

    if (left > 0)
    {
        motor1.setSpeed(left);
        motor1.forward();
    }
    else
    {
        motor1.setSpeed(abs(left));
        motor1.backward();
    }
}

bool allSensorsBlack()
{
    for (int i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] < 900)
            return false;
    }
    return true;
}
