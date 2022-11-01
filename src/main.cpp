#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

#define QuarterPi 0.78539816339
#define HalfPi 1.57079632679
#define ThreeQuarterPi 2.35619449019

unsigned long Ch_time[5];
unsigned long A_time;
int value[6];
bool send = 0;
// uint8_t broadcastAddress[][6] = {0xCC, 0x50, 0xE3, 0x56, 0xAD, 0xF4}; // Alley
// uint8_t broadcastAddress[][6] = {};
uint8_t broadcastAddress[][6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast
// uint8_t broadcastAddress[][6] = {0x48, 0x55, 0x19, 0x00, 0x4A, 0xAF}; // D1?

unsigned int period = 40;
/*
bool Effect[4][10]= {
  {},//R1
  {0,1,0,0,1,0,0,0,0,0},//B1
  {},//R2
  {0,0,0,0,0,0,1,0,0,1},//B2
};*/
/// Padrão de piscada dos leds
bool Effect[4][19] = {
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // R1
    {},                                                        // B1
    {},                                                        // R2
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, // B2
};

/// Estrutura de comunicação EspNow
typedef struct struct_message
{
  /// Valor entre -255 e 255
  signed int motor_e;
  /// Valor entre -255 e 255
  signed int motor_d;
  /// Valor entre 1000ms e 2000ms
  int arma_vel;
  /// Primeiro indice é o led, o segundo são os valores de RGB (0-255 cada)
  byte led[2][3];
} struct_message;

struct_message Controle;

void IRAM_ATTR CH0();
void IRAM_ATTR CH1();
void IRAM_ATTR CH2();
void IRAM_ATTR CH3();
void IRAM_ATTR CH4();
void IRAM_ATTR CH5();

void CH0()
{
  unsigned long time = micros();
  if (digitalRead(12))
  {
    Ch_time[0] = time;
    send = 1;
  }
  else if (Ch_time[0] < time)
    value[0] = map(constrain(time - Ch_time[0], 1000, 2000), 1000, 2000, 0, 255);
}
void CH1()
{
  unsigned long time = micros();
  if (digitalRead(13))
    Ch_time[1] = time;
  else if (Ch_time[1] < time)
    value[1] = map(constrain(time - Ch_time[1], 1000, 2000), 1000, 2000, 0, 255);
}
void CH2()
{
  unsigned long time = micros();
  if (digitalRead(14))
    Ch_time[2] = time;
  else if (Ch_time[2] < time)
    value[2] = map(constrain(time - Ch_time[2], 1000, 2000), 1000, 2000, 0, 255);
}
void CH3()
{
  unsigned long time = micros();
  if (digitalRead(4))
    Ch_time[3] = time;
  else if (Ch_time[3] < time)
    value[3] = map(constrain(time - Ch_time[3], 1000, 2000), 1000, 2000, 0, 255);
}
void CH4()
{
  unsigned long time = micros();
  if (digitalRead(5))
    Ch_time[4] = time;
  else if (Ch_time[4] < time)
    value[4] = map(constrain(time - Ch_time[4], 1000, 2000), 1000, 2000, 0, 255);
}

/// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    Serial.println("Delivery success");
  }
  else
  {
    Serial.println("Delivery fail");
  }
}
/// Calibrar valores recebidos do controle
void CalibrateValues()
{
  value[0] = constrain(map(value[0], 50, 241, 0, 255), 0, 255);
  value[1] = constrain(map(value[1], 12, 237, -255, 255), -255, 255);
  value[2] = constrain(map(value[2], 15, 233, -255, 255), -255, 255);
  value[3] = constrain(map(value[3], 8, 236, -255, 255), -255, 255);
  value[4] = constrain(map(value[4], 0, 246, 0, 255), 0, 255);
}
/// Salva valores convertidos do motor e corrige caso robo capote
void Save2send(float Angle, int vel)
{
  if (value[4]) // Testa se robo capotou
  {
    Controle.motor_d = vel * sin(HalfPi * cos(Angle - ThreeQuarterPi));
    Controle.motor_e = vel * sin(HalfPi * cos(Angle - QuarterPi));
  }
  else
  {
    Controle.motor_e = -vel * sin(HalfPi * cos(Angle - ThreeQuarterPi));
    Controle.motor_d = -vel * sin(HalfPi * cos(Angle - QuarterPi));
  }
}
/// Converter valores para direção do motor
void ConverterMotores(signed int X, signed int Y)
{

  if (X != 0)
  {
    double YdX = 1000 * Y / X;
    float Angle = atan(YdX / 1000);
    if (X < 0)
    {
      if (Y > 0)
        Angle += PI;
      else
        Angle -= PI;
    }

    float Module = sqrt(X * X + Y * Y);
    signed int Max_Y = 0;
    signed int Max_X = 0;

    if (abs(X) > abs(Y))
    {
      Max_X = 255;
      Max_Y = 255 * tan(Angle);
    }
    else
    {
      Max_Y = 255;
      Max_X = 255 / tan(Angle);
    }

    int Max_Module = sqrt(Max_X * Max_X + Max_Y * Max_Y);
    int vel = constrain(255 * Module / Max_Module, 0, 255);

    Save2send(Angle, vel);
  }
}
/// DeadZone e Limite de velocidade
void LimitarMotores()
{
  if (abs(Controle.motor_d) > 60)
    Controle.motor_d = map(Controle.motor_d, 0, 255, 0, 200);
  else
    Controle.motor_d = 0;

  if (abs(Controle.motor_e) > 60)
    Controle.motor_e = map(Controle.motor_e, 0, 255, 0, 200);
  else
    Controle.motor_e = 0;
}
/// Blink leds padrão sirene
void PoliceEffectLed()
{
  unsigned int index = (millis() - A_time) / period;

  Controle.led[0][0] = 255 * Effect[0][index];
  Controle.led[0][2] = 255 * Effect[1][index];
  Controle.led[1][0] = 255 * Effect[2][index];
  Controle.led[1][2] = 255 * Effect[3][index];

  if (index >= sizeof(Effect[0]))
    A_time = millis();
}
/// Limitador da velocidade da arma
void ConstrainArma()
{
  if (value[4])
    Controle.arma_vel = constrain(map(value[0], 255, 0, 1500, 1750), 1500, 1750);
  else
    Controle.arma_vel = constrain(map(value[0], 255, 0, 1500, 1250), 1250, 1500);
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0)
    Serial.println("ESPNow Init Failed");

  WiFi.setSleep(false);

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcastAddress[0], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  attachInterrupt(digitalPinToInterrupt(12), CH0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(13), CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(14), CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), CH4, CHANGE);
}

void loop()
{
  if (send)
  {
    delay(5);
    CalibrateValues();
    ConverterMotores(value[2], -value[1]);
    LimitarMotores();
    ConstrainArma();
    PoliceEffectLed();

    send = 0;
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    delay(1);
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    delay(1);
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    delay(1);

    Serial.print(Controle.motor_e);
    Serial.print("  ");
    Serial.print(Controle.motor_d);
    Serial.print("  ");
    Serial.println(Controle.arma_vel);
  }
}
