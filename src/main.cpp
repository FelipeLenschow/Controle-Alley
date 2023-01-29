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
bool Send = 0;
byte FreioCntD, FreioCntE;

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
  /// 1 para ativo 0 caso contrario
  bool FreioE;
  bool FreioD;
} struct_message;

struct_message Controle;
struct_message LastControle;

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
    Send = 1;
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
  X = X == 0 ? 1 : X;
  float Angle = atan((float)Y / (float)X);
  if (X < 0)
  {
    if (Y > 0)
      Angle += PI;
    else
      Angle -= PI;
  }

  float Module = sqrt(X * X + Y * Y);

  float Vel = Module * Module / 255;
  int Max_Vel = (abs(X) > abs(Y)) ? 255 / abs(cos(Angle)) : 255 / abs(sin(Angle));
  int vel = constrain(255 * Vel / Max_Vel, 0, 255);
  Save2send(Angle, vel);
}
/// DeadZone e Limite de velocidade
void LimitarMotores()
{
  if (abs(Controle.motor_d) > 10)
  {
    if (Controle.motor_d > 0)
      Controle.motor_d = map(Controle.motor_d, 0, 255, 40, 255);
    else
      Controle.motor_d = map(Controle.motor_d, 0, -255, -40, -255);
  }
  else
    Controle.motor_d = 0;

  if (abs(Controle.motor_e) > 10)
  {
    if (Controle.motor_e > 0)
      Controle.motor_e = map(Controle.motor_e, 0, 255, 40, 255);
    else
      Controle.motor_e = map(Controle.motor_e, 0, -255, -40, -255);
  }
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
    Controle.arma_vel = constrain(map(value[0], 255, 0, 1500, 2000), 1500, 2000);
  else
    Controle.arma_vel = constrain(map(value[0], 255, 0, 1500, 1000), 1000, 1500);
}
/// Identifica entrada na DeadZone para freiar brevemente
void Freio()
{
  if (!Controle.motor_d)
  {                                                 // É pra parar?
    if (FreioCntD)                                  // Ja começou a freiar?
      if (FreioCntD <= 25)                          // É pra continuar freiando?
        FreioCntD++;                                // Freia e aumenta a contagem
      else                                          //
        FreioCntD = 0;                              // Para e freiar
    else if (LastControle.motor_d)                  // Ainda nao começou a freiar, mas deve?
      FreioCntD = 1;                                // Freia e começa a contagem
  }                                                 //
  else                                              //
    FreioCntD = 0;                                  // Entao é pra continuar andando
  Controle.FreioD = (FreioCntD > 0 ? true : false); // Aplica o freio

  if (!Controle.motor_e)
  {
    if (FreioCntE)
      if (FreioCntE <= 10)
        FreioCntE++;
      else
        FreioCntE = 0;
    else if (LastControle.motor_e)
      FreioCntE = 1;
  }
  else
    FreioCntE = 0;
  Controle.FreioE = (FreioCntE > 0 ? true : false);
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
  if (Send)
  {
    delay(2);
    LastControle = Controle;
    CalibrateValues();
    ConverterMotores(value[2], -value[1]);
    LimitarMotores();
    Freio();
    ConstrainArma();
    PoliceEffectLed();

    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));
    esp_now_send(broadcastAddress[0], (uint8_t *)&Controle, sizeof(Controle));

    Send = 0;

    Serial.print(Controle.motor_e);
    Serial.print("  ");
    Serial.print(Controle.motor_d);
    Serial.print("  ");
    Serial.print(Controle.FreioD);
    Serial.print("  ");
    Serial.print(Controle.FreioE);
    Serial.print("  ");
    Serial.println(Controle.arma_vel);
  }
}
