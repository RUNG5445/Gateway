// Gateway

#include <Arduino.h>
#include <ArduinoJson.h>          // JSON library for Arduino
#include <TinyGsmClientSIM7600.h> // Library for GSM communication using SIM7600 module
#include <LoRa.h>                 // Library for LoRa communication
#include <Wire.h>                 // I2C communication library

// Define Pin Configurations
#define SerialAT Serial1
#define SerialMon Serial
#define MIN_SPACE 500000
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 26
#define PIN_RX 27
#define PWR_PIN 4
#define PIN_RST 5
#define PIN_SUP 12
#define BAT_ADC 35
#define ss 15
#define rst 14
#define dio0 13

// Global variables
bool DEBUG = true;
String response, LoRaData, latText, lonText, gpsinfo;
float temp;
int humi;
String NodeName;
double sleepmin = 1;
int packetRssi;

// LoRa configuration
int SyncWord;
int TxPower;
long freq;
double interval;
int sendSyncWord = 5;

void modemPowerOn()
{
  const int SUPPLY_PIN = PIN_SUP;
  const int RESET_PIN = PIN_RST;
  const int POWER_PIN = PWR_PIN;

  pinMode(SUPPLY_PIN, OUTPUT);
  digitalWrite(SUPPLY_PIN, HIGH);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  delay(3000);
  digitalWrite(RESET_PIN, LOW);

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  delay(100);
  digitalWrite(POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(POWER_PIN, LOW);
}

String sendAT(String command, int interval, boolean debug)
{
  String response = "";

  SerialAT.println(command);

  long int startTime = millis();

  while (((millis() - startTime)) < interval)
  {
    while (SerialAT.available() > 0)
    {
      int readData = SerialAT.read();
      response += char(readData);
    }
  }

  SerialAT.flush();

  if (debug)
  {
    SerialMon.print(response);
  }

  return response;
}

bool lorafind()
{
  while (true)
  {
    int packetSize = LoRa.parsePacket();
    // Serial.print("00000");
    if (packetSize)
    {
      Serial.print("Received packet '");

      char LoRaData[255];
      int dataIndex = 0;

      while (LoRa.available())
      {
        char receivedChar = LoRa.read();
        Serial.print(receivedChar);

        LoRaData[dataIndex] = receivedChar;
        dataIndex++;

        if (dataIndex >= sizeof(LoRaData) - 1)
        {
          LoRaData[dataIndex] = '\0';
          break;
        }

        if (dataIndex == 1 && receivedChar != '{')
        {
          dataIndex = 0;
          break;
        }
      }

      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());

      if (dataIndex > 0)
      {
        LoRaData[dataIndex] = '\0';
        packetRssi = LoRa.packetRssi();
        return true;
      }
    }
  }
}

float calculateAverage(int arrayofval[], int sizes)
{
  float sum = 0;

  for (int i = 0; i < sizes; i++)
  {
    sum += arrayofval[i];
  }

  return sum / sizes;
}

float sniavg(int times)
{
  int Rssi[times];

  for (int i = 0; i < times; i++)
  {
    Serial.println("\n-------------------------------------------------------------------------------------------------------------");
    Serial.print("\nPacket #");
    Serial.println(i + 1);

    if (lorafind())
    {
      Rssi[i] = packetRssi;
    }
  }
  float rssiavg = calculateAverage(Rssi, times);
  Serial.println("\n----------------------------------------------- All values -------------------------------------------------");
  for (int i = 0; i < times; i++)
  {
    Serial.print("RSSI Test # ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(Rssi[i]);
  }
  return rssiavg;
}

void setup()
{
  unsigned long startTime = millis();
  SerialMon.begin(UART_BAUD);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  LoRa.setPins(ss, rst, dio0);
  LoRa.setSyncWord(0xF1);
  while (!LoRa.begin(923E6))
  {
    Serial.println(".");
    delay(500);
  }

  Serial.println("LoRa Initializing OK!");
  delay(1000);
}

void loop()
{
  Serial.println("RSSI Test");
  LoRa.setSyncWord(0xF1);
  float RSSI = sniavg(10);
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.print("Average of RSSI : ");
  Serial.println(RSSI);

  Serial.println("\n\n\n");
}