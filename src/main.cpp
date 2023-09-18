// Gateway LORA-Test

#include <Arduino.h>
#include <ArduinoJson.h> // JSON library for Arduino
#include <LoRa.h>        // Library for LoRa communication
#include <Wire.h>        // I2C communication library

// Define Pin Configurations
#define SerialMon Serial
#define UART_BAUD 115200
#define ss 15
#define rst 14
#define dio0 13

// Global variables
int packetRssi;

// LoRa configuration
int TxPower;
double interval;

// LoRa test configuration
long freq = 923E6;
int SyncWord = 0xF1;
int spreadingFactor = 12;
long signalBandwidth = 125E3;
int trialtimes = 20;

bool lorafind()
{
  while (true)
  {
    int packetSize = LoRa.parsePacket();
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

        //   if (dataIndex >= sizeof(LoRaData) - 1)
        //   {
        //     LoRaData[dataIndex] = '\0';
        //     break;
        //   }

        //   if (dataIndex == 1 && receivedChar != '{')
        //   {
        //     dataIndex = 0;
        //     break;
        //   }
      }

      Serial.print("' with RSSI ");
      Serial.print(LoRa.packetRssi());

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
    // Serial.println("\n-------------------------------------------------------------------------------------------------------------");
    Serial.print("\nPacket #");
    Serial.print(i + 1);
    Serial.print(" ");

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
  delay(5000);
  unsigned long startTime = millis();
  SerialMon.begin(UART_BAUD);

  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(freq))
  {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(SyncWord);
  LoRa.setSpreadingFactor(spreadingFactor);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.enableCrc();

  // Initialize LoRa module
  Serial.print("SyncWord: ");
  Serial.println(SyncWord, HEX);
  Serial.print("Frequency: ");
  Serial.println(freq);
  Serial.print("SpreadingFactor: ");
  Serial.println(spreadingFactor);
  Serial.print("SignalBandwidth: ");
  Serial.println(signalBandwidth);

  Serial.println("LoRa Initializing OK!");
  delay(1000);
}

void loop()
{
  Serial.println("RSSI Test from " + String(trialtimes) + " values");

  float RSSI = sniavg(trialtimes);
  Serial.println("-----------------------------------------------------------------------------------------------------------------");
  Serial.print("Average of RSSI from " + String(trialtimes) + " values: ");
  Serial.println(RSSI);

  Serial.println("\n\n\n");
}