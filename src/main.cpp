//Gateway

#include <Arduino.h>
#include <ArduinoJson.h>               // JSON library for Arduino
#include <TinyGsmClientSIM7600.h>      // Library for GSM communication using SIM7600 module
#include <LoRa.h>                      // Library for LoRa communication
#include <Wire.h>                      // I2C communication library
#include <LiquidCrystal_I2C.h>         // Library for driving I2C-connected LCD display

// Initialize the LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);

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
#define ss 14
#define rst 13
#define dio0 2

// Global variables
bool DEBUG = true;
String response, LoRaData, lattext, lontext;
float temp;
int humi;
String device_id = "Node1";
double sleepmin = 1;

void modemPowerOn() {
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

String sendAT(String command, int interval, boolean debug) {
  String response = "";

  SerialAT.println(command);

  long int startTime = millis();

  while (((millis() - startTime)) < interval) {
    while (SerialAT.available() > 0) {
      int readData = SerialAT.read();
      response += char(readData);
    }
  }

  SerialAT.flush();

  if (debug) {
    SerialMon.print(response);
  }

  return response;
}

float convertCoordinate(String coordString) {
  float degrees = 0.0;

  if (coordString.length() > 10) {
    // Extract degrees part
    String degreesString = coordString.substring(0, 3);
    int degreesValue = degreesString.toInt();

    // Extract minutes part
    String minutesString = coordString.substring(3);
    float minutesValue = minutesString.toFloat();

    // Calculate decimal degrees
    degrees = degreesValue + (minutesValue / 60.0);
  } else {
    // Extract degrees part
    String degreesString = coordString.substring(0, 2);
    int degreesValue = degreesString.toInt();

    // Extract minutes part
    String minutesString = coordString.substring(2);
    float minutesValue = minutesString.toFloat();

    // Calculate decimal degrees
    degrees = degreesValue + (minutesValue / 60.0);
  }

  // Print the converted value
  Serial.print("Decimal Degrees: ");
  Serial.println(degrees, 6); // Print with 6 decimal places
  return degrees;
}

void updatelcd(const String & text, int row) {
  // lcd.clear();
  lcd.setCursor(0, row);
  lcd.print("                "); // Clear the row
  lcd.setCursor(0, row);
  lcd.print(text); // Print the new text
}

void request() {
  bool DEBUG = true;
  SerialMon.println("\n----------   Start of sendrequest()   ----------\n");

  String http_str = "AT+HTTPPARA=\"URL\",\"http://rung.ddns.net:1142/data?id=" +
                    device_id +
                    "&temp=" + temp +
                    "&humi=" + humi +
                    "&lat=" + lattext +
                    "&lon=" + lontext + "\"\r\n";

  Serial.println(http_str);
  sendAT("AT+HTTPINIT", 3000, DEBUG);
  sendAT(http_str, 5000, DEBUG);
  sendAT("AT+HTTPACTION=0", 5000, DEBUG);
  sendAT("AT+HTTPTERM", 5000, DEBUG);

  delay(2000);
  SerialMon.println("\n----------   End of sendrequest()   ----------\n");
}

void connect2LTE() {
  SerialMon.println("\n----------   Start of connect2LTE()   ----------\n");
  boolean DEBUG = 1;

  delay(1000);
  sendAT("AT+NETCLOSE", 8000, DEBUG);

  delay(1000);
  sendAT("AT+CPIN?", 5000, DEBUG);

  delay(1000);
  sendAT("AT+CSOCKSETPN=1", 5000, DEBUG);
  sendAT("AT+NETOPEN", 5000, DEBUG);
  sendAT("AT+IPADDR", 5000, DEBUG);

  delay(1000);
  sendAT("AT+CPING=\"rung.ddns.net\",1,4", 10000, DEBUG);
  SerialMon.println("\n----------   End of connect2LTE()   ----------\n");
}

void processJsonInput(const char * jsonInput) {
  StaticJsonDocument < 64 > doc;
  DeserializationError error = deserializeJson(doc, jsonInput);

  if (error) {
    Serial.print("Parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  temp = doc["Temperature"];
  humi = doc["Humidity"];
}

void setup() {
  unsigned long startTime = millis();
  SerialMon.begin(UART_BAUD);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xF1);
  Serial.println("LoRa Initializing OK!");

  lcd.init();
  lcd.backlight();

  modemPowerOn();
  delay(1000);

  for (int i = 0; i < 2; i++) {
    // Send the AT command to enable GNSS
    String response = sendAT("AT+CGNSSPWR=1", 5000, DEBUG);
    updatelcd(response.c_str(), 1);

    delay(2000); // Delay before the next command

    // Send the AT command to retrieve GNSS info
    response = sendAT("AT+CGNSSINFO?", 5000, DEBUG);
    updatelcd(response.c_str(), 1);

    delay(2000); // Delay before the next iteration
  }
}

void loop() {

  int packetSize = LoRa.parsePacket();
  while (packetSize) {
    if (packetSize) {
      Serial.print("Received packet '");

      char LoRaData[255];
      int dataIndex = 0;

      while (LoRa.available()) {
        char receivedChar = LoRa.read();
        Serial.print(receivedChar);

        LoRaData[dataIndex] = receivedChar;
        dataIndex++;

        if (dataIndex >= sizeof(LoRaData) - 1) {
          LoRaData[dataIndex] = '\0';
          break;
        }

        if (dataIndex == 1 && receivedChar != '{') {
          dataIndex = 0;
          break;
        }
      }

      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());

      if (dataIndex > 0) {
        LoRaData[dataIndex] = '\0';
        processJsonInput(LoRaData);
        delay(100);
      }
    }
  }

  for (int i = 0; i < 20; i++) {
    // response = sendAT("AT+CGNSSINFO", 6000, DEBUG);
    response = "3,11,,00,00,1337.36458,N,10039.47833,E,140823,071209.00,-13.7,5.680,325.76,6.9,3.2,6.0";
    Serial.println(response);

    if (response.indexOf(",N") != -1) {
      int latStart = response.indexOf(",") + 1;
      latStart = response.indexOf(",", latStart) + 1;
      latStart = response.indexOf(",", latStart) + 1;
      latStart = response.indexOf(",", latStart) + 1;
      latStart = response.indexOf(",", latStart) + 1;
      int longStart = response.indexOf(",", latStart) + 1;
      longStart = response.indexOf(",", longStart) + 1;

      String latitude = response.substring(latStart, response.indexOf(",", latStart));
      String longitude = response.substring(longStart, response.indexOf(",", longStart));

      float lat = convertCoordinate(latitude);
      String latText = String(lat, 6);
      float lon = convertCoordinate(longitude);
      String lonText = String(lon, 6);

      Serial.print(latText);
      Serial.println(lonText);
      updatelcd(latText, 0);
      updatelcd(lonText, 1);
    } else {
      int startIndex = response.indexOf(",") + 1;
      int endIndex = response.lastIndexOf(",");
      String relevantInfo = response.substring(startIndex, endIndex);
      relevantInfo.replace(",", "0");
      updatelcd(relevantInfo.c_str(), 1);
      updatelcd("No data", 0);
    }
    delay(100);
  }

  connect2LTE();
  request();
}