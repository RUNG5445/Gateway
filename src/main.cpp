// Gateway

#include <Arduino.h>
#include <ArduinoJson.h>          // JSON library for Arduino
#include <TinyGsmClientSIM7600.h> // Library for GSM communication using SIM7600 module
#include <LoRa.h>                 // Library for LoRa communication
#include <Wire.h>                 // I2C communication library

#define SerialAT Serial1
#define SerialMon Serial

TinyGsmSim7600 modem(SerialAT);
TinyGsmSim7600::GsmClientSim7600 client(modem);

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
#define apiKey (String) "pk.71031a62fba9814c0898ae766b971df1"

// Global variables
bool DEBUG = true;
String response, LoRaData, latText, lonText, gpsinfo;
float temp;
int humi;
String NodeName;
unsigned long startTime;

// LoRa configuration
int SyncWord;
int TxPower;
long freq;
double interval;
int sendSyncWord = 5;

// Network information
struct NETWORK_INFO
{
  // Location
  String date;
  String time;
  String lat;
  String lon;

  // Cell site information
  char type[10];
  char mode[10];
  String mcc;
  String mnc;
  int lac = 0;
  String cid;
  char freq_b[15];
  double rsrq = 0;
  double rsrp = 0;
  double rssi = 0;
  int rssnr;
};
NETWORK_INFO networkinfo;

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

float convertCoordinate(String coordString)
{
  float degrees = 0.0;

  if (coordString.length() > 10)
  {
    String degreesString = coordString.substring(0, 3);
    int degreesValue = degreesString.toInt();

    String minutesString = coordString.substring(3);
    float minutesValue = minutesString.toFloat();

    degrees = degreesValue + (minutesValue / 60.0);
  }
  else
  {
    String degreesString = coordString.substring(0, 2);
    int degreesValue = degreesString.toInt();

    String minutesString = coordString.substring(2);
    float minutesValue = minutesString.toFloat();

    degrees = degreesValue + (minutesValue / 60.0);
  }

  Serial.print("Decimal Degrees: ");
  Serial.println(degrees, 6);
  return degrees;
}

void readcellinfo()
{
  SerialMon.println("\n----------   Start of readcellinfo()   ----------\n");
  // Send AT command to retrieve cell info
  String info = sendAT("AT+CPSI?", 10000, 1);

  // Process the response
  int startIndex = info.indexOf("+CPSI: ");
  info = info.substring(startIndex + 7, startIndex + 80);
  info.replace("\r", "");
  info.replace("\n", "");
  info.replace("AT+CLBS=4ERROR", "");
  info.replace("AT+CPSI?+CPSI: NO SERVICE", "");

  // Extract the values from the response
  int commaIndex = 0;
  int lastCommaIndex = 0;
  String values[15];
  for (int i = 0; i < 15; i++)
  {
    commaIndex = info.indexOf(',', lastCommaIndex);
    String temp = info.substring(lastCommaIndex, commaIndex);
    values[i] = temp;
    lastCommaIndex = commaIndex + 1;
  }

  // Convert and store the relevant values in the networkinfo struct
  int lacDec = (int)strtol(values[3].c_str(), NULL, 16);
  networkinfo.rssnr = values[13].toInt();
  networkinfo.mcc = values[2].substring(0, 3);
  networkinfo.mnc = values[2].substring(4, 6);
  networkinfo.lac = lacDec;
  networkinfo.cid = values[4];
  delay(1000);
  Serial.println("Base station info");
  Serial.println("RSSNR : " + String(networkinfo.rssnr));
  Serial.println("MCC   : " + networkinfo.mcc);
  Serial.println("MNC   : " + networkinfo.mnc);
  Serial.println("LAC   : " + String(networkinfo.lac));
  Serial.println("Cell id  : " + networkinfo.cid);
  SerialMon.println("\n----------   End of readcellinfo()   ----------\n");
}

void sendrequest()
{
  SerialMon.println("\n----------   Start of sendrequest()   ----------\n");
  String payload = "{\"token\":\"" + apiKey + "\",\"radio\":\"lte\",\"mcc\":" + networkinfo.mcc + ",\"mnc\":" + networkinfo.mnc + ",\"cells\":[{\"lac\":" + networkinfo.lac + ",\"cid\":" + networkinfo.cid + ",\"psc\":0}],\"address\":1}";
  String response;
  client.connect("ap1.unwiredlabs.com", 80);
  String request = "POST /v2/process.php HTTP/1.1\r\n";
  request += "Host: ap1.unwiredlabs.com\r\n";
  request += "Content-Type: application/x-www-form-urlencoded\r\n";
  request += "Content-Length: ";
  request += String(payload.length());
  request += "\r\n\r\n";
  request += payload;
  client.print(request);
  while (client.connected())
  {
    while (client.available())
    {
      char c = client.read();
      response += c;
      client.write(c);
    }
  }
  client.stop();
  Serial.println("Response =" + response);
  int startIndex = response.indexOf("\"lat\":");
  int endIndex = response.indexOf(",\"lon\":");
  latText = response.substring(startIndex + 6, endIndex);
  startIndex = endIndex + 7;
  endIndex = response.indexOf(",\"accuracy\":");
  lonText = response.substring(startIndex, endIndex);
  Serial.println("Latitude: " + latText);
  Serial.println("Longitude: " + lonText);
  SerialMon.println("\n----------   End of sendrequest()   ----------\n");
}

void request()
{
  bool DEBUG = true;
  SerialMon.println("\n----------   Start of sendrequest()   ----------\n");

  String http_str = "AT+HTTPPARA=\"URL\",\"https://tuz1jwn73m.execute-api.ap-southeast-1.amazonaws.com/data?"
                    "nodename=" +
                    NodeName +
                    "&temperature=" + temp +
                    "&humidity=" + humi +
                    "&latitude=" + latText +
                    "&longitude=" + lonText + "\"\r\n";

  Serial.println(http_str);
  sendAT("AT+HTTPINIT", 2000, DEBUG);
  sendAT(http_str, 2000, DEBUG);
  sendAT("AT+HTTPACTION=0", 6000, DEBUG);
  sendAT("AT+HTTPTERM", 2000, DEBUG);

  delay(2000);
  SerialMon.println("\n----------   End of sendrequest()   ----------\n");
}

String getJsonConfig()
{
  bool DEBUG = true;
  SerialMon.println("\n---------- Start of sendRequest() ----------\n");
  String jsonConfig;

  for (int i = 0; i < 5; i++)
  {
    if (jsonConfig.indexOf('{') != -1)
    {
      break;
    }
    String httpCommand = "AT+HTTPPARA=\"URL\",\"https://tuz1jwn73m.execute-api.ap-southeast-1.amazonaws.com/showconfig\"";

    sendAT("AT+HTTPINIT", 3000, DEBUG);
    sendAT(httpCommand, 3000, DEBUG);
    sendAT("AT+HTTPACTION=0", 3000, DEBUG);
    jsonConfig = sendAT("AT+HTTPREAD=0,500", 3000, DEBUG);
    sendAT("AT+HTTPTERM", 2000, DEBUG);
  }

  String jsonPart;
  int startPos = jsonConfig.indexOf('{');
  int endPos = jsonConfig.lastIndexOf('}');

  if (startPos != -1 && endPos != -1)
  {
    jsonPart = jsonConfig.substring(startPos, endPos + 1);
    Serial.println(jsonPart);
  }
  else
  {
    Serial.println("JSON not found in the response.");
  }

  return jsonPart;
}

String createJsonString(int SyncWord, int TxPower, long freq, double interval)
{
  Serial.println("\n----------   Start of createJsonString()   ----------\n");
  StaticJsonDocument<512> doc;

  doc["SyncWord"] = SyncWord;
  doc["TxPower"] = TxPower;
  doc["freq"] = freq * 1000000;
  doc["interval"] = interval;

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.println("\n---------- End of createJsonString() ----------\n");
  return jsonString;
}

String processGPSResponse(String response)
{
  Serial.println("\n----------   Start of processGPSResponse()   ----------\n");
  Serial.println(response);
  String result = "";

  if (response.indexOf(",N") != -1)
  {
    int latStart = response.indexOf(":") + 2; // Start after ": "
    int latEnd = response.indexOf(",N");
    int longStart = response.indexOf(",N") + 3; // Start after ",N"
    int longEnd = response.indexOf(",E");

    String latitude = response.substring(latStart, latEnd);
    String longitude = response.substring(longStart, longEnd);

    float lat = convertCoordinate(latitude);
    latText = String(lat, 6);
    float lon = convertCoordinate(longitude);
    lonText = String(lon, 6);

    result = latText + "," + lonText;
  }
  else
  {
    int startIndex = response.indexOf(":") + 2; // Start after ": "
    int endIndex = response.indexOf(",");
    String relevantInfo = response.substring(startIndex, endIndex);
    relevantInfo.replace(",", "0");
    result = relevantInfo;
  }
  Serial.println("\n----------   End of processGPSResponse()   ----------\n");
  return result;
}

String processGNSSResponse(String response)
{
  SerialMon.println("\n----------   Start of processGPSResponse()   ----------\n");
  Serial.println(response);
  String result = "";

  if (response.indexOf(",N") != -1)
  {
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
    latText = String(lat, 6);
    float lon = convertCoordinate(longitude);
    lonText = String(lon, 6);

    result = latText + "," + lonText;
  }
  else
  {
    int startIndex = response.indexOf(",") + 1;
    int endIndex = response.lastIndexOf(",");
    String relevantInfo = response.substring(startIndex, endIndex);
    relevantInfo.replace(",", "0");
    result = relevantInfo;
  }
  SerialMon.println("\n----------   End of processGPSResponse()   ----------\n");
  return result;
}

void getconfigfromJson(String jsonInput)
{
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonInput);

  if (error)
  {
    Serial.print("Parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  SyncWord = doc["Syncword"];
  TxPower = doc["Tx_power"];
  freq = doc["Frequency"];
  interval = doc["Tx_Interval"];
}

bool connect2LTE()
{
  SerialMon.println("\n----------   Start of connect2LTE()   ----------\n");
  boolean DEBUG = 1;

  delay(1000);
  sendAT("AT+NETCLOSE", 1000, DEBUG);

  delay(1000);
  sendAT("AT+CPIN?", 2000, DEBUG);

  delay(1000);
  sendAT("AT+CSOCKSETPN=1", 5000, DEBUG);
  String res = sendAT("AT+NETOPEN", 5000, DEBUG);

  if (res.indexOf("OK") == -1)
  {
    esp_restart();
  }

  String response = sendAT("AT+IPADDR", 5000, DEBUG);

  // sendAT("AT+CPING=\"rung.ddns.net\",1,4", 10000, DEBUG);
  SerialMon.println("\n----------   End of connect2LTE()   ----------\n");
  return true;
}

void processJsonInput(const char *jsonInput)
{
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonInput);

  if (error)
  {
    Serial.print("Parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  NodeName = doc["NodeName"].as<String>();
  temp = doc["Temperature"];
  humi = doc["Humidity"];
}

void sleep(float sec)
{
  Serial.println("\n----------   Start of sleep()   ----------\n");
  double min_d = sec / 60;
  // Set wakeup time
  esp_sleep_enable_timer_wakeup((interval - min_d) * 60 * 0.8 * 1000000);

  // Print the duration in minutes to the serial monitor
  Serial.print("Duration: ");
  Serial.print(sec / 60);
  Serial.println(" minutes");

  // Go to sleep now
  Serial.print("Going to sleep for ");
  Serial.print((interval - min_d) * 0.8);
  Serial.println(" minutes");
  Serial.println("\n----------   End of sleep()   ----------\n");
  esp_deep_sleep_start();
}

void openGPS()
{
  for (int i = 0; i < 4; i++)
  {
    String response = sendAT("AT+CGNSSPWR=1", 1000, DEBUG);
    sendAT("AT+CGPSINFO?", 1000, DEBUG);
  }
}

void findGPS(float mintimeout)
{
  unsigned long findGPSstartTime = millis();
  while ((millis() - findGPSstartTime) < (mintimeout * 60000))
  {
    unsigned long usedtime = (millis() - findGPSstartTime) / 1000;
    Serial.println("\n Used : " + String(usedtime) + " seconds \n");
    gpsinfo = sendAT("AT+CGPSINFO", 1000, DEBUG);
    if (gpsinfo.indexOf(",,,,,,,,") == -1 && gpsinfo.indexOf("ERROR") == -1)
    {
      Serial.print(gpsinfo);
      processGPSResponse(gpsinfo);
      Serial.print("Used : " + String(usedtime) + " seconds to find GPS");
      break;
    }
  }
}

void setup()
{
  startTime = millis();
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
  modemPowerOn();
  delay(500);
  openGPS();
  findGPS(1);
  connect2LTE();
  getconfigfromJson(getJsonConfig());
}

void loop()
{
  LoRa.setSyncWord(0xF1);
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
      delay(5000);

      String jsonOutput = createJsonString(SyncWord, TxPower, freq, interval);

      Serial.println("Switching to sending state...");
      Serial.print("Packet send: ");
      Serial.println(jsonOutput);
      LoRa.setSyncWord(0xF2);
      LoRa.beginPacket();
      LoRa.print(jsonOutput);
      LoRa.endPacket();
      Serial.println("Packet sent.");

      processJsonInput(LoRaData);

      for (int i = 0; i < 5; i++)
      {
        sendAT("AT+CGNSSINFO?", 500, DEBUG);
        Serial.println("Requesting GPS info...");
        gpsinfo = sendAT("AT+CGPSINFO", 3000, DEBUG);
        Serial.print("Received GPS info: ");
        Serial.println(gpsinfo);

        if (gpsinfo.indexOf(",,,,,,,,") == -1)
        {
          break;
        }
      }

      if (gpsinfo.indexOf(",,,,,,,,") != -1)
      {
        Serial.println("No valid GPS info, performing other actions...");
        readcellinfo();
        sendrequest();
        request();
        esp_restart();
      }
      else
      {
        processGPSResponse(gpsinfo);
        request();
      }
      unsigned long endTime = millis();
      unsigned long duration = endTime - startTime;
      float durationSeconds = duration / 1000.0;

      sleep(durationSeconds);
    }
  }
}