#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGsmClientSIM7600.h>
#include <LoRa.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#define SerialAT Serial1
#define SerialMon Serial
TinyGsmSim7600 modem(SerialAT);
TinyGsmSim7600::GsmClientSim7600 client(modem);
TinyGPSPlus gps;
String URI = "http://xxxx.xxxx.xxxx.xxxx:8050/";
int PORT = 8050;
String enddeviceslist[] = {"Node1", "Node2", "Node3"};
String nodenametemp[100];
int nodenametemp_num = sizeof(enddeviceslist) / sizeof(enddeviceslist[0]);
int enddevices_num = sizeof(enddeviceslist) / sizeof(enddeviceslist[0]);
int node = 0;
bool recvall = false;
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
#define apiKey (String) "xxxxxxxxxxxxxxxxxxxxxxxxxxx"
bool DEBUG = true;
String response, LoRaData, latText, lonText, gpsinfo;
String serialres;
String latitude, longitude;
String extractedString = "";
String guser = "user2";
String NodeName[1000], user[300];
float degrees = 0.0;
float lat, lon;
float temp[200], humi[200], ebattlvl[200];
unsigned long setupstartTime, waitingtime, setupendtime, setuptime;
double speed = -1;
bool speedcheck;
float battPercentage;
float battVoltage;
RTC_DATA_ATTR int gSyncWord;
RTC_DATA_ATTR int gTxPower;
RTC_DATA_ATTR long gfreq;
RTC_DATA_ATTR double ginterval;
RTC_DATA_ATTR int gspreadingFactor;
RTC_DATA_ATTR long gsignalBandwidth;
int dgSyncWord = 0xF1;
int dgTxPower = 20;
int dgspreadingFactor = 9;
long dgfreq = 923E6;
long dgsignalBandwidth = 125E3;
double dginterval = 3;
int eSyncWord;
int eTxPower;
int espreadingFactor;
long efreq;
long esignalBandwidth;
double einterval;
double interval;
int sendSyncWord = 5;
struct NETWORK_INFO
{
  String date;
  String time;
  String lat;
  String lon;
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
bool wifiConnected = false;
const char *ssid = "o";
const char *password = "00000000";
const char *server = "xxxx.xxxx.xxxx.xxxx";
const int port = 8050;
String host = String(server);
WiFiClient wifiClient;
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
void connectToWiFi()
{
  SerialMon.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    SerialMon.print(".");
  }
  wifiConnected = true;
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
void changeGWconfig()
{
  if (eSyncWord != gSyncWord)
  {
    gSyncWord = eSyncWord;
  }
  if (eTxPower != gTxPower)
  {
    gTxPower = eTxPower;
  }
  if (efreq != gfreq)
  {
    gfreq = efreq * 1000000;
  }
  if (einterval != ginterval)
  {
    ginterval = einterval;
  }
}
void waitForGPSFix(float timeoutMinutes)
{
  unsigned long startTime = millis();
  unsigned long timeoutMillis = timeoutMinutes * 60000;
  while (millis() - startTime < timeoutMillis)
  {
    while (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
      {
        if (gps.location.isValid())
        {
          lat = gps.location.lat();
          lon = gps.location.lng();
          if ((lat >= -90 && lat <= 90) && (lon >= -180 && lon <= 180))
          {
            return;
          }
        }
      }
    }
  }
}
void GPSavg(int attemps)
{
  float latlist[attemps], lonlist[attemps];
  for (int i = 0; i < attemps; i++)
  {
    waitForGPSFix(1);
    latlist[i] = lat;
    lonlist[i] = lon;
  }
  latText = String(latlist[attemps / 2], 6);
  lonText = String(lonlist[attemps / 2], 6);
  for (int i = 0; i < attemps; i++)
  {
  }
}
void readcellinfo()
{
  String info = sendAT("AT+CPSI?", 10000, 1);
  int startIndex = info.indexOf("+CPSI: ");
  info = info.substring(startIndex + 7, startIndex + 80);
  info.replace("\r", "");
  info.replace("\n", "");
  info.replace("AT+CLBS=4ERROR", "");
  info.replace("AT+CPSI?+CPSI: NO SERVICE", "");
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
  int lacDec = (int)strtol(values[3].c_str(), NULL, 16);
  networkinfo.rssnr = values[13].toInt();
  networkinfo.mcc = values[2].substring(0, 3);
  networkinfo.mnc = values[2].substring(4, 6);
  networkinfo.lac = lacDec;
  networkinfo.cid = values[4];
  delay(1000);
}
void sendLocationRequestwifi()
{
  if (!WiFi.isConnected())
  {
    return;
  }
  String payload = "{\"token\":\"" + apiKey + "\",\"radio\":\"lte\",\"mcc\":" + networkinfo.mcc + ",\"mnc\":" + networkinfo.mnc + ",\"cells\":[{\"lac\":" + networkinfo.lac + ",\"cid\":" + networkinfo.cid + ",\"psc\":0}],\"address\":1}";
  String response;
  WiFiClient wifiClient;
  if (!wifiClient.connect("ap1.unwiredlabs.com", 80))
  {
    return;
  }
  String request = "POST /v2/process.php HTTP/1.1\r\n";
  request += "Host: ap1.unwiredlabs.com\r\n";
  request += "Content-Type: application/x-www-form-urlencoded\r\n";
  request += "Content-Length: ";
  request += String(payload.length());
  request += "\r\n\r\n";
  request += payload;
  wifiClient.print(request);
  const unsigned long TIMEOUT_DURATION = 10000;
  unsigned long startTime = millis();
  while (wifiClient.connected() && (millis() - startTime) < TIMEOUT_DURATION)
  {
    if (wifiClient.available())
    {
      String responseLine = wifiClient.readStringUntil('\n');
      response += responseLine;
    }
  }
  wifiClient.stop();
  delay(1500);
  int startIndex = response.indexOf("\"lat\":");
  int endIndex = response.indexOf(",\"lon\":");
  latText = response.substring(startIndex + 6, endIndex);
  startIndex = endIndex + 7;
  endIndex = response.indexOf(",\"accuracy\":");
  lonText = response.substring(startIndex, endIndex);
}
void sendLocationRequest()
{
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
  int startIndex = response.indexOf("\"lat\":");
  int endIndex = response.indexOf(",\"lon\":");
  latText = response.substring(startIndex + 6, endIndex);
  startIndex = endIndex + 7;
  endIndex = response.indexOf(",\"accuracy\":");
  lonText = response.substring(startIndex, endIndex);
}
void sendHttpRequest()
{
  bool DEBUG = true;
  for (int i = 0; i < nodenametemp_num; i++)
  {
    unsigned long startTime = millis();
    unsigned long timeout = 10000;
    String payload = "{";
    payload += "\"nodename\":\"" + nodenametemp[i] + "\",";
    payload += "\"user\":\"" + user[i] + "\",";
    payload += "\"temperature\":" + String(temp[i]) + ",";
    payload += "\"humidity\":" + String(humi[i]) + ",";
    payload += "\"latitude\":" + String(latText) + ",";
    payload += "\"longitude\":" + String(lonText) + ",";
    payload += "\"ebatlvl\":" + String(ebattlvl[i]) + ",";
    payload += "\"gbatlvl\":" + String(battPercentage, 2) + ",";
    if (speedcheck == true)
    {
      payload += "\"speed\":" + String(speed);
    }
    else
    {
      payload += "\"speed\":null";
    }
    payload += "}";
    String response;
    client.connect("xxxx.xxxx.xxxx.xxxx", PORT);
    String request = "POST /api/data HTTP/1.1\r\n";
    request += "Host: xxxx.xxxx.xxxx.xxxx\r\n";
    request += "Content-Type: application/json\r\n";
    request += "Content-Length: ";
    request += String(payload.length());
    request += "\r\n\r\n";
    request += payload;
    client.print(request);
    while (client.connected() && (millis() - startTime) < timeout)
    {
      while (client.available())
      {
        char c = client.read();
        response += c;
        client.write(c);
      }
    }
    String successMessage = "Data inserted successfully";
    if (response.indexOf(successMessage) != -1)
    {
    }
    else
    {
    }
    client.stop();
    delay(1500);
  }
}
void sendHttpRequestwifi()
{
  if (!wifiConnected)
  {
    connectToWiFi();
  }
  WiFiClient wifiClient;
  if (!wifiClient.connect(host.c_str(), 8050))
  {
    wifiClient.connect(host.c_str(), 8050);
    return;
  }
  for (int i = 0; i < nodenametemp_num; i++)
  {
    if (!wifiClient.connect(host.c_str(), 8050))
    {
      wifiClient.connect(host.c_str(), 8050);
      return;
    }
    String payload = "{";
    payload += "\"nodename\":\"" + nodenametemp[i] + "\",";
    payload += "\"user\":\"" + user[i] + "\",";
    payload += "\"temperature\":" + String(temp[i]) + ",";
    payload += "\"humidity\":" + String(humi[i]) + ",";
    payload += "\"latitude\":" + String(latText) + ",";
    payload += "\"longitude\":" + String(lonText) + ",";
    payload += "\"ebatlvl\":" + String(ebattlvl[i]) + ",";
    payload += "\"gbatlvl\":" + String(battPercentage, 2) + ",";
    if (speedcheck == true)
    {
      payload += "\"speed\":" + String(speed);
    }
    else
    {
      payload += "\"speed\":null";
    }
    payload += "}";
    String request = "POST /api/data HTTP/1.1\r\n";
    request += "Host: ";
    request += host;
    request += "\r\n";
    request += "Content-Type: application/json\r\n";
    request += "Content-Length: ";
    request += String(payload.length());
    request += "\r\n\r\n";
    request += payload;
    wifiClient.print(request);
    while (wifiClient.connected())
    {
      if (wifiClient.available())
      {
        String response = wifiClient.readStringUntil('\n');
      }
    }
    wifiClient.stop();
    delay(5000);
  }
}
String fetchJsonConfig()
{
  String response;
  int maxRetries = 10;
  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (client.connect("xxxx.xxxx.xxxx.xxxx", PORT))
    {
      String request = "GET /api/showconfig?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: xxxx.xxxx.xxxx.xxxx\r\n";
      request += "Connection: close\r\n";
      request += "\r\n";
      client.print(request);
      unsigned long startTime = millis();
      while (client.connected() && (millis() - startTime) < 10000)
      {
        while (client.available())
        {
          char c = client.read();
          response += c;
          Serial.write(c);
        }
      }
      client.stop();
      delay(1000);
      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');
      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        return jsonPart;
      }
      else
      {
        response = "";
      }
    }
    else
    {
    }
  }
  return "";
}
String fetchJsonConfigWIFI()
{
  String response;
  int maxRetries = 10;
  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (wifiClient.connect(server, port))
    {
      String request = "GET /api/showconfig?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: " + String(server) + "\r\n";
      request += "Connection: close\r\n";
      request += "\r\n";
      wifiClient.print(request);
      unsigned long startTime = millis();
      while (wifiClient.connected() && (millis() - startTime) < 10000)
      {
        while (wifiClient.available())
        {
          char c = wifiClient.read();
          response += c;
          Serial.write(c);
        }
      }
      wifiClient.stop();
      delay(1000);
      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');
      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        return jsonPart;
      }
      else
      {
        response = "";
      }
    }
    else
    {
    }
  }
  return "";
}
String fetchActiveNode()
{
  String response;
  int maxRetries = 10;
  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (client.connect("xxxx.xxxx.xxxx.xxxx", PORT))
    {
      String request = "GET /api/node?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: xxxx.xxxx.xxxx.xxxx\r\n";
      request += "\r\n";
      client.print(request);
      unsigned long startTime = millis();
      while (client.connected() && (millis() - startTime) < 10000)
      {
        while (client.available())
        {
          char c = client.read();
          response += c;
          Serial.write(c);
        }
      }
      client.stop();
      delay(2000);
      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');
      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        return jsonPart;
      }
      else
      {
        response = "";
      }
    }
    else
    {
    }
  }
  return "";
}
String fetchActiveNodeWIFI()
{
  String response;
  int maxRetries = 10;
  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (wifiClient.connect(server, port))
    {
      String request = "GET /api/node?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: " + String(server) + "\r\n";
      request += "\r\n";
      wifiClient.print(request);
      unsigned long startTime = millis();
      while (wifiClient.connected() && (millis() - startTime) < 10000)
      {
        while (wifiClient.available())
        {
          char c = wifiClient.read();
          response += c;
          Serial.write(c);
        }
      }
      wifiClient.stop();
      delay(2000);
      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');
      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        return jsonPart;
      }
      else
      {
        response = "";
      }
    }
    else
    {
    }
  }
  return "";
}
void parseActiveNode(String jsonInput)
{
  const char *jsonString = jsonInput.c_str();
  StaticJsonDocument<500> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error)
  {
    return;
  }
  JsonArray nodenames = doc["nodenames"];
  const size_t arraySize = nodenames.size();
  if (arraySize > 1)
  {
    String nodenamesArray[arraySize];
    for (size_t i = 0; i < arraySize; i++)
    {
      nodenamesArray[i] = nodenames[i].as<String>();
    }
    for (size_t i = 0; i < arraySize; i++)
    {
    }
    if (enddevices_num == arraySize)
    {
      for (size_t i = 0; i < arraySize; i++)
      {
        for (size_t j = 0; j < enddevices_num; j++)
        {
          if (enddeviceslist[i] == nodenamesArray[j])
          {
            enddeviceslist[i] = nodenamesArray[j];
          }
        }
      }
    }
    else
    {
      enddevices_num = arraySize;
      for (size_t j = 0; j < enddevices_num; j++)
      {
        enddeviceslist[j] = nodenamesArray[j];
      }
    }
    for (size_t i = 0; i < arraySize; i++)
    {
    }
    for (size_t i = 0; i < enddevices_num; i++)
    {
    }
  }
  else
  {
  }
}
void parseJsonConfig(String jsonInput)
{
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonInput);
  if (error)
  {
    eSyncWord = dgSyncWord;
    eTxPower = dgTxPower;
    efreq = dgfreq;
    interval = dginterval;
    return;
  }
  eSyncWord = doc["Syncword"];
  eTxPower = doc["Tx_power"];
  efreq = doc["Frequency"];
  if (efreq < 1000000)
  {
    efreq = efreq * 1000000;
  }
  einterval = doc["Tx_interval"];
  changeGWconfig();
}
String createJsonString(int SyncWord, int TxPower, long freq, double interval)
{
  StaticJsonDocument<512> doc;
  if (gSyncWord == 0 || gTxPower == 0 || gfreq == 0 || ginterval == 0)
  {
    gSyncWord = dgSyncWord;
    gTxPower = dgTxPower;
    gfreq = dgfreq;
    gspreadingFactor = dgspreadingFactor;
    gsignalBandwidth = dgsignalBandwidth;
    ginterval = dginterval;
  }
  if (SyncWord == 0)
  {
    SyncWord = dgSyncWord;
  }
  if (TxPower == 0)
  {
    TxPower = dgTxPower;
  }
  if (freq == 0)
  {
    freq = dgfreq;
  }
  if (interval == 0)
  {
    interval = dginterval;
  }
  doc["SyncWord"] = SyncWord;
  doc["TxPower"] = TxPower;
  if (freq < 1000)
  {
    doc["freq"] = freq * 1000000;
  }
  else
  {
    doc["freq"] = freq;
  }
  doc["interval"] = interval;
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}
bool connect2LTE()
{
  boolean DEBUG = 1;
  delay(1000);
  sendAT("AT+NETCLOSE", 1000, DEBUG);
  delay(1000);
  sendAT("AT+CPIN?", 2000, DEBUG);
  delay(1000);
  sendAT("AT+CSOCKSETPN=1", 3000, DEBUG);
  String res = sendAT("AT+NETOPEN", 3000, DEBUG);
  if (res.indexOf("OK") == -1 || res.indexOf("not") != -1)
  {
    connectToWiFi();
  }
  else
  {
    String response = sendAT("AT+IPADDR", 5000, DEBUG);
    if (response.indexOf("ERROR") != -1)
    {
      if (!wifiConnected)
      {
        connectToWiFi();
      }
    }
  }
  return true;
}
void sleep(float sec)
{
  double min_d = sec / 60;
  esp_sleep_enable_timer_wakeup((ginterval - min_d) * 60 * 0.5 * 1000000);
  esp_deep_sleep_start();
}
void readBattLevel()
{
  const int numOfReadings = 10000;
  const float batteryFullVoltage = 4.2;
  const float batteryOffVoltage = 2.5;
  int battReadingsSum = 0;
  for (int i = 0; i < numOfReadings; i++)
  {
    battReadingsSum += analogRead(BAT_ADC);
  }
  float avgReading = static_cast<float>(battReadingsSum) / static_cast<float>(numOfReadings);
  battVoltage = avgReading * (3.3 / 4096.0) * 2.0 * 1.079691;
  battPercentage = 100.0 * (1.0 - ((batteryFullVoltage - battVoltage) / (batteryFullVoltage - batteryOffVoltage)));
  if (battPercentage > 100.00)
  {
    battPercentage = 100.00;
  }
}
void setup()
{
  setupstartTime = millis();
  Serial.begin(UART_BAUD);
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  Serial2.begin(9600, SERIAL_8N1, 33, 32);
  LoRa.setPins(ss, rst, dio0);
  if (gSyncWord == 0 || gTxPower == 0 || gfreq == 0 || ginterval == 0)
  {
    gSyncWord = dgSyncWord;
    gTxPower = dgTxPower;
    gfreq = dgfreq;
    gspreadingFactor = dgspreadingFactor;
    gsignalBandwidth = dgsignalBandwidth;
    ginterval = dginterval;
  }
  while (!LoRa.begin(gfreq))
  {
    delay(500);
  }
  LoRa.setTxPower(gTxPower);
  LoRa.setSyncWord(gSyncWord);
  LoRa.setSpreadingFactor(gspreadingFactor);
  LoRa.setSignalBandwidth(gsignalBandwidth);
  LoRa.enableCrc();
  modemPowerOn();
  delay(500);
  connect2LTE();
  if (wifiConnected)
  {
    parseJsonConfig(fetchJsonConfigWIFI());
    parseActiveNode(fetchActiveNodeWIFI());
  }
  else
  {
    parseJsonConfig(fetchJsonConfig());
    parseActiveNode(fetchActiveNode());
  }
  setupendtime = millis();
  setuptime = setupendtime - setupstartTime;
  for (int i = 0; i < enddevices_num; i++)
  {
    nodenametemp[i] = enddeviceslist[i];
  }
  nodenametemp_num = enddevices_num;
  recvall = false;
}
void loop()
{
  waitingtime = millis();
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    char LoRaData[255];
    int dataIndex = 0;
    while (LoRa.available())
    {
      char receivedChar = LoRa.read();
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
    if (dataIndex > 0)
    {
      LoRaData[dataIndex] = '\0';
      DynamicJsonDocument doc(256);
      deserializeJson(doc, LoRaData);
      String input_nodename = doc["NodeName"].as<String>();
      String input_user = doc["User"].as<String>();
      float input_temp = doc["Temperature"];
      float input_humi = doc["Humidity"];
      float input_batt = doc["BatLvl"];
      for (int i = 0; i < enddevices_num; i++)
      {
        if (enddeviceslist[i] == input_nodename)
        {
          user[node] = input_user;
          temp[node] = input_temp;
          humi[node] = input_humi;
          ebattlvl[node] = input_batt;
          nodenametemp[node] = input_nodename;
          for (int j = i; j < enddevices_num - 1; j++)
          {
            enddeviceslist[j] = enddeviceslist[j + 1];
          }
          enddevices_num--;
          node = node + 1;
          break;
        }
      }
    }
    if (enddevices_num == 0)
    {
      recvall = true;
      delay(5000);
      String jsonOutput = createJsonString(eSyncWord, eTxPower, efreq, einterval);
      for (int i = 0; i < 4; i++)
      {
        LoRa.setTxPower(20);
        LoRa.setSyncWord(242);
        LoRa.beginPacket();
        LoRa.print(jsonOutput);
        LoRa.endPacket();
        delay(1000);
      }
    }
  }
  if (waitingtime > (ginterval * 60 * 1000) + setuptime)
  {
    recvall = true;
  }
  if (recvall)
  {
    waitForGPSFix(0.017);
    float latValue = lat;
    float lonValue = lon;
    if ((latValue > -90 && latValue < 0) || (latValue > 0 && latValue <= 90) && (lonValue > -180 && lonValue < 0) || (lonValue > 0 && lonValue <= 180))
    {
      GPSavg(5);
      speed = gps.speed.kmph();
      speedcheck = true;
      readBattLevel();
      if (wifiConnected)
      {
        sendHttpRequestwifi();
      }
      else
      {
        sendHttpRequest();
      }
    }
    else
    {
      speedcheck = false;
      readcellinfo();
      readBattLevel();
      if (wifiConnected)
      {
        sendLocationRequestwifi();
        sendHttpRequestwifi();
      }
      else
      {
        sendLocationRequest();
        sendHttpRequest();
      }
    }
    unsigned long endTime = millis();
    unsigned long duration = endTime - waitingtime + setuptime;
    float durationSeconds = duration / 1000.0;
    changeGWconfig();
    sleep(durationSeconds);
  }
}
