// Gateway

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGsmClientSIM7600.h>
#include <LoRa.h>
#include <TinyGPS++.h>

#define SerialAT Serial1
#define SerialMon Serial

TinyGsmSim7600 modem(SerialAT);
TinyGsmSim7600::GsmClientSim7600 client(modem);
TinyGPSPlus gps;

// Define API URI
String URI = "http://rung.ddns.net:8050/";
int PORT = 8050;

// Define End devices list
String enddeviceslist[] = {"Node1", "Node2", "Node3"};
String nodenametemp[100];
int nodenametemp_num = sizeof(enddeviceslist) / sizeof(enddeviceslist[0]);
int enddevices_num = sizeof(enddeviceslist) / sizeof(enddeviceslist[0]);
int node = 0;
bool recvall = false;

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

// LoRa configuration
RTC_DATA_ATTR int gSyncWord;
RTC_DATA_ATTR int gTxPower;
RTC_DATA_ATTR long gfreq;
RTC_DATA_ATTR double ginterval;
RTC_DATA_ATTR int gspreadingFactor;
RTC_DATA_ATTR long gsignalBandwidth;

// Default LoRa Gateway configuration
int dgSyncWord = 0xF1;
int dgTxPower = 20;
int dgspreadingFactor = 9;
long dgfreq = 923E6;
long dgsignalBandwidth = 125E3;
double dginterval = 3;

// End devices LoRa configuration
int eSyncWord;
int eTxPower;
int espreadingFactor;
long efreq;
long esignalBandwidth;
double einterval;

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
    Serial.print("\n------------------ ");
    Serial.print(i + 1);
    Serial.println(" ------------------");
    waitForGPSFix(1);
    latlist[i] = lat;
    lonlist[i] = lon;
  }
  latText = String(latlist[attemps / 2], 6);
  lonText = String(lonlist[attemps / 2], 6);

  Serial.print("\n------------------ ");
  Serial.print("All values");
  Serial.println(" ------------------");
  for (int i = 0; i < attemps; i++)
  {
    Serial.print("Latitude ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(String(latlist[i], 6));
    Serial.print("  Longitude ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(String(lonlist[i], 6));
  }
  Serial.println("------------------------------------------------");
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

void sendLocationRequest()
{
  SerialMon.println("\n----------   Start of sendLocationRequest()   ----------\n");
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
  SerialMon.println("\n----------   End of sendLocationRequest()   ----------\n");
}

void sendHttpRequest()
{
  bool DEBUG = true;
  SerialMon.println("\n----------   Start of sendHttpRequest()   ----------\n");

  for (int i = 0; i < nodenametemp_num; i++)
  {
    unsigned long startTime = millis();
    unsigned long timeout = 10000;

    // Create a JSON object with key-value pairs
    String payload = "{";
    payload += "\"nodename\":\"" + NodeName[i] + "\",";
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
    client.connect("rung.ddns.net", PORT);
    String request = "POST /api/data HTTP/1.1\r\n";
    request += "Host: rung.ddns.net\r\n";
    request += "Content-Type: application/json\r\n";
    request += "Content-Length: ";
    request += String(payload.length());
    request += "\r\n\r\n";
    request += payload;
    Serial.println(request);
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
      Serial.println("\nData was inserted successfully");
    }
    else
    {
      Serial.println("\nData was not inserted");
    }
    client.stop();
    delay(1500);
  }

  SerialMon.println("\n----------   End of sendHttpRequest()   ----------\n");
}

String fetchActiveNode()
{
  Serial.println("\n----------   Start of fetchJsonConfig()   ----------\n");
  String response;
  int maxRetries = 10; // Maximum number of retries

  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (client.connect("rung.ddns.net", PORT))
    {
      String request = "GET /api/node?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: rung.ddns.net\r\n";
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
      Serial.println("Response =" + response);

      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');

      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        Serial.println("\nExtracted JSON: " + jsonPart);
        return jsonPart;
      }
      else
      {
        Serial.println("JSON not found in the response (Retry " + String(retry) + " of " + String(maxRetries) + ").");
        response = ""; // Clear the response for the next attempt
      }
    }
    else
    {
      Serial.println("Failed to connect to the server (Retry " + String(retry) + " of " + String(maxRetries) + ").");
    }
  }

  Serial.println("\nReached maximum number of retries. Exiting fetchJsonConfig()\n");
  return "";
}

void parseActiveNode(String jsonInput)
{
  // GPSavg(1);
  const char *jsonString = jsonInput.c_str();

  StaticJsonDocument<500> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error)
  {
    Serial.print(F("Failed to parse JSON: "));
    Serial.println(error.c_str());
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

    Serial.println(F("Extracted nodenames:"));
    for (size_t i = 0; i < arraySize; i++)
    {
      Serial.println(nodenamesArray[i]);
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

    Serial.println(F("nodenames(nodenamesArray):"));
    for (size_t i = 0; i < arraySize; i++)
    {
      Serial.println(nodenamesArray[i]);
    }

    Serial.println(F("nodenames(enddeviceslist):"));
    for (size_t i = 0; i < enddevices_num; i++)
    {
      Serial.println(enddeviceslist[i]);
    }
  }
  else
  {
    Serial.print("Failed to get Nodenames");
  }
}

String fetchJsonConfig()
{
  Serial.println("\n----------   Start of fetchJsonConfig()   ----------\n");
  String response;
  int maxRetries = 10; // Maximum number of retries

  for (int retry = 1; retry <= maxRetries; retry++)
  {
    if (client.connect("rung.ddns.net", PORT))
    {
      String request = "GET /api/showconfig?user=" + guser + " HTTP/1.1\r\n";
      request += "Host: rung.ddns.net\r\n";
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
      Serial.println("\nResponse = " + response);

      int start = response.indexOf('{');
      int end = response.lastIndexOf('}');

      if (start != -1 && end != -1 && end > start)
      {
        String jsonPart = response.substring(start, end + 1);
        Serial.println("\nExtracted JSON: " + jsonPart);
        return jsonPart;
      }
      else
      {
        Serial.println("JSON not found in the response (Retry " + String(retry) + " of " + String(maxRetries) + ").");
        response = "";
      }
    }
    else
    {
      Serial.println("Failed to connect to the server (Retry " + String(retry) + " of " + String(maxRetries) + ").");
    }
  }

  Serial.println("\nReached maximum number of retries. Exiting fetchJsonConfig()\n");
  return "";
}

void changeGWconfig()
{
  Serial.println("\n----------   Start of changeGWconfig()   ----------\n");

  if (eSyncWord != gSyncWord)
  {
    gSyncWord = eSyncWord;
    Serial.println(gSyncWord);
    Serial.println("SyncWord changed");
  }
  if (eTxPower != gTxPower)
  {
    gTxPower = eTxPower;
    Serial.println("TxPower changed");
  }
  if (efreq != gfreq)
  {
    gfreq = efreq * 1000000;
    Serial.println("Freq changed");
  }
  if (einterval != ginterval)
  {
    ginterval = einterval;
    Serial.println("interval changed");
  }
  Serial.println("\n\nLoRa Gateway Configuration ( changeGWconfig() )\n");
  Serial.print("SyncWord: ");
  Serial.println(gSyncWord, HEX);
  Serial.print("TxPower: ");
  Serial.println(gTxPower);
  Serial.print("Frequency: ");
  Serial.println(gfreq);
  Serial.print("Interval: ");
  Serial.println(ginterval);
  Serial.print("SpreadingFactor: ");
  Serial.println(gspreadingFactor);
  Serial.print("SignalBandwidth: ");
  Serial.println(gsignalBandwidth);
  Serial.println("\n----------   End of changeGWconfig()   ----------\n");
}

String createJsonString(int SyncWord, int TxPower, long freq, double interval)
{
  Serial.println("\n----------   Start of createJsonString()   ----------\n");
  StaticJsonDocument<512> doc;
  if (gSyncWord == 0 || gTxPower == 0 || gfreq == 0 || ginterval == 0)
  {
    gSyncWord = dgSyncWord;
    gTxPower = dgTxPower;
    gfreq = dgfreq;
    gspreadingFactor = dgspreadingFactor;
    gsignalBandwidth = dgsignalBandwidth;
    ginterval = dginterval;
    Serial.println("\n\nNo value use default");
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

  Serial.println("\n---------- End of createJsonString() ----------\n");

  return jsonString;
}

void parseJsonConfig(String jsonInput)
{
  SerialMon.println("\n----------   Start of parseJsonConfig()   ----------\n");
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonInput);

  if (error)
  {
    Serial.print("Parsing failed: ");
    Serial.println(error.c_str());
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

  Serial.print(efreq);

  changeGWconfig();
  SerialMon.println("\n----------   End of parseJsonConfig()   ----------\n");
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
  sendAT("AT+CSOCKSETPN=1", 3000, DEBUG);
  String res = sendAT("AT+NETOPEN", 3000, DEBUG);
  Serial.println("--------------");
  Serial.println("Response: " + res); // Print the response for debugging
  Serial.println("--------------");
  if (res.indexOf("OK") == -1 || res.indexOf("not") != -1)
  {
    Serial.println("Restarting...");
    esp_restart();
  }

  String response = sendAT("AT+IPADDR", 5000, DEBUG);
  Serial.println("--------------");
  Serial.println("Response: " + response);
  Serial.println("--------------");
  if (response.indexOf("ERROR") != -1)
  {
    Serial.println("Restarting...");
    esp_restart();
  }

  // sendAT("AT+CPING=\"rung.ddns.net\",1,4", 10000, DEBUG);
  SerialMon.println("\n----------   End of connect2LTE()   ----------\n");
  return true;
}

void sleep(float sec)
{
  Serial.println("\n----------   Start of sleep()   ----------\n");
  double min_d = sec / 60;

  esp_sleep_enable_timer_wakeup((ginterval - min_d) * 60 * 0.5 * 1000000);

  Serial.print("Duration: ");
  Serial.print(sec / 60);
  Serial.println(" minutes");

  Serial.print("Going to sleep for ");
  Serial.print((ginterval - min_d) * 0.7);
  Serial.println(" minutes");
  Serial.println("\n----------   End of sleep()   ----------\n");
  esp_deep_sleep_start();
}

void readBattLevel()
{
  SerialMon.println("\n----------   Start of readBattLevel()   ----------\n");
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

  Serial.println("batt_level : " + String(battPercentage, 2));
  if (battPercentage > 100.00)
  {
    battPercentage = 100.00;
  }
  Serial.println("batt_volt : " + String(battVoltage, 2));
  SerialMon.println("\n----------   End of readBattLevel()   ----------\n");
}

void setup()
{
  SerialMon.println("\n----------   Start of Setup   ----------\n");
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
    Serial.println("\n\nNo value use default");
  }
  while (!LoRa.begin(gfreq))
  {
    Serial.println(".");
    delay(500);
  }

  LoRa.setTxPower(gTxPower);
  LoRa.setSyncWord(gSyncWord);
  LoRa.setSpreadingFactor(gspreadingFactor);
  LoRa.setSignalBandwidth(gsignalBandwidth);
  LoRa.enableCrc();

  // Show LoRa module configuraion
  Serial.println("\nLoRa Gateway Configuration\n");
  Serial.print("SyncWord: ");
  Serial.println(gSyncWord, HEX);
  Serial.print("TxPower: ");
  Serial.println(gTxPower);
  Serial.print("Frequency: ");
  Serial.println(gfreq);
  Serial.print("Interval: ");
  Serial.println(ginterval);
  Serial.print("SpreadingFactor: ");
  Serial.println(gspreadingFactor);
  Serial.print("SignalBandwidth: ");
  Serial.println(gsignalBandwidth);

  Serial.println("\n\nLoRa Initialized!\n");

  modemPowerOn();
  delay(500);
  connect2LTE();
  parseJsonConfig(fetchJsonConfig());
  parseActiveNode(fetchActiveNode());
  SerialMon.println("\n----------   End of Setup   ----------\n");
  SerialMon.println("\nWaiting for Data\n");
  setupendtime = millis();
  setuptime = setupendtime - setupstartTime;
  for (int i = 0; i < enddevices_num; i++)
  {
    nodenametemp[i] = enddeviceslist[i];
  }
  nodenametemp_num = enddevices_num;
}

void loop()
{
  waitingtime = millis();
  // recvall = true;

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

      DynamicJsonDocument doc(256);
      deserializeJson(doc, LoRaData);

      String input_nodename = doc["NodeName"].as<String>();
      String input_user = doc["User"].as<String>();
      float input_temp = doc["Temperature"];
      float input_humi = doc["Humidity"];
      float input_batt = doc["BatLvl"];

      for (int i = 0; i < enddevices_num; i++)
      {
        Serial.println(enddeviceslist[i]);

        if (enddeviceslist[i] == input_nodename)
        {
          Serial.println("Successfully receieved data from " + enddeviceslist[i]);
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

          Serial.print("Remaining nodename: ");
          for (int k = 0; k < enddevices_num; k++)
          {
            Serial.print(enddeviceslist[k]);
            Serial.print(" ");
          }
          Serial.println();
          node = node + 1;
          break;
        }
      }
    }
    if (enddevices_num == 0)
    {
      Serial.print("Temp: ");
      for (int i = 0; i < nodenametemp_num; i++)
      {
        if (i == nodenametemp_num - 1)
        {
          Serial.print(temp[i]);
        }
        else
        {
          Serial.print(temp[i]);
          Serial.print(", ");
        }
      }
      Serial.println();

      Serial.print("Humi: ");
      for (int i = 0; i < nodenametemp_num; i++)
      {
        if (i == nodenametemp_num - 1)
        {
          Serial.print(humi[i]);
        }
        else
        {
          Serial.print(humi[i]);
          Serial.print(", ");
        }
      }
      Serial.println();

      Serial.print("BattLvl: ");
      for (int i = 0; i < nodenametemp_num; i++)
      {
        if (i == nodenametemp_num - 1)
        {
          Serial.print(ebattlvl[i]);
        }
        else
        {
          Serial.print(ebattlvl[i]);
          Serial.print(", ");
        }
      }
      Serial.println();

      recvall = true;
      Serial.println("Successfully received data from all nodes.");

      delay(5000);

      String jsonOutput = createJsonString(eSyncWord, eTxPower, efreq, einterval);

      Serial.println("Switching to sending state...");
      Serial.print("Packet send: ");
      Serial.println(jsonOutput);
      LoRa.setTxPower(20);
      LoRa.setSyncWord(242);
      LoRa.beginPacket();
      LoRa.print(jsonOutput);
      LoRa.endPacket();
      Serial.println("Packet sent.");
    }
  }
  if (waitingtime > (ginterval * 60 * 1000) + setuptime)
  {
    SerialMon.println(waitingtime);
    recvall = true;
  }

  if (recvall)
  {
    waitForGPSFix(0.017);
    float latValue = lat;
    float lonValue = lon;
    Serial.print("Latitude: ");
    Serial.println(latValue, 6);
    Serial.print("Longitude: ");
    Serial.println(lonValue, 6);

    if ((latValue > -90 && latValue < 0) || (latValue > 0 && latValue <= 90) && (lonValue > -180 && lonValue < 0) || (lonValue > 0 && lonValue <= 180))
    {
      GPSavg(5);
      Serial.print("Speed in km/h = ");
      speed = gps.speed.kmph();
      Serial.println(speed);
      speedcheck = true;
      readBattLevel();
      sendHttpRequest();
    }
    else
    {
      Serial.println("No valid GPS info, performing other actions...");
      speedcheck = false;
      readcellinfo();
      sendLocationRequest();
      readBattLevel();
      sendHttpRequest();
    }
    unsigned long endTime = millis();
    unsigned long duration = endTime - waitingtime + setuptime;
    float durationSeconds = duration / 1000.0;
    changeGWconfig();
    // sleep(durationSeconds);
  }
}
