#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "TinyGPS++.h"
#include <HardwareSerial.h>

// The TinyGPSPlus object
String serialres, latText, lonText;
String latitude, longitude;
String extractedString = "";
float degrees = 0.0;
float lat, lon;

struct GPS_info
{
  float lat;
  float lon;
};
GPS_info gps_info;

TinyGPSPlus gps;

float convertlatCoordinate(String coordString)
{
  degrees = 0.0;
  String degreesString = coordString.substring(0, 2);
  int degreesValue = degreesString.toInt();

  String minutesString = coordString.substring(2);
  float minutesValue = minutesString.toFloat();

  degrees = degreesValue + (minutesValue / 60.0);

  Serial.print("Latitude : ");
  Serial.println(degrees, 6);
  return degrees;
}

float convertlonCoordinate(String coordString)
{
  degrees = 0.0;
  String degreesString = coordString.substring(0, 3);
  int degreesValue = degreesString.toInt();

  String minutesString = coordString.substring(3);
  float minutesValue = minutesString.toFloat();

  degrees = degreesValue + (minutesValue / 60.0);

  Serial.print("Longtitude : ");
  Serial.println(degrees, 6);
  return degrees;
}

bool readgps(int interval)
{
  serialres = "";
  lat = 0;
  lon = 0;
  long int startTime = millis();
  while (((millis() - startTime)) < interval)
  {
    while (Serial2.available() > 0)
    {
      int readData = Serial2.read();
      serialres += char(readData);
    }
  }
  Serial2.flush();
  if (serialres.indexOf(",A") != -1 && serialres.indexOf("$GPRMC") != -1)
  {
    int startPos = serialres.indexOf("$GPRMC");
    int endPos = serialres.indexOf("E", startPos);
    extractedString = serialres.substring(startPos, endPos + 2);
    Serial.println(extractedString);
    // Find the positions of 'N,' and 'E,' in the input string
    int nPos = serialres.indexOf("N,");
    int ePos = serialres.indexOf("E,");

    // Check if 'N,' and 'E,' were found in the input string
    if (nPos != -1 && ePos != -1)
    {
      // Extract the latitude and longitude substrings
      latitude = extractedString.substring(nPos - 11, nPos);
      lat = convertlatCoordinate(latitude);
      latText = String(lat, 6);

      longitude = extractedString.substring(ePos - 12, ePos);
      lon = convertlonCoordinate(longitude);
      lonText = String(lon, 6);
      return true;
    }
  }
  else
  {
    return false;
  }
  return false;
}

void findGPS(float mintimeout)
{
  unsigned long findGPSstartTime = millis();
  unsigned long timeoutMillis = mintimeout * 60000;

  while (millis() - findGPSstartTime < timeoutMillis)
  {
    lat = 0;
    lon = 0;
    unsigned long usedtime = (millis() - findGPSstartTime) / 1000;
    // Serial.println("\nUsed : " + String(usedtime) + " seconds");

    if (readgps(300) && lat > 13 && lon > 100)
    {
      // Serial.println("GPS found!");
      // Serial.println("Used : " + String(usedtime) + " seconds to find GPS");
      break;
    }
    else
    {
      // Serial.println("Finding GPS...");
    }
  }
}

float calculateAverage(float arrayofval[], int sizes)
{
  float sum = 0;

  for (int i = 0; i < sizes; i++)
  {
    sum += arrayofval[i];
  }

  return sum / sizes;
}

void GPSavg(int times)
{
  float latlist[times], lonlist[times];

  for (int i = 0; i < times; i++)
  {
    Serial.print("\n------------------ ");
    Serial.print(i);
    Serial.println(" ------------------");
    findGPS(1);
    latlist[i] = lat;
    lonlist[i] = lon;
  }
  latText = String(calculateAverage(latlist, times), 6);
  lonText = String(calculateAverage(lonlist, times), 6);

  Serial.print("\n------------------ ");
  Serial.print("All values");
  Serial.println(" ------------------");
  for (int i = 0; i < 10; i++)
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

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 14, 15);
  delay(1000);
}
void loop()
{
  GPSavg(10);
  Serial.print("Average of Lat and lon : ");
  Serial.println(latText + "," + lonText);
  Serial.println("END\n\n\n");
  delay(2000);
  esp_restart;
}
