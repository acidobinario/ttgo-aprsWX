#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <config.h>

#define READ_DELAY_MS 60000 // 1 mn between reads (in milliseconds)



// MQTT topics: humidity, temperature, pressure and battery.
String humidity_topic;
String temperature_topic;
String pressure_topic;
String battery_topic;

#define BME_PWR 17
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

WiFiClient espClient;

PubSubClient client(espClient);
int status = WL_IDLE_STATUS;


long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float press = 0.0;
float diff = 1.0;

// Screen pin assignments
#define LCD_MOSI 19
#define LCD_CLK 18
#define LCD_CS 5
#define LCD_DC 16
#define LCD_RST 23
#define LCD_BL 4

// Battery reading
#define battery 34

void draw()
{
}

void setupWiFi()
{
 unsigned long start = millis(); 
  // We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  Serial.println("Attempting to connect to WiFi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    if ((millis() - start) > 30000){
      ESP.restart();
    }
  }
  Serial.println("connected to WiFi ");
}

void reconnect()
{
  int mqttFailuresNumber = 0;

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(DEVICE_ID, mqtt_user, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      if (mqttFailuresNumber == 3)
      {
        // reset wifi if too many mqtt failures
        mqttFailuresNumber = 0;

        // re-initialize the Wifi connection, whatever happened
        WiFi.reconnect();
      }
      else
      {
        mqttFailuresNumber++;
        Serial.println(mqttFailuresNumber);
      }
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

String getTemp(float pTemp)
{
  String strTemp;
  int intTemp;
  Serial.println("Run getTemp(float pTemp)");
  intTemp = (int)pTemp;
  strTemp = String(intTemp);
  // strTemp.replace(".", "");
  Serial.println("intTemp: " + strTemp + " length:" + String(strTemp.length()));

  switch (strTemp.length())
  {
  case 1:
    return "00" + strTemp;
    break;
  case 2:
    return "0" + strTemp;
    break;
  case 3:
    return strTemp;
    break;
  default:
    return "-999";
  }
}

String getHum(float pHum)
{
  String strHum;
  int intHum;
  Serial.println("Run getHum(float pHum)");
  intHum = (int)pHum;
  strHum = String(intHum);
  Serial.println("intHum: " + strHum + " length:" + String(strHum.length()));
  switch (strHum.length())
  {
  case 1:
    return "0" + strHum;
    break;
  case 2:
    return strHum;
    break;
  case 3:
    if (intHum == 100)
    {
      return "00";
    }
    else
    {
      return "-99";
    }
    break;
  default:
    return "-99";
  }
}

String getPres(float pPress)
{
  String strPress;
  int intPress = 0;
  intPress = (int)(pPress);
  strPress = String(intPress);
  Serial.println(strPress + " strPress " + String(strPress.length()) + " length");
  Serial.println("Run getPres(float pPress)");

  // strPress.replace(".", "");
  /// strPress = strPress.substring(0,5);
  // return strPress;

  switch (strPress.length())
  {
  case 1:
    return "000" + strPress + "0";
    break;
  case 2:
    return "00" + strPress + "0";
    break;
  case 3:
    return "0" + strPress + "0";
    break;
  case 4:
    return strPress;
    break;
  // case 5:
  //   return strPress;
  //   break;
  default:
    return "-99999";
  }
}

void bmeStart()
{
  digitalWrite(BME_PWR, 1);

  unsigned bme_status = bme.begin(0x76, &Wire);
  if (!bme_status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
  }
}

void setup()
{
  pinMode(BME_PWR, OUTPUT);
  Serial.begin(9600);

  humidity_topic = String(DEVICE_ID) + "/humidity";
  temperature_topic = String(DEVICE_ID) + "/temperature";
  pressure_topic = String(DEVICE_ID) + "/pressure";
  battery_topic = String(DEVICE_ID) + "/battery";

  setupWiFi();
  client.setServer(mqtt_server, 1883);

  bmeStart();
  delay(200);

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  float newTemp = bme.readTemperature();
  float newHum = bme.readHumidity();
  float newPress = (bme.readPressure() / 100.0F);
  float bat = analogRead(battery);

  if (!isnan(newTemp))
  {
    // if (abs(newTemp - temp) < 10)
    // {
    temp = newTemp;
    Serial.print("Temperature:");
    Serial.println(String(temp).c_str());
    client.publish(temperature_topic.c_str(), String(temp).c_str(), true);
    // }
  }

  if (!isnan(newHum))
  {
    // if (abs(hum - newHum) < 10)
    // {
    hum = newHum;
    Serial.print("Humidity:");
    Serial.println(String(hum).c_str());
    client.publish(humidity_topic.c_str(), String(hum).c_str(), true);
    // }
  }

  if (!isnan(newPress))
  {
    // if (abs(press - newPress) < 10)
    press = newPress;
    Serial.print("Pressure:");
    Serial.println(String(press).c_str());
    client.publish(pressure_topic.c_str(), String(press).c_str(), true);
  }

  if (!isnan(bat))
  {
    Serial.print("Battery:");
    Serial.println(String(bat).c_str());
    client.publish(battery_topic.c_str(), String(bat).c_str(), true);
  }
  delay(300);
  /// @brief APRS SENDING
  //
  //
  //
  //
  int count = 0;
  String packet, aprsauth, tempStr, humStr, presStr;
  Serial.println("Run clientConnectTelNet()");
  while (!espClient.connect(SERVER.c_str(), APRSPORT) && count < 20)
  {
    Serial.println("Didn't connect with server: " + String(SERVER) + " APRSPORT: " + String(APRSPORT));
    delay(1000);
    espClient.stop();
    espClient.flush();
    Serial.println("Run client.stop");
    Serial.println("Trying to connect with server: " + String(SERVER) + " APRSPORT: " + String(APRSPORT));
    count++;
    Serial.println("Try: " + String(count));
  }
  if (count == 20)
  {
    Serial.println("Tried: " + String(count) + " don't send the packet!");
  }
  else
  {
    Serial.println("Connected with server: " + String(SERVER) + " APRSPORT: " + String(APRSPORT));
    // tempStr = getTemp(temp);
    // Serial.println(getTemp(temp));
    float TempF = 0;
    TempF = (temp * 1.8) + 32;
    tempStr = getTemp(TempF);
    // humStr = getHum(hum);
    // Serial.println(getHum(hum));
    humStr = getHum(hum);
    // presStr = getPres(press);
    // Serial.println(getPres(press));
    presStr = getPres(press);
    Serial.println("Leu tempStr=" + tempStr + " humStr=" + humStr + " presStr=" + presStr);
    while (espClient.connected())
    { // there is some problem with the original code from WiFiClient.cpp on procedure uint8_t WiFiClient::connected()
      // it don't check if the connection was close, so you need to locate and change the line below:
      // if (!_client ) to:
      // if (!_client || _client->state() == CLOSED)
      delay(1000);
      Serial.println("Run client.connected()");
      if (tempStr != "-999" || presStr != "-99999" || humStr != "-99")
      {
        aprsauth = "user " + USER + " pass " + PAS + "\n";
        espClient.write(aprsauth.c_str());
        delay(500);
        Serial.println("Send client.write=" + aprsauth);

        packet = USER + ">APRMCU,TCPIP*,qAC,T2BRAZIL:=" + LAT + "/" + LON +
                 "_.../...g...t" + tempStr +
                 "r...p...P...h" + humStr +
                 "b" + presStr + COMMENT + "\n";

        espClient.write(packet.c_str());
        delay(500);
        Serial.println("Send espClient.write=" + packet);

        espClient.stop();
        espClient.flush();
        Serial.println("Telnet client disconnect ");
      }
    }
  }

  digitalWrite(BME_PWR, 0);
  esp_sleep_enable_timer_wakeup(30000000);//30 seconds
  esp_deep_sleep_start();

}

void loop()
{
  
}