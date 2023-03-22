#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// WiFi
#define WIFI_SSID "Network"
#define WIFI_PASS "ypkwgfwf"

// Mqtt
#define MQTT_SERVER "mqtt.eclipseprojects.io"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "1"
#define MQTT_TOPIC "smkt/geo"
#define MQTT_PUBLISH_DELAY 10000

TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 5);

WiFiClient wifiClient;
PubSubClient client(wifiClient);

u_int64_t mqttTimer = millis();

void setup()
{
  Serial.begin(9600);
  SerialGPS.begin(9600);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print("Connecting to WiFi : ");
    Serial.println(WiFi.SSID());
  }
  client.setServer(MQTT_SERVER, MQTT_PORT);

  if (client.connect(MQTT_CLIENT_ID))
  {
    Serial.println("MQTT connected");
  }
  else
  {
    Serial.print("Failed with state ");
    Serial.println(client.state());
    delay(2000);
  }
}

void loop()
{
  while (SerialGPS.available() > 0)
  {
    if (gps.encode(SerialGPS.read()))
    {
      if (gps.location.isValid())
      {
        Serial.println("GPS Connected");
        if (millis() - mqttTimer > MQTT_PUBLISH_DELAY)
        {
          Serial.println("Send message");
          client.publish(MQTT_TOPIC,
                         (
                             F("GEO;") + String(MQTT_CLIENT_ID) + F(";") + String(gps.location.lat()) + F(";") + String(gps.location.lng()))
                             .c_str());
          mqttTimer = millis();
        }
      }
      client.loop();
    }
  }
  client.loop();
}