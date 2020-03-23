#include "time.h"

#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_wifi.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

const char *ssid     = "XXXXXX";
const char *password = "XXXXXX";

const char *       metrics_server = "127.0.0.1";
const unsigned int metrics_port   = 19501;

const char *ntpServer = "ntp.se";

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 5 * 1

void connectWiFi()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void disconnectWiFi()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    esp_bluedroid_disable();
    esp_bt_controller_disable();
}

struct SensorData {
    time_t timestamp;
    float  temperature;
    float  pressure;
    float  humidity;
};

void checkBME(float &temperature, float &pressure, float &humidity)
{
    if (bme.begin() == 0) {
        Serial.println("Could not find a valid BME280 sensor");
        return;
    }

    temperature = bme.readTemperature();
    pressure    = bme.readPressure() / 100.0F;
    humidity    = bme.readHumidity();
}

void sendMessage(struct SensorData &sensorData)
{
    WiFiClient client;
    if (!client.connect(metrics_server, metrics_port)) {
        Serial.println("connection failed");
        return;
    }

    StaticJsonDocument<200> doc;
    doc["sensor"]      = "greenhouse";
    doc["timestamp"]   = sensorData.timestamp;
    doc["temperature"] = sensorData.temperature;
    doc["pressure"]    = sensorData.pressure;
    doc["humidity"]    = sensorData.humidity;
    serializeJson(doc, client);

    client.stop();
}

void setup()
{
    Serial.begin(115200);

    connectWiFi();

    configTime(0, 0, ntpServer);
}

void loop()
{
    struct SensorData sensorData;

    time(&sensorData.timestamp);

    checkBME(sensorData.temperature, sensorData.pressure, sensorData.humidity);

    sendMessage(sensorData);

    disconnectWiFi();

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}
