#include "connectivity_task.h"
#include "WiFi.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT_Client.h"
#include "ArduinoJson.h"
#include "secrets.h"
#include "serial/serial_protocol_plaintext.h"
#include "proto_gen/smartknob.pb.h"



WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_PW);

String smartknob_id = String(SMARTKNOB_ID);

String topic = "homeassistant/device_automation/knob_" + smartknob_id + "/action_knob/config";
String action_topic = "knob/" + smartknob_id + "/action";
Adafruit_MQTT_Publish discovery_feed = Adafruit_MQTT_Publish(&mqtt, topic.c_str());
Adafruit_MQTT_Publish action_feed = Adafruit_MQTT_Publish(&mqtt, action_topic.c_str());

ConnectivityTask::ConnectivityTask(const uint8_t task_core) : Task("Connectivity", 4096, 1, task_core) {
    queue_ = xQueueCreate(5, sizeof(Message));
    assert(queue_ != NULL);
}

ConnectivityTask::~ConnectivityTask() {}

void sendMqttKnobStateDiscoveryMsg() {
    DynamicJsonDocument payload(1024);
    char buffer[512];
    JsonObject device;
    JsonArray identifiers;

    payload["automation_type"] = "trigger";
    payload["topic"] = action_topic;
    device = payload.createNestedObject("device");
    device["name"] = "SmartKnob";
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(smartknob_id);

    discovery_feed.publish(buffer);
}

void ConnectivityTask::run() {
    
    initWiFi();
    connectToMqttBroker();
    sendMqttKnobStateDiscoveryMsg();

    while (1) {
        connectToMqttBroker();

        Message message;
        if (xQueueReceive(queue_, &message, 0) == pdTRUE) {
            
            StaticJsonDocument<200> payload;
            char buffer[256];
            payload["trigger_name"] = message.trigger_name;
            payload["trigger_value"] = message.trigger_value;
            size_t n = serializeJson(payload, buffer);

            action_feed.publish(buffer);
        }

        delay(1);
    }
}

void ConnectivityTask::sendMqttMessage(Message message) {
    xQueueSend(queue_, &message, portMAX_DELAY);
}

void ConnectivityTask::initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    //printf("Connecting to WiFi...\n");
    delay(1000);
  }
  printf("Wifi Connected!\n");
  Serial.println(WiFi.localIP());
  
}

void ConnectivityTask::connectToMqttBroker() {
  int8_t ret;

  
  
  // Stop if already connected.
  if (mqtt.connected()) {
    
    return;
} else {
    //printf("Connecting to MQTT... \n");
}
  

    

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       //printf("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  printf("MQTT Connected!");
}