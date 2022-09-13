#ifndef U_MACROS_H
#define U_MACROS_H
//#define RASP_MQTT       0

#define TOKEN                       "/*Removed For Github upload*/" 
#define MQTT_CLIENT_NAME            "NodeMCU"
/********** Control Variables *******************************/
#ifdef RASP_MQTT
#define FAN_CONTROL_TOPIC           "home-automation/fan-control"
#define SWITCH_BOARD_CONTROL_TOPIC  "home-automation/switch-board-control"
#define AUTO_MODE_TOPIC             "home-automation/auto-mode"  
#define OCCUPANCY_SENSOR_LABEL      "occupancy-sensor"
#define SUBSCRIBE()     client.subscribe(FAN_CONTROL_TOPIC);\
                        client.subscribe(SWITCH_BOARD_CONTROL_TOPIC);\
                        client.subscribe(AUTO_MODE_TOPIC);
#else
#define HOME_AUTO_LABEL             "home-automation"
#define FAN_CONTROL_TOPIC           "/v1.6/devices/home-automation/fan-control/lv"
#define SWITCH_BOARD_CONTROL_TOPIC  "/v1.6/devices/home-automation/switch-board-control/lv"
#define AUTO_MODE_TOPIC             "/v1.6/devices/home-automation/auto-mode/lv"  
#define OCCUPANCY_SENSOR_LABEL      "occupancy-sensor"
#define SUBSCRIBE()     char* topicToSubscribe;\
                         sprintf(topicToSubscribe, "%s", "");\
                          sprintf(topicToSubscribe, "%s", FAN_CONTROL_TOPIC);\
                          client.subscribe(topicToSubscribe);\
                         sprintf(topicToSubscribe, "%s", "");\
                          sprintf(topicToSubscribe, "%s", SWITCH_BOARD_CONTROL_TOPIC);\
                          client.subscribe(topicToSubscribe);\
                          sprintf(topicToSubscribe, "%s", "");\
                          sprintf(topicToSubscribe, "%s", AUTO_MODE_TOPIC);\
                          client.subscribe(topicToSubscribe);
                         
#endif

#define ENERGY_MONITORING_LABEL     "energy-monitoring"
#define WEATHER_LABEL               "weather"

#define TEMPERATURE_LABEL           "temperature"
#define HUMIDITY_LABEL              "humidity"
#define PRESSURE_LABEL              "pressure"
#define ALTITUDE_LABEL              "altitude"
#define AQI_LABEL                   "aqi"
#define POND_TEMPERATURE_LABEL      "pond-temperature"
#define UV_VALUE_LABEL              "uv-value"

#define VOLTAGE_LABEL               "voltage"
#define CURRENT_LABEL               "current"

#define TEMPERATURE_FIELD_NO      1
#define HUMIDITY_FIELD_NO         2
#define PRESSURE_FIELD_NO         3
#define ALTITUDE_FIELD_NO         4
#define AQI_FIELD_NO              5
#define POND_TEMPERATURE_FIELD_NO 6
#define VOLTAGE_FIELD_NO          7
#define CURRENT_FIELD_NO          8
#define UV_SENSOR_INDEX_NO        9

#endif