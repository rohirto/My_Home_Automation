#ifndef U_MACROS_H
#define U_MACROS_H
/********** Control Variables *******************************/
#define FAN_CONTROL_TOPIC           "home-automation/fan-control"
#define SWITCH_BOARD_CONTROL_TOPIC  "home-automation/switch-board-control"
#define AUTO_MODE_TOPIC             "home-automation/auto-mode"  
#define OCCUPANCY_SENSOR_LABEL      "occupancy-sensor"

#define SUBSCRIBE()     client.subscribe(FAN_CONTROL_TOPIC);\
                        client.subscribe(SWITCH_BOARD_CONTROL_TOPIC);\
                        client.subscribe(AUTO_MODE_TOPIC);

#define TEMPERATURE_LABEL           "temperature"
#define HUMIDITY_LABEL              "humidity"
#define PRESSURE_LABEL              "pressure"
#define ALTITUDE_LABEL              "altitude"
#define AQI_LABEL                   "aqi"
#define POND_TEMPERATURE_LABEL      "pond-temperature"
#define UV_VALUE_LABEL              "uv-value"

#define VOLTAGE_LABEL               "voltage"
#define CURRENT_LABEL               "current"

#endif