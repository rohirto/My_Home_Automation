#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h> 
#include <WiFiManager.h> 
#include <ArduinoOTA.h>
#include <WebSocketsClient.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "u_macros.h"

//Defines
//#define DEBUG   

const char* mqtt_server ="192.168.1.149";     //Static raspberry Server
WiFiClient espClient;
PubSubClient client(espClient);
bool connected  = false;

/********Func Prototypes********************************************/
void callback(char*, byte*, unsigned int);  //MQTT callback func
void reconnect(void);     //reconnect to MQTT
void Sensors_setup(void);
void Relay_setup(void);
void getData(void);
void showData(void);
void processData(void);

/****************Control Variables ******************************/
// Doppler Sensor RCWL 0516
const byte RCWL_input_pin = D1;
//Relay Pins
#define RELAY_PIN_SWITCH  D3
#define RELAY_PIN_FAN     D0 //D2 not working
//Auto mode flag
volatile byte auto_mode_flag = false;
volatile byte occupancy_flag = false;

/************ Timers Variables ************************************/
unsigned long startMillis;  //Some global vaiable anywhere in program
unsigned long currentMillis;
volatile byte temp_humd_timer = 30;  // In 10 secs multiple //1 min timer
volatile byte temp_humd_timer_elapsed = false;
volatile byte occupancy_timer_elapsed = false;
volatile byte energy_timer_elapsed = false;
volatile byte ten_sec_counter = 0;
volatile byte occupancy_timer = 6;
volatile byte energy_timer = 6;   // 1 min

/************ NRF Variables ************************************/
/*************NRF24************************/
#define CE_PIN   2
#define CSN_PIN 15
const byte SlaveAddress[5] = {'R','x','A','A','A'};
//const byte EnergySlaveAddress[5] = {'R','x','A','A','B'};
RF24 radio(CE_PIN, CSN_PIN);
char dataReceived[50]; // this must match dataToSend in the TX
bool newData = false;
bool showWeatherData_flag = false;
bool showEnergyData_flag = false;
bool publishWeatherData = false;
bool publishEnergyData = false;
/*******************************************/
/************* Weater Data ********************/
float temperature_data = 0, humidity_data = 0, 
      pressure_data = 0, altitude_data = 0, AQI_data = 0,
      pond_temperature_data = 0, uv_value = 0;
/**********************************************/
//Energy data 
float voltage_data = 0, current_data = 0;


void setup() {
  Serial.begin(115200);
  #if DEBUG
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  #endif
  Serial.println("Booting");

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect("Flop ESP", "espflopflop");
  wifiManager.setSTAStaticIPConfig(IPAddress(192,168,1,150), IPAddress(192,168,1,1), IPAddress(255,255,255,0)); // optional DNS 4th argument
  //wifiManager.resetSettings();    //Uncomment to reset the Wifi Manager

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("My Room");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  /**********************OTA Ends**********************************************************************/
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /* MQTT Settings */
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  SUBSCRIBE();

  /* Basic Setup */
  Sensors_setup();
  Relay_setup();

  /* NRF Setup */
  radio.begin();
  //radio.setPALevel(RF24_PA_LOW); 
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, SlaveAddress);
  //radio.openReadingPipe(2, EnergySlaveAddress);
  radio.startListening();

  //  //Timer start
  startMillis = millis();

}

void loop() {
   char temp_buff1[10];

  /* OTA stuff */
  ArduinoOTA.handle();
  /*****OTA Ends **************/

  /* Wifi Stuff */
  connected = client.connected();
  if (!connected) {
    reconnect();
  }
  client.loop();


  /********** Application Code ********************************/
  //NRF Stuff
  //NRF24 Recieve 
  getData();
  // Process Data 
  processData();
  showData();


   //Occupancy Flag Update 
  if(occupancy_flag == false)
  {
    if(digitalRead(RCWL_input_pin))
    {
      occupancy_flag = true;
    }
  }
  //PIR auto mode related stuff
  if (occupancy_timer_elapsed == true)
  {
    Serial.print(F("\nSending occupancy val "));
    Serial.println(occupancy_flag);
   
    if(connected)
    {
      client.publish(OCCUPANCY_SENSOR_LABEL,itoa(int(occupancy_flag),temp_buff1,10));
      //client.publish(OCCUPANCY_SENSOR_LABEL, occupancy_flag);
      delay(1000);
      //client.loop();
    }
    /**** AUTO MODE *************/
    if(auto_mode_flag == true)
    {
      if(occupancy_flag == false)
      {
        //Off command
        digitalWrite(RELAY_PIN_FAN, HIGH);
        //Off command
        digitalWrite(RELAY_PIN_SWITCH, HIGH);
        
      }
    }
    if(occupancy_flag == true)
    {
      if(!digitalRead(RCWL_input_pin))
      {
        occupancy_flag = false;
      }
    }
    occupancy_timer_elapsed = false;
  }

  /* Publish Energy Data*/
   if(publishEnergyData == true)
    {
      if(connected)
      {
        sprintf(temp_buff1,"%f",voltage_data);
        client.publish(VOLTAGE_LABEL,temp_buff1);
        delay(1000);
        sprintf(temp_buff1,"%f",current_data);
        client.publish(CURRENT_LABEL,temp_buff1);
        delay(1000);
        //client.loop();
      }
      publishEnergyData = false;
    }

  /* Publsh Weather Data */
   if(publishWeatherData == true)
    {
      if(connected)
      {
         // Publish values on server 
         sprintf(temp_buff1,"%f",temperature_data);
        client.publish(TEMPERATURE_LABEL,temp_buff1);
        //client.ubidotsPublish(WEATHER_LABEL);
        
         sprintf(temp_buff1,"%f",humidity_data);
        client.publish(HUMIDITY_LABEL,temp_buff1);
        delay(1000);

       
         sprintf(temp_buff1,"%f",pressure_data);
        client.publish(PRESSURE_LABEL,temp_buff1);
        //client.ubidotsPublish(WEATHER_LABEL);
        
         sprintf(temp_buff1,"%f",altitude_data);
        client.publish(ALTITUDE_LABEL,temp_buff1);
       
        delay(1000);
        
        sprintf(temp_buff1,"%f",AQI_data);
        client.publish(AQI_LABEL,temp_buff1);
        //client.ubidotsPublish(WEATHER_LABEL);
        
         sprintf(temp_buff1,"%f",pond_temperature_data);
        client.publish(POND_TEMPERATURE_LABEL,temp_buff1);
       
        delay(1000);
        
        sprintf(temp_buff1,"%f",uv_value);
        client.publish(UV_VALUE_LABEL,temp_buff1);
        
         delay(1000);
        //client.loop();
      }
      publishWeatherData = false;
    }
  timer_function(); //Update Timers 
  /************** App Code ends *******************************/
}

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char temp_buff[10];
  for (int i = 0; i<10; i++)
  {
    temp_buff[i] = '\0';
  }
  for( int i = 0; i < length; i++)
  {
    temp_buff[i] = (char)payload[i];
    Serial.print((char)payload[i]);  
  }
  Serial.println();
  float f_value = atof(temp_buff);
   if(strcmp(topic,FAN_CONTROL_TOPIC) == 0)
  {
    if(f_value == 1)
    {
      //ON command
       digitalWrite(RELAY_PIN_FAN, LOW);
    }
    else
    {
      //Off command
      digitalWrite(RELAY_PIN_FAN, HIGH);
    }
  }
  else if(strcmp(topic,SWITCH_BOARD_CONTROL_TOPIC) == 0)
  {
    if(f_value == 1)
    {
      //ON command
      digitalWrite(RELAY_PIN_SWITCH, LOW);
    }
    else
    {
      //Off command
      digitalWrite(RELAY_PIN_SWITCH, HIGH);
    }
  }
  else if(strcmp(topic,AUTO_MODE_TOPIC) == 0)
  {
    if(f_value == 1)
    {
      auto_mode_flag = true;
    }
    else
    {
      auto_mode_flag = false;
    }
  }

  //Test Blink
#if DEBUG
int i;
  for(i =0;i<5;i++)
  {
     digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
    delay(1000);                      // Wait for a second
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  }
#endif
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client_xyzsdjf")) {    //This MQTT CLient ID needs to be Unique
      Serial.println("connected");
      // Subscribe
      SUBSCRIBE();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void timer_function()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if ( currentMillis - startMillis >= 10000)
  {
    startMillis = currentMillis;
    ten_sec_counter++;

    if ((ten_sec_counter % occupancy_timer) == 0)
    {
      occupancy_timer_elapsed = true;
    }
    if((ten_sec_counter % energy_timer) == 0)
    {
      energy_timer_elapsed = true;
    }
    if ((ten_sec_counter % temp_humd_timer) == 0) //test whether the period has elapsed
    {
      //temp_humd_timer_elapsed = true;
      ten_sec_counter = 0;  //IMPORTANT to save the start time of the current LED state.
    }
  }

}

void getData() {
  uint8_t pipe;
    if ( radio.available(&pipe) ) {
      uint8_t bytes = radio.getPayloadSize();
        radio.read( &dataReceived,bytes );
        Serial.println(dataReceived);
        newData = true;
    }
}

void showData() {
  if(showWeatherData_flag == true){
    Serial.print(temperature_data);
    Serial.print("\t");
    Serial.print(humidity_data);
    Serial.print("\t");
    Serial.print(pressure_data);
    Serial.print("\t");
    Serial.print(altitude_data);
    Serial.print("\t");
    Serial.println(AQI_data);
    Serial.print("\t");
    Serial.print(pond_temperature_data);
    Serial.print("\t");
    Serial.println(uv_value);
    showWeatherData_flag = false;
  }
  if(showEnergyData_flag == true)
  {
    Serial.print(voltage_data);
    Serial.print("\t");
    Serial.println(current_data);
    showEnergyData_flag = false;
  }
    
}
void processData() {
  char str_temp[10];
  if(newData == true) {
    char* str_ptr = dataReceived;
    char* str_ptr1 = str_temp;
    switch(*str_ptr++){
      case '1':
      //Temperature Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      temperature_data = atof(str_temp);
      break;

      case '2':
      //Humidity Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      humidity_data = atof(str_temp);
      break;

      case '3':
      //Pressure Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      pressure_data = atof(str_temp);
      break;
      
      case '4':
      //Altitude Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      altitude_data = atof(str_temp);
      break;

      case '5':
      //AQI Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      AQI_data = atof(str_temp);
      break;
      case '6':
      //Pond Temperature Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      pond_temperature_data = atof(str_temp);
      //showWeatherData_flag = true;
      //publishWeatherData = true;
      break;
       case '9':
      //UV Index
       if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      uv_value = atof(str_temp);
      showWeatherData_flag = true;
      publishWeatherData = true;
      break;
      
      case '7':
      //Voltage Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      voltage_data = atof(str_temp);
      break;
      case '8':
      //Current Data 
      if(*str_ptr++ == '~'){
        while(*str_ptr != '~') {
          *str_ptr1++ = *str_ptr++;
        }
      }
      current_data = atof(str_temp);
      showEnergyData_flag = true;
      publishEnergyData = true;
      break;
    }
  }
  newData = false;
}
void Sensors_setup()
{

 //RCWL Motion Sensor/ Doppler Sensor
  pinMode(RCWL_input_pin, INPUT);

}
void Relay_setup()
{
  //define relay pins and their state
  pinMode(RELAY_PIN_SWITCH, OUTPUT);
  digitalWrite(RELAY_PIN_SWITCH, HIGH);
  pinMode(RELAY_PIN_FAN, OUTPUT);
  digitalWrite(RELAY_PIN_FAN, HIGH);
}