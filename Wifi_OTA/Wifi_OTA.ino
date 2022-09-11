#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h> 
#include <WiFiManager.h> 
#include <ArduinoOTA.h>
#include <WebSocketsClient.h>
#include <DNSServer.h>
#include <PubSubClient.h>

//Defines
//#define DEBUG   

const char* mqtt_server ="test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);

/********Func Prototypes********************************************/
void callback(char*, byte*, unsigned int);  //MQTT callback func
void reconnect(void);     //reconnect to MQTT

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
  client.subscribe("test/output");

}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
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
      client.subscribe("test/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}