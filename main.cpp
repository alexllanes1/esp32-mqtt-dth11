/****************************** Libraries ******************************************/
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"
#include "LITTLEFS.h"
/****************************** Pins ******************************************/

#define DHTPIN 27 // DHT pin connect to GPIO2
#define DHTTYPE DHT11 // DHT sensor type
#define LED_PIN 2

/****************************** Functions ******************************************/

DHT dht(DHTPIN,DHTTYPE, 15); //Class for DHT11/22 sensor

/****************************** MQTT Variables ******************************************/
/*************************** Change the Variables ***************************************/
#define WLAN_SSID "Internet Name"
#define WLAN_PASS "Internet Password"

/* These variables are used for MQTT protocol */
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT "Port Number, erase the quotes and put only a number" 
#define AIO_USERNAME "MQTT Username"
#define AIO_KEY "MQTT Key"


/********** Variables **************/
float humidity_data=0.0;
float temperature_data=0.0;
unsigned long currentMillis=0;
unsigned long previousMillis = 0;
const long wait_publish = 5000;


/********** MQTT Pub/Sub **************/
void connect();
WiFiClient client;
//const char MQTT_SERVER[]PROGMEM=AIO_SERVER;
const char MQTT_SERVER[]PROGMEM=AIO_SERVER;
const char MQTT_CLIENTID[]PROGMEM=AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[]PROGMEM=AIO_USERNAME;
const char MQTT_PASSWORD[]PROGMEM=AIO_KEY;
//Setup the MQTT client class by passing in the WiFi client and MQTT server and login details
Adafruit_MQTT_Client mqtt(&client,MQTT_SERVER,AIO_SERVERPORT,MQTT_CLIENTID,MQTT_USERNAME,MQTT_PASSWORD); //Feeds/
//Setup feeds for temperature and humidity
const char TEMPERATURE_FEED[]PROGMEM=AIO_USERNAME"/feeds/temperature";
Adafruit_MQTT_Publish temperature=Adafruit_MQTT_Publish(&mqtt,TEMPERATURE_FEED);
const char HUMIDITY_FEED[]PROGMEM=AIO_USERNAME"/feeds/humidity";
Adafruit_MQTT_Publish humidity=Adafruit_MQTT_Publish(&mqtt,HUMIDITY_FEED);
const char LED_FEED[] PROGMEM = AIO_USERNAME "/feeds/LED";
Adafruit_MQTT_Subscribe LED = Adafruit_MQTT_Subscribe(&mqtt, LED_FEED);

void setup() {
  
  pinMode(LED_PIN, OUTPUT);

  //Start the DHT11, serial and PINS
  dht.begin();
  Serial.begin(115200);

  
  //Connect to WiFi access point
  Serial.println();Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.print(WLAN_SSID);
  WiFi.begin(WLAN_SSID,WLAN_PASS);
  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address"));
  Serial.println(WiFi.localIP());
  
  mqtt.subscribe(&LED);  

  //MQTT connection
  Serial.println(F("IoT home and MQTT"));
  connect();
  
}
void loop() {

  Adafruit_MQTT_Subscribe *subscription;

    // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

  // Grab the current state of the sensor
  int humidity_data = (int)dht.readHumidity();
  int temperature_data = (int)dht.readTemperature();

  // Publish data
  if (! temperature.publish(temperature_data))
    Serial.println(F("Failed to publish temperature"));
  else
    Serial.println(F("Temperature published!"));

  if (! humidity.publish(humidity_data))
    Serial.println(F("Failed to publish humidity"));
  else
    Serial.println(F("Humidity published!"));

      // this is our 'wait for incoming subscription packets' busy subloop
  while (subscription = mqtt.readSubscription(1000)) {

    // we only care about the LED events
    if (subscription == &LED) {

      // convert mqtt ascii payload to int
      char *value = (char *)LED.lastread;
      //Serial.print(F("Received: "));
      //Serial.println(value);

      // Apply message to LED
      String message = String(value);
      message.trim();
      if (message == "ON") {
          Serial.println(F("ON"));
        digitalWrite(LED_PIN, HIGH);
        //Serial.println("On-board LED ON");
        }
      if (message == "OFF") {
          Serial.println(F("OFF"));
        digitalWrite(LED_PIN, LOW);
        //Serial.println("On-board LED OFF");
        }

    }

  }

  // Repeat every 10 seconds
  delay(5000);

}

// connect to adafruit io via MQTT

void connect(){
  Serial.println(F("Connecting to IoT home..."));
  int8_t ret;
  while((ret=mqtt.connect())!=0)
  {
    switch(ret)
    {
      case 1: Serial.println(F("Wrong protocol"));
        break;
      case 2: Serial.println(F("ID rejected"));
        break;
      case 3: Serial.println(F("Server unvail"));
        break;
      case 4: Serial.println(F("Bad user/pass"));
        break;
      case 5: Serial.println(F("Not authed"));
        break;
      case 6: Serial.println(F("Failed to suscribe"));
        break;
      default: Serial.println(F("Connection failed"));
          break;
    }
    if(ret>=0)
      mqtt.disconnect();
    Serial.println(F("Retrying connection..."));
  }
  Serial.println(F("IoT home Connected!"));
}