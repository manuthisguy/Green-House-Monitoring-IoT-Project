#include "DHT.h"
#include <ESP32Servo.h>
#include "VOneMqttClient.h"

#define DHTTYPE DHT11    // DHT11
const int dht11Pin = 6;  // Digital pin connected to the DHT11 sensor

const int ledPinR = 18;  // red LED (Maker: Pin 9, NodeMCU: Pin 5)
const int ledPinY = 16;  // yellow LED (Maker: Pin 6, NodeMCU: Pin 18)
const int ledPinG = 15;  // green LED (Maker: Pin 5, NodeMCU: Pin 19)

const int moistureAO = 4;

Servo servo;
const int servoPin = 21;

const int gasAO = 14;

const int buzzerPin = 12;

const int relayPin = 9;

//////////////////////
const char* DHT11sensor = "9a9c41c9-e991-4b54-ba5f-0794490ed4bd";
const char* MoistureSensor = "5eef287b-2edf-48ff-b1f8-2ad597e18f61";
const char* MQ2sensor = "c6d4dc23-2aba-480c-bff9-fa7c92b91254"; 
//Create an instance of VOneMqttClient
VOneMqttClient voneClient;

//last message time
unsigned long lastMsgTime = 0;

DHT dht(dht11Pin, DHTTYPE);


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void green_light() {
  digitalWrite(ledPinR, LOW);
  digitalWrite(ledPinY, LOW);
  digitalWrite(ledPinG, HIGH);
  Serial.println("LED: Green");
}

void yellow_light() {
  digitalWrite(ledPinR, LOW);
  digitalWrite(ledPinY, HIGH);
  digitalWrite(ledPinG, LOW);
  Serial.println("LED: Yellow");
}

void red_light() {
  digitalWrite(ledPinR, HIGH);
  digitalWrite(ledPinY, LOW);
  digitalWrite(ledPinG, LOW);
  Serial.println("LED: Red");
}

void setup() {
  Serial.begin(9600);
  setup_wifi();   
  voneClient.setup();  // Initiate serial communication
////////////////////////////////

////////////////////////////////
 servo.attach(servoPin);
 servo.write(0);

  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinG, OUTPUT);

  pinMode(moistureAO, INPUT);

  pinMode(gasAO, INPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, false);

  dht.begin();  // Initialize DHT sensor
}

void loop() {
  Serial.print("\n");
  
  delay(1000);                // Set a delay between readings
  float h = dht.readHumidity();     // Read humidity
  float t = dht.readTemperature();   // Read temperature

  int m = analogRead(moistureAO);
  float moisture_percentage = (100 - ( (m/4095.00) * 100 ) );

  float smokeValue = analogRead(gasAO);

  // Check whether the given floating-point number argument is a NaN value
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from the DHT sensor");
    return;
  }

  if (moisture_percentage > 40) {
    green_light();
  } else if (moisture_percentage > 20 && moisture_percentage < 40) {
    yellow_light();
  } else if (moisture_percentage < 20) {
    red_light();
  }

  if(smokeValue > 1200)
  {
    tone(buzzerPin, 1000); // 1000 Hz tone
    Serial.print("Air: Polluted! \n");

    servo.write(120);

    digitalWrite(relayPin, true);
  }
  else if(smokeValue < 1200)
  {
    noTone(buzzerPin); 
    Serial.print("Air: Clear \n");

    digitalWrite(relayPin, false);

    servo.write(0);
    
  }



  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" degrees Celsius");
  Serial.print("\n");

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print("\n");

  Serial.print("Moisture = ");
  Serial.print(moisture_percentage);
  Serial.print("%");
  Serial.print("\n");

  Serial.print("Smoke = ");
  Serial.print(smokeValue);
  //Serial.print("%");
  Serial.print("\n");


  ///////////////////////////
    if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "DHTSensor Fail";
    voneClient.publishDeviceStatusEvent(DHT11sensor, true);
  }
  voneClient.loop();

  unsigned long cur = millis();
  if (cur - lastMsgTime > INTERVAL) {
    lastMsgTime = cur;
   
    //Publish telemetry data

    JSONVar DHT11_Info;    
    DHT11_Info["Humidity"] = h;
    DHT11_Info["Temperature"] = t;
    voneClient.publishTelemetryData(DHT11sensor, DHT11_Info);

    JSONVar SoilMoisture_Info;    
    SoilMoisture_Info["Soil moisture"] = moisture_percentage;
    voneClient.publishTelemetryData(MoistureSensor, SoilMoisture_Info);

    JSONVar MQ2_Info;    
    MQ2_Info["Gas detector"] = smokeValue/100;
    voneClient.publishTelemetryData(MQ2sensor, MQ2_Info);

    //Sample sensor fail message
    //String errorMsg = "DHTSensor Fail";
    //voneClient.publishDeviceStatusEvent(DHT11Sensor, false, errorMsg.c_str());
  }
  
}
