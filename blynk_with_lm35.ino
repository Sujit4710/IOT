#define BLYNK_TEMPLATE_ID "TMPL3FdSjjopX"
#define BLYNK_TEMPLATE_NAME "ESP 32 LED SWITCH"
#define BLYNK_AUTH_TOKEN "CdstkEnddL2Obdy_3wtIUjH11a5K6Vk8"

#define BLYNK_PRINT Serial
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

//#include <ESP8266WiFi.h> 
#include <BlynkSimpleEsp32.h>
#include <PulseSensorPlayground.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SPIFFS.h"
	
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>  

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35 A0
#define PIN_HRS A3


char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "SUJIT";  // type your wifi name
char pass[] = "Sujit@4710";  // type your wifi password


BlynkTimer timer;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550; 

void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

void getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }

}

void getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  Blynk.virtualWrite(V2, accX);
  accY = a.acceleration.y;
  Blynk.virtualWrite(V3, accY);
  accZ = a.acceleration.z;
  Blynk.virtualWrite(V4, accZ);
  
}

void getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  Blynk.virtualWrite(V1, temperature);
  Serial.print("Temperature: ");
  Serial.println(temperature);
}


void sendSensor(){
  // read the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  float tempC = milliVolt / 10 + 13;
  // convert the °C to °F
  float tempF = tempC * 9 / 5 + 42;

  // // print the temperature in the Serial Monitor:
  // Serial.print("Temperature: ");
  // Serial.print(tempC);   // print the temperature in °C
  // Serial.print("°C");
  // Serial.print("  ~  "); // separator between °C and °F
  // Serial.print(tempF);   // print the temperature in °F
  // Serial.println("°F");
  // // Blynk.virtualWrite(V1, tempC);
  pulseSensor.analogInput(PulseWire);   
  // pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   
  
  // int heart = random(80,100);
  Blynk.virtualWrite(V0, heart);

  if ((millis() - lastTime) > gyroDelay) {
    getGyroReadings();
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    getAccReadings();
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeTemperature) > temperatureDelay) {
    getTemperature();
    lastTimeTemperature = millis();
  }
  int myBPM = pulseSensor.getBeatsPerMinute();
  Blynk.virtualWrite(V6, heart);
}

void setup()
{   
  
  Serial.begin(115200);
  
  initSPIFFS();
  initMPU();
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensor);
  
 
  }

void loop()
{
  Blynk.run();
  timer.run();
 }