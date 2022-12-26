#include <WiFi.h>
#include <ArduinoHA.h>
#include <Wire.h>
#include <FastLED.h>
#include <FastLED_GFX.h>
#include <FastLEDMatrix.h>
#include <BH1750FVI.h>
#include <ESP32Servo.h> // библиотека для сервомотора

Servo myservo; // назвали сервомотор «myservo»
int pos = 1; // начальная позиция = 1 градус
int prevangle = 1;
#define NUM_LEDS 64
CRGB leds[NUM_LEDS];
#define LED_PIN 18
#define COLOR_ORDER GRB
#define CHIPSET WS2812
#define wind 16  // пин вентилятора GP16
#define pump 17  // пин насоса GP17

BH1750FVI LightSensor_1(BH1750FVI::k_DevModeContHighRes);
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HASensorNumber lightSensor("lightSensor");
HASensor lamp("myLamp");
HASwitch fanSwitch("myFanSwitch");
HASwitch pumpSwitch("myPumpSwitch");
HALight window("window", HALight::BrightnessFeature);

void onFanSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(wind, (state ? HIGH : LOW));
  sender->setState(state);  // report state back to the Home Assistant
}

void onPumpSwitchCommand(bool state, HASwitch* sender) {
  digitalWrite(pump, (state ? HIGH : LOW));
  sender->setState(state);  // report state back to the Home Assistant
}

void onOpenCommand(uint8_t angle, HALight* sender) {
  if (prevangle < angle) {
  for (pos = prevangle; pos <= angle; pos += 1)
  {
  myservo.write(pos);
  delay(15); // если угол задан больше предыдущего, то доводим до нужного угла в ++
  }
  prevangle = angle;
  }
  else if (prevangle > angle) {
  for (pos = prevangle; pos >= angle; pos -= 1)
  {
  myservo.write(pos);
  delay(15); // если угол задан меньше предыдущего, то доводим до нужного угла в --
  }
  prevangle = angle;
  }

  sender->setBrightness(angle); // report brightness back to the Home Assistant
}

void setup() {
  Serial.begin(115200);
  byte mac[6];
  WiFi.macAddress(mac);
  device.setName("Smart greenhouse");
  device.setUniqueId(mac, sizeof(mac));

  Serial.println("Connecting");
  WiFi.begin("DiR-615", "C3c25b212D");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // waiting for the connection
  }
  Serial.println("Connected");

  delay(512);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  LightSensor_1.begin();
  pinMode(wind, OUTPUT);    // настройка пина вентилятора на выход
  pinMode(pump, OUTPUT);    // настройка пина вентилятора на выход
  digitalWrite(wind, LOW);  // изначально выключим проветривание
  digitalWrite(pump, LOW);  // изначально выключим проветривание
  myservo.attach(19);

  lightSensor.setIcon("mdi:weather-sunny");  //
  lightSensor.setName("Brightness");
  lightSensor.setUnitOfMeasurement("lx");

  lamp.setIcon("mdi:wall-sconce-round");
  lamp.setName("Lamplight");

  fanSwitch.setIcon("mdi:fan");
  fanSwitch.setName("Fan");
  fanSwitch.onCommand(onFanSwitchCommand);

  pumpSwitch.setIcon("mdi:water-pump");
  pumpSwitch.setName("Water pump");
  pumpSwitch.onCommand(onPumpSwitchCommand);

  window.setName("Window");
  //window.setIcon("window-closed-variant");
  window.setBrightnessScale(180);
  window.onBrightnessCommand(onOpenCommand);

  mqtt.begin("192.168.0.2", "mqtt", "mqtt");
}

void checkBrightnessAndTurnLights() {
  float l = LightSensor_1.GetLightIntensity();
  lightSensor.setValue(l);
  int power = map(l, 0, 200, 50, 0);
  if (l > 200) {
    power = 0;
    lamp.setValue("OFF");
  } else {
    lamp.setValue("ON");
  }
  FastLED.setBrightness(power);
  fill_solid(leds, NUM_LEDS, CRGB(255, 0, 255));
  FastLED.show();
}

void loop() {
  mqtt.loop();

  checkBrightnessAndTurnLights();

  delay(1000);
}