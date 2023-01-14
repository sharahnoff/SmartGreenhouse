#include <WiFi.h>       // библиотека для wifi
#include <ArduinoHA.h>  // библиотека для подключения к home assistant
#include <Wire.h>       // библиотека для общения датчика по I2C интерфейсу
#include <FastLED.h>    // библиотеки для матрицы
#include <FastLED_GFX.h>
#include <FastLEDMatrix.h>
#include <BH1750FVI.h>         // библиотека датчика освещенности
#include <ESP32Servo.h>        // библиотека для сервомотора
#include <Adafruit_ADS1015.h>  // библиотека для аналогоцифрового преобразователя ADS1015
#include <VEML6075.h>          // остальные библиотеки для датчиков
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme280;                                    // датчик температуры и давления воздуха
VEML6075 veml6075;                                         // датчик ультрафиолета
BH1750FVI LightSensor_1(BH1750FVI::k_DevModeContHighRes);  // датчик освещенности
Servo myservo;                                             // назвали сервомотор «myservo»
int pos = 0;                                               // начальная позиция = 0 градус
char mooving = 's';                                        // текущее состояние окна
Adafruit_ADS1015 ads(0x48);                                // датчик температуры и влажности воздуха
const float air_value = 84148.0;                           // Показание АЦП на воздухе
const float water_value = 46792.0;                         // Показание АЦП в воде
HALight::RGBColor curColor;                                // текущий цвет матрицы
bool ledState = false;                                     // состояние матрицы
#define NUM_LEDS 64                                        // количество светодиодов в матрице - 64
CRGB leds[NUM_LEDS];                                       // называем матрицу - leds
#define LED_PIN 18                                         // пин к которому подключена матрица - 18
#define COLOR_ORDER GRB                                    // порядок цветов матрицы
#define CHIPSET WS2812                                     // тип светодиодов
#define wind 16                                            // пин вентилятора GP16
#define pump 17                                            // пин насоса GP17
char ssid[] = "DiR-615";                                   // Логин Wi-Fi
char pass[] = "C3c25b212D";                                // Пароль от Wi-Fi
char mqttlog[] = "mqtt";                                   // Логин Mosquito Broker
char mqttpass[] = "mqtt";                                  // Пароль Mosquito Broker

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 15);                // добавление элементов в HA
HASensorNumber airTempSensor("airTempSensor");  // добавление сенсоров
HASensorNumber airHumidSensor("airHumidSensor");
HASensorNumber airPresSensor("airPresSensor");
HASensorNumber lightSensor("lightSensor");
HASensorNumber tempSensor("tempSensor");
HASensorNumber wetSensor("wetSensor");
HASensorNumber UVASensor("uvaSensor");
HASensorNumber UVBSensor("uvbSensor");
HASensorNumber UVSensor("uvSensor");
HASwitch fanSwitch("myFanSwitch");                                               // выключатель вентилятора
HASwitch pumpSwitch("myPumpSwitch");                                             // выключатель водяной помпы
HACover cover("myWindow");                             // форточка
HALight light("prettyLight", HALight::BrightnessFeature | HALight::RGBFeature);  // освещение

void onLightStateCommand(bool state, HALight* sender) {  // действия при нажатии кнопки включения освещения в НА
  if (state && state != ledState) {
    FastLED.setBrightness(50);                                                      // установка яркости матрицы
    fill_solid(leds, NUM_LEDS, CRGB(curColor.red, curColor.green, curColor.blue));  // установка цвета матрицы
    FastLED.show();                                                                 // отобразить установки матрицы
    sender->setBrightness(50);                                                      // отправка значения яркости матрицы в НА
  }
  if (!state && state != ledState) {
    FastLED.setBrightness(0);
    fill_solid(leds, NUM_LEDS, CRGB(curColor.red, curColor.green, curColor.blue));
    FastLED.show();
    sender->setBrightness(0);
  }
  ledState = state;
  sender->setState(state);  // отправка состояния матрицы в НА
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {  // действия при изменении яркости в НА
  FastLED.setBrightness(brightness);
  fill_solid(leds, NUM_LEDS, CRGB(curColor.red, curColor.green, curColor.blue));
  FastLED.show();
  sender->setBrightness(brightness);
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {  // действия при изменении цвета в НА
  fill_solid(leds, NUM_LEDS, CRGB(color.red, color.green, color.blue));
  curColor = color;
  FastLED.show();
  sender->setRGBColor(color);
}

void onFanSwitchCommand(bool state, HASwitch* sender) {  // действие при переключении выключателя вентилятора в НА
  digitalWrite(wind, (state ? HIGH : LOW));
  sender->setState(state);
}

void onPumpSwitchCommand(bool state, HASwitch* sender) {  // действие при переключении выключателя водяной помпы в НА
  digitalWrite(pump, (state ? HIGH : LOW));
  sender->setState(state);
}

void onCoverCommand(HACover::CoverCommand cmd, HACover* sender) {  // действия при работе с форточкой
  if (cmd == HACover::CommandOpen) {
    sender->setState(HACover::StateOpening);
    mooving = 'o';
  } else if (cmd == HACover::CommandClose) {
    sender->setState(HACover::StateClosing);
    mooving = 'c';
  } else if (cmd == HACover::CommandStop) {
    mooving = 's';
    sender->setState(HACover::StateStopped);
  }
}

void setup() {
  Serial.begin(115200);  // настройка скорости обмена данными по последовательному интерфейсу

  byte mac[6];  // настройка устройства для Home assistant
  WiFi.macAddress(mac);
  device.setName("Smart Greenhouse");
  device.setUniqueId(mac, sizeof(mac));

  Serial.println("Connecting");  // подключение к WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // ожидание подключения к WiFi
  }
  Serial.println("Connected");

  Wire.begin(21, 22);  // добавляем последовательный порт
  Wire.setClock(10000L);
  if (!veml6075.begin())  // включаем и проверяем датчик ультрафиолета
    Serial.println("VEML6075 not found!");

  bool bme_status = bme280.begin();  // включение датчика воздуха и его проверка
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);  // объявляем нашу матрицу
  curColor.red = 255;                                  // устанавливаем изначальный цвет матрицы
  curColor.green = 0;
  curColor.blue = 0;

  LightSensor_1.begin();        // включение датчика освещенности
  ads.setGain(GAIN_TWOTHIRDS);  // включение датчика температуры и влажности почвы
  ads.begin();
  pinMode(wind, OUTPUT);    // настройка пина вентилятора на выход
  pinMode(pump, OUTPUT);    // настройка пина помпы на выход
  digitalWrite(wind, LOW);  // изначально выключим проветривание
  digitalWrite(pump, LOW);  // изначально выключим полив
  myservo.attach(19);       // пин 19 для сервомотора
  myservo.write(pos);       // перемещаем сервомотор в определенное положение

  cover.onCommand(onCoverCommand);  // назначение обработчика команд от НА
  cover.setName("My window");       // присваиваем имя для отображения в НА

  light.setName("LED matrix");
  light.onStateCommand(onLightStateCommand);
  light.setBrightnessScale(50);                    // устанавливаем диапазон яркости в НА
  light.onBrightnessCommand(onBrightnessCommand);  // назначение обработчика команд от НА
  light.onRGBColorCommand(onRGBColorCommand);

  lightSensor.setIcon("mdi:weather-sunny");  // устанавливаем иконку для отображения в НА
  lightSensor.setName("Brightness");
  lightSensor.setUnitOfMeasurement("lx");  // устанавливаем единицы измерения

  UVASensor.setIcon("mdi:sun-wireless");
  UVASensor.setName("UVA");
  UVASensor.setUnitOfMeasurement(" ");

  UVBSensor.setIcon("mdi:sun-wireless");
  UVBSensor.setName("UVB");
  UVBSensor.setUnitOfMeasurement(" ");

  UVSensor.setIcon("mdi:sun-wireless");
  UVSensor.setName("UV");
  UVSensor.setUnitOfMeasurement(" ");

  tempSensor.setIcon("mdi:thermometer");
  tempSensor.setName("Ground temperature");
  tempSensor.setUnitOfMeasurement("C");

  wetSensor.setIcon("mdi:water-outline");
  wetSensor.setName("Ground wetness");
  wetSensor.setUnitOfMeasurement("%");

  airTempSensor.setIcon("mdi:thermometer");
  airTempSensor.setName("Air temperature");
  airTempSensor.setUnitOfMeasurement("C");

  airHumidSensor.setIcon("mdi:water-percent");
  airHumidSensor.setName("Air humidity");
  airHumidSensor.setUnitOfMeasurement("%");

  airPresSensor.setIcon("mdi:gauge");
  airPresSensor.setName("Air pressure");
  airPresSensor.setUnitOfMeasurement("hPa");

  fanSwitch.setIcon("mdi:fan");
  fanSwitch.setName("Fan");
  fanSwitch.onCommand(onFanSwitchCommand);

  pumpSwitch.setIcon("mdi:water-pump");
  pumpSwitch.setName("Water pump");
  pumpSwitch.onCommand(onPumpSwitchCommand);

  mqtt.begin("192.168.0.2", mqttlog, mqttpass);  // подключение к сервверу home assistant
}

void checkBrightness() {
  float l = LightSensor_1.GetLightIntensity();  // получаем данные с датчика освещенности
  lightSensor.setValue(l);                      // отправляем данные на сервевр
}

void checkTempAndWet() {  // чтение и отправка значений с датчика почвы
  float adc0 = (float)ads.readADC_SingleEnded(0) * 6.144 * 16;
  float adc1 = (float)ads.readADC_SingleEnded(1) * 6.144 * 16;
  float t1 = ((adc1 / 1000));
  float h1 = map(adc0, air_value, water_value, 0.0, 100.0);
  tempSensor.setValue(t1);
  wetSensor.setValue(h1);
}

void checkUV() {  // чтение и отправка значений с датчика ультрафиолета
  veml6075.poll();
  float uva = veml6075.getUVA();
  float uvb = veml6075.getUVB();
  float uv_index = veml6075.getUVIndex();
  UVASensor.setValue(uva);
  UVBSensor.setValue(uvb);
  UVSensor.setValue(uv_index);
}

void checkAir() {  // чтение и отправка значений с датчика воздуха
  airTempSensor.setValue(bme280.readTemperature());
  airHumidSensor.setValue(bme280.readHumidity());
  airPresSensor.setValue(bme280.readPressure() / 100.0F);
}

void moveCover(int diff) {  // функция вращения форточки на заданный угол
  int newPos = pos + diff;
  if (newPos >= 0 && newPos <= 180) {
    myservo.write(newPos);
    pos = newPos;
  } else {
    mooving = 's';
    cover.setState(HACover::StateStopped);
  }
}

void checkCoverMove() {  // проверка состояния форточки
  switch (mooving) {
    case 'o':
      moveCover(5);
      break;
    case 'c':
      moveCover(-5);
      break;
  }
}

void loop() {
  mqtt.loop();        // поддержание связи с НА
  checkBrightness();  // проверка датчиков и устройств умной теплички
  checkTempAndWet();
  checkUV();
  checkAir();
  checkCoverMove();
  delay(256);  // задержка 256 мс
}