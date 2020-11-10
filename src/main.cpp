#include <Arduino.h>
#include "Wire.h"
#include "MPU6050.h"
#include "RunningAverage.h"
#include "FastLED.h"
#include "LittleFS.h"
#include "ArduinoJson.h"

#define NUM_LEDS 1
#define DATA_PIN D7
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define LIGHT_PIN D8
#define FRONT_LIGHT_BUTTON D6
#define JSON_NAME "config.json"
#define DESERIALISATION_CAPACITY 2*JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(3) + 40
#define SERIALISATION_CAPACITY 2*JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(3)

//for leds
CRGB leds[NUM_LEDS];

//MPU6050 accelgyro;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
MPU6050 accelgyro(MPU_addr);

int16_t ax, ay, az;
int16_t gx, gy, gz;

//for running averages
int nb_vals = 20;

RunningAverage val_ax(nb_vals);
RunningAverage val_ay(nb_vals);
RunningAverage val_az(nb_vals);
RunningAverage val_gx(nb_vals);
RunningAverage val_gy(nb_vals);
RunningAverage val_gz(nb_vals);

//program's values
long current_millis;
long blinker_millis;
long reader_millis;
long light_stop_millis;
long last_loop;
unsigned char light_stop_power;
bool stop_light_state;
bool operation;
bool turning;
bool blinker_state;
bool front_light_state;
bool front_light_state_stop;
bool front_light_state_blinker;

// #define LED_PIN 13
// bool blinkState = false;

void light_stoping() {
  if (current_millis - light_stop_millis >= 10) {
    if (light_stop_power >= 255 && operation)
      operation = false;
    if (light_stop_power <= 0 && !operation)
      operation = true;
    if (operation)
      light_stop_power += 15;
    if (!operation)
      light_stop_power -= 15;
    leds[0] = CRGB(light_stop_power, 0, 0);
    light_stop_millis = current_millis;
    FastLED.show();
  }
}

void blinker() {
  if (current_millis - blinker_millis >= 500) {
    if (!blinker_state) {
      leds[0] = CRGB(255, 65, 0);
      blinker_state = true;
    }
    else {
      leds[0] = CRGB(0, 0, 0);
      blinker_state = false;
    }
    blinker_millis = current_millis;
    FastLED.show();
  }
}

void init_led() {
  digitalWrite(LIGHT_PIN, HIGH);
  for (int r = 0; r < 255; r+=2) {
    leds[0] = CRGB(r, 0, 0);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, LOW);
  for (int r = 255; r > 1; r-=2) {
    leds[0] = CRGB(r, 0, 0);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, HIGH);
  for (int g = 0; g < 255; g+=2) {
    leds[0] = CRGB(0, g, 0);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, LOW);
  for (int g = 255; g > 1; g-=2) {
    leds[0] = CRGB(0, g, 0);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, HIGH);
  for (int b = 0; b < 255; b+=2) {
    leds[0] = CRGB(0, 0, b);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, LOW);
  for (int b = 255; b > 1; b-=2) {
    leds[0] = CRGB(0, 0, b);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, HIGH);
  for (int a = 0; a < 255; a+=2) {
    leds[0] = CRGB(a, a, a);
    FastLED.show();
    delay(2);
  }
  digitalWrite(LIGHT_PIN, LOW);
  for (int a = 255; a > 1; a-=2) {
    leds[0] = CRGB(a, a, a);
    FastLED.show();
    delay(2);
  }
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void read_file(File file) {
  DynamicJsonDocument doc(DESERIALISATION_CAPACITY);
  deserializeJson(doc, file);
  JsonArray accelerometer = doc["accelerometer"];
  JsonArray gyroscope = doc["gyroscope"];
  accelgyro.setXGyroOffset(gyroscope[0]);
  accelgyro.setYGyroOffset(gyroscope[1]);
  accelgyro.setZGyroOffset(gyroscope[2]);
  accelgyro.setXAccelOffset(accelerometer[0]);
  accelgyro.setYAccelOffset(accelerometer[1]);
  accelgyro.setZAccelOffset(accelerometer[2]);
  file.close();
}

File calibration(File file) {

  return (file);
}

File test_file(File file) {
  DynamicJsonDocument doc(DESERIALISATION_CAPACITY);
  if (deserializeJson(doc, file)) {
    file.close();
    create_file();
    file = LittleFS.open(JSON_NAME, "r");
    calibration(file);
  } else {
    if (!doc["calibration"]) {
      calibration(file);
    }
  }
  return (file);
}

void create_file() {
  File file = LittleFS.open(JSON_NAME, "w");
  file.print("{\"calibration\":false,\"accelerometer\":[0,0,0],\"gyroscope\":[0,0,0]}");
  file.close();
}

void setup() {
    LittleFS.begin();
    File file = LittleFS.open(JSON_NAME, "r");
    if (!file){
      file.close();
      create_file();
      File file = LittleFS.open(JSON_NAME, "r");
    }
    file = test_file(file);
    read_file(file);
    LittleFS.end();
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");

    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Updating internal sensor offsets...");

    val_ax.fillValue(0, nb_vals);
    val_ay.fillValue(0, nb_vals);
    val_az.fillValue(0, nb_vals);
    val_gx.fillValue(0, nb_vals);
    val_gy.fillValue(0, nb_vals);
    val_gz.fillValue(0, nb_vals);

    current_millis = 0;
    blinker_millis = 0;
    reader_millis = 0;
    light_stop_millis = 0;
    last_loop = 0;
    light_stop_power = 0;
    stop_light_state = false;
    turning = false;
    blinker_state = false;
    operation = true;
    front_light_state = true;
    front_light_state_stop = false;
    front_light_state_blinker = false;

    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    pinMode(LIGHT_PIN, OUTPUT);
    digitalWrite(LIGHT_PIN, LOW);
    pinMode(FRONT_LIGHT_BUTTON, INPUT_PULLUP);
    init_led();
}

void loop() {
  last_loop = current_millis;
  current_millis = millis();

  if (last_loop > current_millis) {
    blinker_millis = 0;
    reader_millis = 0;
    light_stop_millis = 0;
    last_loop = 0;
  }

  if (current_millis - reader_millis >= 50) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    val_ax.addValue(ax);
    val_ay.addValue(ay);
    val_az.addValue(az);
    val_gx.addValue(gx);
    val_gy.addValue(gy);
    val_gz.addValue(gz);
    reader_millis = current_millis;

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("   ");
    Serial.print(ay); Serial.print("   ");
    Serial.print(az); Serial.print("   ");
    Serial.print("          ");
    Serial.print(gx); Serial.print("   ");
    Serial.print(gy); Serial.print("   ");
    Serial.print(gz); Serial.print("   ");
    Serial.println("");
  }

  if ((val_ay.getAverage() > 10000) || (val_ay.getAverage() < -10000)) {
    turning = true;
  }
  if ((val_ay.getAverage() < 10000) && (val_ay.getAverage() > -10000)) {
    turning = false;
  }
  if (turning) {
    blinker();
    front_light_state_blinker = false;
  //Serial.print("TURNING");
  } else {
    blinker_state = false;
    front_light_state_blinker = true;
  }

  if (val_ax.getAverage() > 10000) {
    stop_light_state = true;
    front_light_state_stop = false;
  } else {
    stop_light_state = false;
    front_light_state_stop = true;
  }

  if (stop_light_state) {
    light_stoping();
  }

  if (!stop_light_state && !turning) {
    leds[0] = CRGB(0 ,0 ,0);
    FastLED.show();
  }

  if (!front_light_state_blinker || !front_light_state_stop) {
    front_light_state = false;
  } else {
    front_light_state = true;
  }

  if (front_light_state == true && digitalRead(FRONT_LIGHT_BUTTON) == LOW) {
    digitalWrite(LIGHT_PIN, HIGH);
  } else {
    digitalWrite(LIGHT_PIN, LOW);
  }
}