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
#define JSON_NAME "/config.json"
#define DESERIALISATION_CAPACITY 2*JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(3) + 40
#define SERIALISATION_CAPACITY 2*JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(3)

//for leds
CRGB leds[NUM_LEDS];

//MPU6050 accelgyro;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
MPU6050 accelgyro(MPU_addr);

int16_t ax, ay, az;
int16_t gx, gy, gz;

//for calibration system
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

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

bool is_in_interval(int number, int min, int max, bool included){
  if (included) {
    if (number <= max && number >= min)
      return (true);
    else
      return (false);
  } else {
    if (number < max && number > min)
      return (true);
    else
      return (false);
    }
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

void print_file() {
  File file = LittleFS.open(JSON_NAME, "r");
  Serial.println("print the interior of the file");
  while (file.available())
    Serial.write(file.read());
  file.close();
  Serial.println("");
}

void read_file() {
  print_file();
  File file = LittleFS.open(JSON_NAME, "r");
  DynamicJsonDocument doc(DESERIALISATION_CAPACITY);
  deserializeJson(doc, file);
  JsonArray accelerometer = doc["accelerometer"];
  JsonArray gyroscope = doc["gyroscope"];
  int gx_save = gyroscope[0];
  int gy_save = gyroscope[1];
  int gz_save = gyroscope[2];
  int ax_save = accelerometer[0];
  int ay_save = accelerometer[1];
  int az_save = accelerometer[2];
  Serial.println("gyro save:");
  Serial.println(gx_save);
  Serial.println(gy_save);
  Serial.println(gz_save);
  Serial.println("accéléro save :");
  Serial.println(ax_save);
  Serial.println(ay_save);
  Serial.println(az_save);
  accelgyro.setXGyroOffset(gyroscope[0]);
  accelgyro.setYGyroOffset(gyroscope[1]);
  accelgyro.setZGyroOffset(gyroscope[2]);
  accelgyro.setXAccelOffset(accelerometer[0]);
  accelgyro.setYAccelOffset(accelerometer[1]);
  accelgyro.setZAccelOffset(accelerometer[2]);
  file.close();
}

void calibration_update_file() {
  File file = LittleFS.open(JSON_NAME, "w");
  DynamicJsonDocument doc(SERIALISATION_CAPACITY);
  JsonArray accelerometer = doc.createNestedArray("accelerometer");
  JsonArray gyroscope = doc.createNestedArray("gyroscope");
  String json_in_string = "";
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  Serial.println("\nReading sensors for first time...");
  leds[0] = CRGB(255, 0, 255);
  FastLED.show();
  delay(1000);
  meansensors();
  delay(1000);
  Serial.println("\nCalculating offsets...");
  calibration();
  Serial.println("End of calibration");
  delay(1000);
  meansensors();
  if (is_in_interval(mean_ax, -100, 100, true) && is_in_interval(mean_ay, -100, 100, true) &&
      is_in_interval(mean_az, 16284, 16484, true) && is_in_interval(mean_gx, -100, 100, true) &&
      is_in_interval(mean_gy, -100, 100, true) && is_in_interval(mean_gz, -100, 100, true)) {
        doc["calibration"] = true;
        accelerometer.add(ax_offset);
        accelerometer.add(ay_offset);
        accelerometer.add(az_offset);
        gyroscope.add(gx_offset);
        gyroscope.add(gy_offset);
        gyroscope.add(gz_offset);
  } else {
    Serial.println("\n\n\n//////////////////JUST BEFORE RESTART\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n\n\n");
    ESP.restart();
  }
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  Serial.println("\n\n\n//////////////////JUST BEFORE SERIALIZEJSON\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n\n\n");
  serializeJson(doc, json_in_string);
  Serial.println("\n\n\n//////////////////JUST AFTER SERIALIZEJSON\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n\n\n");
  Serial.print(json_in_string);
  file.print(json_in_string);
  Serial.println("\n\n\n//////////////////JUST AFTER PRINTING IN FILE\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n\n\n");
  file.close();
}

void create_file() {
  File file = LittleFS.open(JSON_NAME, "w");
  file.print("{\"calibration\":false,\"accelerometer\":[0,0,0],\"gyroscope\":[0,0,0]}");
  Serial.println("in create file");
  file.close();
}

void test_file() {
  DynamicJsonDocument doc(DESERIALISATION_CAPACITY);
  File file = LittleFS.open(JSON_NAME, "r");
  bool calibration;
  if (deserializeJson(doc, file)) {
    Serial.println("deserialisation error");
    file.close();
    create_file();
    calibration_update_file();
  } else {
    calibration = doc["calibration"];
    Serial.println(calibration);
    if (!calibration) {
      file.close();
      calibration_update_file();
    } else {
      file.close();
    }
  }
}

void setup() {
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

    LittleFS.begin();
    print_file();
    File file;
    file = LittleFS.open(JSON_NAME, "r");
    if (!file){
      file.close();
      create_file();
    } else {
      file.close();
    }
    test_file();
    read_file();
    LittleFS.end();

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
    /*
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
    */
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