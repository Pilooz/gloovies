/*

 Matériel :
  - MPU 6050c (Gy521 breakout). Alimenter de prérérence en 5v pour éviter le
 bruit sur le bus I2C Ressources  :
  - https://playground.arduino.cc/Main/MPU-6050/


#include <Arduino.h>
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include <Wire.h>
#include "structures.h"

const int MPU_addr = 0x68; // I2C address of the MPU-6050
// int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
mpu6050_struct mesures;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  mesures.AcX = Wire.read() << 8 |
        Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  mesures.AcY = Wire.read() << 8 |
        Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  mesures.AcZ = Wire.read() << 8 |
        Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  mesures.Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  mesures.GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  mesures.GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  mesures.GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = ");
  Serial.print(mesures.AcX);
  Serial.print(" | AcY = ");
  Serial.print(mesures.AcY);
  Serial.print(" | AcZ = ");
  Serial.print(mesures.AcZ);
  Serial.print(" | Tmp = ");
  Serial.print(mesures.Tmp / 340.00 +
               36.53); // equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = ");
  Serial.print(mesures.GyX);
  Serial.print(" | GyY = ");
  Serial.print(mesures.GyY);
  Serial.print(" | GyZ = ");
  Serial.println(mesures.GyZ);
  delay(333);
}

*/

/*

 Matériel :
  - MPU 6050c (Gy521 breakout). Alimenter de prérérence en 5v pour éviter le
 bruit sur le bus I2C Ressources  :
  - https://playground.arduino.cc/Main/MPU-6050/

*/
#include <Arduino.h>
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include <Arduino.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Wire.h"
#include "MPU6050.h"
#include "RunningAverage.h"
#include "FastLED.h"

#define NUM_LEDS 1
#define DATA_PIN D7
#define LED_TYPE WS2812
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

int nb_vals = 50; // for running averages

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 accelgyro;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
MPU6050 accelgyro(MPU_addr); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

RunningAverage val_ax(nb_vals);
RunningAverage val_ay(nb_vals);
RunningAverage val_az(nb_vals);
RunningAverage val_gx(nb_vals);
RunningAverage val_gy(nb_vals);
RunningAverage val_gz(nb_vals);

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

long current_millis;
long blinker_millis;
long reader_millis;
bool turning;
bool blinker_state;

// #define LED_PIN 13
// bool blinkState = false;

void blinker() {
  if (current_millis - blinker_millis >= 500) {
    if (!blinker_state) {
      leds[0] = CRGB(255, 165, 0);
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

void setup() {
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    //while (!accelgyro.testConnection());
    
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values

    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    // Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    // Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    // Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    // Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    // Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    // Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    // Serial.print("\n");

    accelgyro.setXGyroOffset(1000);
    accelgyro.setYGyroOffset(1000);
    accelgyro.setZGyroOffset(1000);
    accelgyro.setXAccelOffset(1000);
    accelgyro.setYAccelOffset(1000);
    accelgyro.setZAccelOffset(1000);

    // Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    // Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    // Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    // Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    // Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    // Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    // Serial.print("\n");


    // configure Arduino LED pin for output
    // pinMode(LED_PIN, OUTPUT);
    val_ax.fillValue(0, nb_vals);
    val_ay.fillValue(0, nb_vals);
    val_az.fillValue(0, nb_vals);
    val_gx.fillValue(0, nb_vals);
    val_gy.fillValue(0, nb_vals);
    val_gz.fillValue(0, nb_vals);

    current_millis = 0;
    blinker_millis = 0;
    reader_millis = 0;
    turning = false;
    blinker_state = false;

    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    leds[0] = CRGB(255,165,0);
    FastLED.show();
    delay(5000);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
}

void loop() {
  current_millis = millis();
    // read raw accel/gyro measurements from device
  if (current_millis - reader_millis >= 100) {
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
    Serial.print(val_ax.getAverage()); Serial.print("   ");
    Serial.print(val_ay.getAverage()); Serial.print("   ");
    Serial.print(val_az.getAverage()); Serial.print("   ");
    Serial.print("          ");
    Serial.print(val_gx.getAverage()); Serial.print("   ");
    Serial.print(val_gy.getAverage()); Serial.print("   ");
    Serial.print(val_gz.getAverage()); Serial.print("   ");
    Serial.println("");
  }

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
      if ((val_gz.getAverage() > 8000) && (val_gx.getAverage() < 2000)) {
        turning = false;
      }
      if ((val_gz.getAverage() < 2000) && (val_gx.getAverage() > 8000)) {
        turning = true;
      }
      if (turning) {
        blinker();
        //Serial.print("TURNING");
      } else {
        blinker_state = false;
        leds[0] = CRGB(0, 0, 0);
        FastLED.show();
      }
        /*
        Serial.print(val_gx.getMin()); Serial.print("\t");
        Serial.print(val_gx.getAverage()); Serial.print("\t");
        Serial.print(val_gx.getMax()); Serial.print("\t");
        */
        // Serial.print(val_gy.getAverage()); Serial.print("\t");
        // Serial.print(val_gz.getAverage()); Serial.print("\t");
        /*
        Serial.print("x value:\t");
        if (val_gx.getAverage() < 1000) {
          Serial.print(1); Serial.print("\t");
        } else if (val_gx.getAverage() > 10000) {
          Serial.print(-1); Serial.print("\t");
        } else {
          Serial.print(0); Serial.print("\t");
        }
        Serial.print("\ny value:\t");
        if (val_gy.getAverage() < 1000) {
          Serial.print(1); Serial.print("\t");
        } else if (val_gy.getAverage() > 10000) {
          Serial.print(-1); Serial.print("\t");
        } else {
          Serial.print(0); Serial.print("\t");
        }
        Serial.print("\nz value:\t");
        if (val_gz.getAverage() < 1000) {
          Serial.print(1); Serial.print("\t");
        } else if (val_gz.getAverage() > 10000) {
          Serial.print(-1); Serial.print("\t");
        } else {
          Serial.print(0); Serial.print("\t");
        }
        */
        // Serial.print(val_ax.getMin()); Serial.print("\t");
        // Serial.print(val_ay.getMin()); Serial.print("\t");
        // Serial.print(val_az.getMin()); Serial.print("\t");
        // Serial.print(val_gx.getMin()); Serial.print("\t");
        // Serial.print(val_gy.getMin()); Serial.print("\t");
        // Serial.print(val_gz.getMin()); Serial.print("\t");

        // Serial.print(val_ax.getMax()); Serial.print("\t");
        // Serial.print(val_ay.getMax()); Serial.print("\t");
        // Serial.print(val_az.getMax()); Serial.print("\t");
        // Serial.print(val_gx.getMax()); Serial.print("\t");
        // Serial.print(val_gy.getMax()); Serial.print("\t");
        // Serial.print(val_gz.getMax()); Serial.print("\t");

        // Serial.print(gx); Serial.print("\t");
        // Serial.print(gy); Serial.print("\t");
        // Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
}