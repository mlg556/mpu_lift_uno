#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "PIDController.cpp"

#define PIN_PWM 3  // ADC0

#define FILTER 4

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;

// PID Controller
float kp0 = 15.0;
float ki0 = 0.2;
float kd0 = 0.0;

float ff = 50;

float setp = 0;

const float out_min = 0;
const float out_max = 255;

PIDController controller(kp0, ki0, kd0, ff, out_min, out_max);

void setup() {
  Serial.begin(115200);

  Serial.println("MPU6050 init...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip !!!");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
}

void loop() {

  float pitch = 0;
  sensors_event_t accel;

  // read FILTER times and average result
  for(int i = 0; i < FILTER; i++) {
    bool get_accel = mpu_accel->getEvent(&accel);

    if (!get_accel)
      Serial.println("ERR");

    pitch += accel.acceleration.pitch;
  }
  pitch = pitch / FILTER;

  // error calc
  float err = pitch - setp;

  // pid calc
  float out = controller.compute(err);

  // convert output to integer
  int out_int = int(round(out));

  analogWrite(PIN_PWM, out_int);

  //delay(1);

  // print every 5ms (50ms?)
  if(millis() % 5 == 0) {
    // Serial.printf("%f\t%d\t\n", pitch, out_int);
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(out_int);
  }


}
