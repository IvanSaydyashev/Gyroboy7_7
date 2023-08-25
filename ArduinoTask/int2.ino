#include <Wire.h>
#include "MS5837.h"
#include <Servo.h>

#define HALL_SENSOR 10
#define MOTPIN 3
#define RED_PIN 8
#define GREEN_PIN 6
#define BLUE_PIN 9
float depth_ = 0;
float power = 1480;
float depth_set = 0.2;
bool hall_val = false;
bool isStart = false;
bool isUp = true;
float time_, prev_time;
int k = 0;
MS5837 sensor;
Servo mot;

void setup() {
  pinMode(HALL_SENSOR, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("Starting");
  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);
  while (true) {
    hall_val = digitalRead(HALL_SENSOR);
    if (hall_val == 0) {

      Serial.println("WAIT FOR MAGNET");
      while (true) {

        set_color('0');
        hall_val = digitalRead(HALL_SENSOR);
        if (hall_val == 1) {
          Serial.println("FINISH HALL INIT");
          break;
        }
      }
      break;
    }
  }
  while (true) {
    hall_val = digitalRead(HALL_SENSOR);
    if (hall_val == 0) {
      Serial.println("INIT MOTOR");
      mot.attach(3);
      motor(1480);
      delay(8000);
      break;
    }
  }

}

void loop() {
  time_ = millis();
  sensor.read();
  depth_ = sensor.depth();
  Serial.print("Depth: ");
  Serial.print(depth_);
  Serial.println("m");
  hall_val = digitalRead(HALL_SENSOR);
  if (!hall_val) {
    while (isUp) {
      Serial.print("UP  ");
      Serial.println(k);
      sensor.read();
      depth_ = sensor.depth();
      mot.writeMicroseconds(1900);
      if (depth_ >= 0.30 && depth_ <= 0.59) {
        set_color('b');
      }
      else if (depth_ >= 0.60 && depth_ <= 0.7) {
        set_color('y');
      }
      else if (depth_ >= 0.7) {
        set_color('r');
      }
      else {
        set_color('g');
      }
      if (depth_ <= 0.25) {
        isUp = false;
        break;
      }
      
    }
    if (depth_ <= 0.2) {
      power = 1300;
    }
    else if (depth_ >= 0.8) {
      k += 1;
      isUp = true;
    }
    if (depth_ >= 0.30 && depth_ <= 0.59) {
      set_color('b');
    }
    else if (depth_ >= 0.60 && depth_ <= 0.79) {
      set_color('y');
    }
    else if (depth_ >= 0.80) {
      set_color('r');
    }
    else {
      set_color('g');
    }
  }
  else {
    set_color('g');
    Serial.println(hall_val);
  }
  mot.writeMicroseconds(power);
  if (k == 50) {
    isUp = true;
    while (isUp) {
      set_color('g');
      Serial.print("UP  ");
      Serial.println(k);
      sensor.read();
      depth_ = sensor.depth() + 0.25;
      mot.writeMicroseconds(1900);
      if (depth_ < 0.20) {
        isUp = false;
        break;
      }
    }
    exit(0);
  }

}

void motor(int value) {
  mot.writeMicroseconds(value);
}

void set_color(char color)
{
  if (color == 'y') {
    analogWrite(GREEN_PIN, 249);
    digitalWrite(BLUE_PIN, 1);
    digitalWrite(RED_PIN, 0);
  }
  else if (color == 'r') {
    digitalWrite(RED_PIN, 0);
    digitalWrite(GREEN_PIN, 1);
    digitalWrite(BLUE_PIN, 1);
  }
  else if (color == 'b') {
    digitalWrite(RED_PIN, 1);
    digitalWrite(GREEN_PIN, 1);
    digitalWrite(BLUE_PIN, 0);
  }
  else if (color == '0') {
    digitalWrite(RED_PIN, 1);
    digitalWrite(GREEN_PIN, 1);
    digitalWrite(BLUE_PIN, 1);
  }
  else if ( color == 'g') {
    digitalWrite(RED_PIN, 1);
    digitalWrite(GREEN_PIN, 0);
    digitalWrite(BLUE_PIN, 1);
  }
}
