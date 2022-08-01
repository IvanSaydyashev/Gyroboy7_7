#define PID_OPTIMIZED_I

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MeOrion.h"
#include <math.h>
#include "GyverTimer.h"
#include "GyverPID.h"

#define BALANCING_LIMITS 40 // Границы балансая
#define BLACK_LEFT_LINE_S 480 // Сырые значения с датчика линии
#define WHITE_LEFT_LINE_S 45
#define BLACK_RIGHT_LINE_S 480
#define WHITE_RIGHT_LINE_S 45
#define MAX_VAL_POT 976
#define N_MEASURE_SPEED 15
#define N_MEASURE_ULTRASONIC 15

MeGyro gyroAccel;
MeUltrasonicSensor ultrasonic(3);
MePort lineSensors(PORT_7);
Me7SegmentDisplay seg7(4);
MeStepper stepperL(PORT_1);
MeStepper stepperR(PORT_2); 

float angleY = 0;
float vKp = 0, vKi = 0, vKd = 0;
float bKp = 50, bKi = 0.00004, bKd = 3;
float lKp = 0, lKi = 0, lKd = 0;

unsigned long currTime, prevTime, loopTime;
float balanceSetPoint = -1;
int targetSpeed = 0;
int u_left, u_right;
float u_speed, u_balance, u_lineFollower;
int currentUltasonicDist;
float angleFixrate = 0.0008;

GTimer_ms myTimer1(20), myTimer2(100);
GyverPID regulator_b(bKp, bKi, bKd, 10);
GyverPID regulator_v(vKp, vKi, vKd, 10);
GyverPID regulator_l(lKp, lKi, lKd, 10);

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initialization...");
  gyroAccel.begin();
  stepperL.setAcceleration(200000); stepperR.setAcceleration(200000);
  stepperL.setMaxSpeed(1000); stepperR.setMaxSpeed(1000);
  regulator_b.setDirection(REVERSE);
  regulator_b.setLimits(-255, 255);
  Serial.println("Initialization completed");
  buzzerOff();
}

void loop()
{
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  //SerialData();
  if (myTimer1.isReady()) {
    gyroAccel.update();
    angleY = gyroAccel.getAngleY();
    if (myTimer2.isReady()) seg7.display(angleY);
    if (abs(angleY) < BALANCING_LIMITS) {
      if (angleY < balanceSetPoint) {
        balanceSetPoint += angleFixrate * loopTime;
      } else {
        balanceSetPoint -= angleFixrate * loopTime;
      }
      if (balanceSetPoint-0.5<=angleY<=balanceSetPoint+0.5){
        regulator_b.integral = 0;
      }
      // Speed
      int lMotSpeed = u_left*2, rMotSpeed = u_right*2;
      //Serial.print("lMotSpeed: "); Serial.print(lMotSpeed); Serial.print(" "); Serial.print("rMotSpeed: "); Serial.print(rMotSpeed); Serial.print("\t");
      int currentSpeed = (lMotSpeed + rMotSpeed) / 2;
      //Serial.print("speed: "); Serial.print(currentSpeed); Serial.print("\t");

      // Speed PID
      //float errorSpeed = currentSpeed - targetSpeed;
      //regulator_v.setpoint = errorSpeed;
      //u_speed = regulator_v.getResultTimer();
      int u_speed = 0;

      // Balance PID
      float targetAngle = balanceSetPoint - u_speed;
      float errorAngle = angleY - targetAngle;
      regulator_b.setpoint = errorAngle;
      u_balance = regulator_b.getResultTimer();

      // Line Sensors
      //int leftLineS = lineSensors.aRead1(); //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
      //int rightLineS = lineSensors.aRead2(); //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
      //leftLineS = map(leftLineS, BLACK_LEFT_LINE_S, WHITE_LEFT_LINE_S, 0, 255);
      //leftLineS = constrain(leftLineS, 0, 255); //Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
      //rightLineS = map(rightLineS, BLACK_RIGHT_LINE_S, WHITE_RIGHT_LINE_S, 0, 255);
      //rightLineS = constrain(rightLineS, 0, 255); //Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t");

      // LineFollower PID
      //float errorLineFollower = leftLineS - rightLineS;
      //u_lineFollower = PID_Control(2, errorLineFollower, 0.05, 0, 0, loopTime, false);
      //u_lineFollower = constrain(u_lineFollower, -25, 25);
      
      // Ultrasonic distance
      //currentUltasonicDist = ultrasonic.distanceCm();

      u_left = -u_balance + u_lineFollower; u_right = u_balance - u_lineFollower;
      u_left = constrain(u_left, -255, 255); u_right = constrain(u_right, -255, 255);
      Serial.print(u_left); Serial.print("  "); Serial.println(u_right);
      stepperL.setSpeed(u_left*2); stepperR.setSpeed(u_right*2);
    }
    else {
      stepperL.setSpeed(0); stepperR.setSpeed(0);
    }
  }
  stepperL.runSpeed(); stepperR.runSpeed();
}

void SerialData(){
  if(Serial.available() > 2) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = command.length(); // Переменая для хронения индекса вхождения цифры в входной строке
    // Поиск первого вхождения цифры от 0 по 9 в подстроку
    for (byte i = 0; i < 10; i++) {
      byte index = command.indexOf(String(i));
      if (index < strIndex && index != 255) strIndex = index;
    }
    String incoming = command.substring(0, strIndex);
    String valueStr = command.substring(strIndex, command.length());
    float value = valueStr.toFloat();
    if (incoming == "bp") {
      bKp = value;
      Serial.print("bp  ");
      Serial.println(value);
      regulator_b.Kp = bKp;
    } else if (incoming == "bi") {
      bKi = value;
      Serial.print("bi  ");
      Serial.println(value);
      regulator_b.Ki = bKi;
      regulator_b.integral = 0;
    } else if (incoming == "bd") {
      bKd = value;
      Serial.print("bd  ");
      Serial.println(value);
      regulator_b.Kd = bKd;
    } else if (incoming == "bsp") {
      balanceSetPoint = value;
      Serial.print("bsp  ");
      Serial.println(value);
      regulator_b.integral = 0;
    } else if (incoming == "af") {
      angleFixrate = value;
      Serial.print("af  ");
      Serial.println(value);
    }
  }
}
