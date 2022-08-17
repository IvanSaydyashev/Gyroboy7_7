#include <Servo.h>

#define MOTPIN 3

Servo mot;

int value = 0;
int pos = 0, new_pos = 0;
void setup() {
  mot.attach(3);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.replace(" ", "");
    byte strIndex = command.length();
    for (byte i = 0; i < 10; i++) {
      byte index = command.indexOf(String(i));
      if (index < strIndex && index != 255) strIndex = index;
    }
    String incoming = command.substring(0, strIndex);
    String valueStr = command.substring(strIndex, command.length());
    value = valueStr.toInt();
    if (incoming == "-") {
      value = map(value, 0, 100, 0, -100);
    }
    Serial.print(value); Serial.print(" ");
  }
  mot.write(map(value, -100, 100, 50, 130));
  delay(40);
}
