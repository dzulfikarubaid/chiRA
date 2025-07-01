#include <Servo.h>

Servo servoEE;
Servo servo1;
Servo servo2;
Servo servo3;

const int irSensorPin = 2;

volatile int objectCount = 0;
long lastDebounceTime = 0;
long debounceDelay = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  servoEE.attach(9);
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(6);
  servoEE.write(90);
  servo1.write(180);
  servo2.write(80);
  servo3.write(160);
  Serial.println("Servos initialized: EE=0, S1=180, S2=80, S3=160");

  pinMode(irSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irSensorPin), incrementObjectCount, FALLING); 
}

void loop() {
  delay(500);

  if (Serial.available() > 0) {
    String command = "";
    while (Serial.available()) {
      command = Serial.readStringUntil('>');
      while (Serial.available()) Serial.read();
    }
    if (command.startsWith("<")) {
      command = command.substring(1);
      Serial.println("Received Servo Command: " + command);
      
      int commaIndex[8];
      int index = 0;
      for (int i = 0; i < command.length() && index < 8; i++) {
        if (command.charAt(i) == ',') {
          commaIndex[index++] = i;
        }
      }
      String cmdType = command.substring(0, commaIndex[0]);

      if (cmdType == "MOVE_ROBOT_ARM") {
        float angleEE = command.substring(commaIndex[0] + 1, commaIndex[1]).toFloat();
        float angle1 = command.substring(commaIndex[1] + 1, commaIndex[2]).toFloat();
        float angle2 = command.substring(commaIndex[2] + 1, commaIndex[3]).toFloat();
        float angle3 = command.substring(commaIndex[3] + 1, commaIndex[4]).toFloat();
        int moveTimeEE = command.substring(commaIndex[4] + 1, commaIndex[5]).toInt();
        int moveTime1 = command.substring(commaIndex[5] + 1, commaIndex[6]).toInt();
        int moveTime2 = command.substring(commaIndex[6] + 1, commaIndex[7]).toInt();
        int moveTime3 = command.substring(commaIndex[7] + 1).toInt();
        
        Serial.println("Servo Parsed: EE=" + String(angleEE) + ", S1=" + String(angle1) + ", S2=" + String(angle2) + ", S3=" + String(angle3) + ", Times=" + String(moveTimeEE) + "," + String(moveTime1) + "," + String(moveTime2) + "," + String(moveTime3));

        angleEE = constrain(angleEE, 0, 90);
        angle1 = constrain(angle1, 0, 180);
        angle2 = constrain(angle2, 60, 165);
        angle3 = constrain(angle3 + 2.0, 125, 180);

        moveServosSimultaneously(angleEE, angle1, angle2, angle3, moveTimeEE, moveTime1, moveTime2, moveTime3);
        Serial.println("Servos moved.");
      } else if (cmdType == "RESET") {
        moveServosSimultaneously(0, 180, 80, 160, 200, 1000, 1000, 1000);
        Serial.println("Servos reset.");
      }
    }
  }
}

void incrementObjectCount() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    objectCount++;
    Serial.println("<PICKED>");
    lastDebounceTime = millis();
  }
}

void moveServosSimultaneously(float targetEE, float target1, float target2, float target3, int moveTimeEE, int moveTime1, int moveTime2, int moveTime3) {
  float currentEE = servoEE.read();
  float current1 = servo1.read();
  float current2 = servo2.read();
  float current3 = servo3.read();

  int maxSteps = max(max(moveTimeEE, moveTime1), max(moveTime2, moveTime3)) / 10; 
  if (maxSteps == 0) maxSteps = 1; 

  float stepEE = (maxSteps > 0) ? (targetEE - currentEE) / maxSteps : 0;
  float step1 = (maxSteps > 0) ? (target1 - current1) / maxSteps : 0;
  float step2 = (maxSteps > 0) ? (target2 - current2) / maxSteps : 0;
  float step3 = (maxSteps > 0) ? (target3 - current3) / maxSteps : 0;
  
  if (maxSteps == 1 && (moveTimeEE == 0 || moveTime1 == 0 || moveTime2 == 0 || moveTime3 == 0)) {
    servoEE.write(round(targetEE));
    servo1.write(round(target1));
    servo2.write(round(target2));
    servo3.write(round(target3));
    return;
  }

  for (int i = 0; i < maxSteps; i++) {
    if (moveTimeEE > 0) currentEE += stepEE;
    if (moveTime1 > 0) current1 += step1;
    if (moveTime2 > 0) current2 += step2;
    if (moveTime3 > 0) current3 += step3;
    
    servoEE.write(round(currentEE));
    servo1.write(round(current1));
    servo2.write(round(current2));
    servo3.write(round(current3));
    delay(10);
  }

  servoEE.write(round(targetEE));
  servo1.write(round(target1));
  servo2.write(round(target2));
  servo3.write(round(target3));
}