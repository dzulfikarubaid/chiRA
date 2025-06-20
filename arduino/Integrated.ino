#include <Servo.h>

Servo servoEE;
Servo servo1;
Servo servo2;
Servo servo3;

const int NUMBER_OF_STABLE_READINGS = 6;
const int ANOMALY_THRESHOLD_CM = 2;
const int CONSECUTIVE_ANOMALY_READINGS = 3;

int trigPin = 13;
int echoPin = 12;

long ultrasonicDuration, ultrasonicCm;
float steadyStateCm = 0;
int anomalyCounter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  servoEE.attach(9);
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(6);
  servoEE.write(0);
  servo1.write(90);
  servo2.write(90);
  servo3.write(180);
  Serial.println("Servos initialized: EE=0, S1=90, S2=90, S3=180");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Ultrasonic sensor calibration...");
  long totalCm = 0;
  for (int i = 0; i < NUMBER_OF_STABLE_READINGS; i++) {
    totalCm += readUltrasonicCm();
    delay(100);
  }
  steadyStateCm = (float)totalCm / NUMBER_OF_STABLE_READINGS;
  Serial.print("Ultrasonic steady state: ");
  Serial.print(steadyStateCm);
  Serial.println(" cm");
  Serial.println("---------------------------------------------");
}

void loop() {
  ultrasonicCm = readUltrasonicCm();
  Serial.print("Ultrasonic Distance: ");
  Serial.print(ultrasonicCm);
  Serial.println(" cm");

  if (ultrasonicCm < (steadyStateCm - ANOMALY_THRESHOLD_CM) && ultrasonicCm > 0) {
    anomalyCounter++;
  } else {
    anomalyCounter = 0;
  }

  if (anomalyCounter >= CONSECUTIVE_ANOMALY_READINGS) {
    Serial.println("!!! DISTANCE ANOMALY DETECTED !!!");
    Serial.println("<FEEDBACK:ANOMALY_JARAK>");
    anomalyCounter = 0;
  }
  delay(200);

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
        moveServosSimultaneously(0, 90, 90, 182, 200, 1000, 1000, 1000);
        Serial.println("Servos reset.");
      }
    }
  }
}

long readUltrasonicCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  ultrasonicDuration = pulseIn(echoPin, HIGH);
  return (ultrasonicDuration / 2) / 29.1;
}

void moveServosSimultaneously(float targetEE, float target1, float target2, float target3, int moveTimeEE, int moveTime1, int moveTime2, int moveTime3) {
  float currentEE = servoEE.read();
  float current1 = servo1.read();
  float current2 = servo2.read();
  float current3 = servo3.read();

  int maxSteps = max(max(moveTimeEE, moveTime1), max(moveTime2, moveTime3)) / 10;
  float stepEE = moveTimeEE > 0 ? (targetEE - currentEE) / (moveTimeEE / 10.0) : (targetEE - currentEE);
  float step1 = moveTime1 > 0 ? (target1 - current1) / (moveTime1 / 10.0) : (target1 - current1);
  float step2 = moveTime2 > 0 ? (target2 - current2) / (moveTime2 / 10.0) : (target2 - current2);
  float step3 = moveTime3 > 0 ? (target3 - current3) / (moveTime3 / 10.0) : (target3 - current3);

  for (int i = 0; i < maxSteps; i++) {
    if (i < moveTimeEE / 10) currentEE += stepEE;
    if (i < moveTime1 / 10) current1 += step1;
    if (i < moveTime2 / 10) current2 += step2;
    if (i < moveTime3 / 10) current3 += step3;
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