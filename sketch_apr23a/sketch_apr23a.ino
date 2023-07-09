
#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;

unsigned long previousMillis = 0;
const int interval = 3;
const float kP = 0.2;

String serData;

int ind1, ind2, ind3, ind4, ind5, ind6;
float pos1, pos2, pos3, pos4, pos5, pos6;
float pos1_goal, pos2_goal, pos3_goal, pos4_goal, pos5_goal, pos6_goal;
float error1, error2, error3, error4, error5, error6;

int pos = 90;


void setup() {

  Serial.begin(57600);
  Serial.setTimeout(10);
  Serial.println("Begin");


  myservo1.attach(10, 500, 2500);  // attaches the servo on pin 10 to the servo object
  myservo2.attach(3, 500, 2500);
  myservo3.attach(6, 500, 2500);
  myservo4.attach(9, 500, 2500);
  myservo5.attach(5, 500, 2500);
  myservo6.attach(11, 500, 2500);

  myservo1.write(pos);
  myservo2.write(pos);
  myservo3.write(pos);
  myservo4.write(pos);
  myservo5.write(pos);
  myservo6.write(pos);

  pos1_goal = pos;
  pos2_goal = pos;
  pos3_goal = pos;
  pos4_goal = pos;
  pos5_goal = pos;
  pos6_goal = pos;

  pos1 = pos;
  pos2 = pos;
  pos3 = pos;
  pos4 = pos;
  pos5 = pos;
  pos6 = pos;

}

void loop() {

  if (Serial.available() > 0) {
    serData = Serial.readString();

    if (serData.indexOf('a') != -1 && serData.indexOf('b') != -1 && serData.indexOf('c') != -1 && serData.indexOf('d') != -1 && serData.indexOf('e') != -1) {

      ind1 = serData.indexOf('a');
      ind2 = serData.indexOf('b');
      ind3 = serData.indexOf('c');
      ind4 = serData.indexOf('d');
      ind5 = serData.indexOf('e');

      pos1_goal = (serData.substring(0, ind1)).toFloat();
      pos2_goal = (serData.substring(ind1 + 1, ind2)).toFloat();
      pos3_goal = (serData.substring(ind2 + 1, ind3)).toFloat();
      pos4_goal = (serData.substring(ind3 + 1, ind4)).toFloat();
      pos5_goal = (serData.substring(ind4 + 1, ind5)).toFloat();
      pos6_goal = (serData.substring(ind5 + 1)).toFloat();
    }


  }

  unsigned long currentMillis = millis();  // get the current time

  if (currentMillis - previousMillis >= interval) {  // check if it's time to update the servos
    previousMillis = currentMillis;  // save the current time
    
    error1 = pos1_goal - pos1;
    error2 = pos2_goal - pos2;
    error3 = pos3_goal - pos3;
    error4 = pos4_goal - pos4;
    error5 = pos5_goal - pos5;
    error6 = pos6_goal - pos6;

    pos1 += error1 * kP;
    pos2 += error2 * kP;
    pos3 += error3 * kP;
    pos4 += error4 * kP;
    pos5 += error5 * kP;
    pos6 += error6 * kP;

    myservo1.writeMicroseconds(map(pos1, 0, 180, 500, 2500));
    myservo2.writeMicroseconds(map(pos2, 0, 180, 500, 2500));
    myservo3.writeMicroseconds(map(pos3, 0, 180, 500, 2500));
    myservo4.writeMicroseconds(map(pos4, 0, 180, 500, 2500));
    myservo5.writeMicroseconds(map(pos5, 0, 180, 500, 2500));
    myservo6.writeMicroseconds(map(pos6, 0, 180, 500, 2500));
  }
}
