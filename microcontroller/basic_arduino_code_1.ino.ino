
#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;


String serData;

int ind1, ind2, ind3, ind4, ind5, ind6;
float pos1_goal, pos2_goal, pos3_goal, pos4_goal, pos5_goal, pos6_goal;

int pos = 90;



float degrees_to_pwm(float position) 
{
  // Ensure position is within the valid range (0 to 180)
  position = constrain(position, 0.0, 180.0);
  
  // Define the range mapping
  float position_min = 0.0;
  float position_max = 180.0;
  float pwm_min = 500.0;
  float pwm_max = 2500.0;
  
  // Perform linear mapping
  float pwm_value = ((position - position_min) / (position_max - position_min)) * (pwm_max - pwm_min) + pwm_min;
  
  return pwm_value;
}


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

    myservo1.writeMicroseconds(degrees_to_pwm(pos1_goal));
    myservo2.writeMicroseconds(degrees_to_pwm(pos2_goal));
    myservo3.writeMicroseconds(degrees_to_pwm(pos3_goal));
    myservo4.writeMicroseconds(degrees_to_pwm(pos4_goal));
    myservo5.writeMicroseconds(degrees_to_pwm(pos5_goal));
    myservo6.writeMicroseconds(degrees_to_pwm(pos6_goal));
 
}
