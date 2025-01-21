#include <ESP32Servo.h>
#include <Arduino.h>
 
Servo my_servo_one;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
Servo my_servo_two;

Servo my_servo_three;

Servo my_servo_four;

Servo my_servo_five;
 
bool considr_rbt_arm_numbr = 0;
 
int waist_angle = 0;
int sholdr_angle = 0;
int elbow_angle = 0;
int wrst_angle = 0;
int gripr_angle = 0;
 
bool waist_reachd = 0;
bool sholdr_reachd = 0;
bool elbow_reachd = 0;
bool wrst_reachd = 0;
bool gripr_reachd = 0;
 
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 7;
#else
int servo_pin_one = 12;
int servo_pin_two = 13;
int servo_pin_three = 14;
int servo_pin_four = 26;
int servo_pin_five = 27;
#endif
 
int waist_angle_old = 0;
int sholdr_angle_old = 0;
int elbow_angle_old = 0;
int wrst_angle_old = 0;
int gripr_angle_old = 0;
 
 
 
void setup()
{
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  my_servo_one.setPeriodHertz(50);    // standard 50 hz servo
  my_servo_one.attach(servo_pin_one, 1000, 2000); // attaches the servo on pin 18 to the servo object
 
  my_servo_two.setPeriodHertz(50);
  my_servo_two.attach(servo_pin_two, 1000,2000);

  my_servo_three.setPeriodHertz(50);
  my_servo_three.attach(servo_pin_three,1000,2000);

  my_servo_four.setPeriodHertz(50);
  my_servo_four.attach(servo_pin_four,500,2400);

  my_servo_five.setPeriodHertz(50);
  my_servo_five.attach(servo_pin_five,500,2400);

  Serial.begin(115200);
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}
 
void loop()
{
 
   if(Serial.available())
{
      String s =  Serial.readString();
 
      for(auto d:s) //dis code which includes seeing if c is dere is now working
      {
 
        if(d == 'a')
        {
          considr_rbt_arm_numbr = 1;
          waist_angle = atoi(s.substring(1,4).c_str());
          sholdr_angle = atoi(s.substring(4,7).c_str());
          elbow_angle = atoi(s.substring(7,10).c_str());
          wrst_angle = atoi(s.substring(10,13).c_str());
          gripr_angle = atoi(s.substring(13,16).c_str());
          waist_reachd = 0;
          sholdr_reachd = 0;
          elbow_reachd = 0;
          wrst_reachd = 0;
          gripr_reachd = 0;
          break;
        }
        else
        {
          considr_rbt_arm_numbr = 0;
        }
      }
}
delay(2);
Serial.print("howdy earlier is ");
Serial.print(considr_rbt_arm_numbr);
 
  if(considr_rbt_arm_numbr == 1)
  {
     if(waist_angle_old < waist_angle)
     {
        for (int pos = waist_angle_old; pos <= waist_angle; pos += 1)
        { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          my_servo_one.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        waist_angle_old = waist_angle;
        waist_reachd = 1;
     }
     if(waist_angle_old > waist_angle)
     {
        for (int pos = waist_angle_old; pos >= waist_angle; pos -= 1)
        { // goes from 180 degrees to 0 degrees
          my_servo_one.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        waist_angle_old = waist_angle;
        waist_reachd = 1;
     }
     else
     {
      waist_reachd = 1;
     }
  }
 
    if(considr_rbt_arm_numbr == 1)
    {
     if(sholdr_angle_old < sholdr_angle)
     {
        for (int pos = sholdr_angle_old; pos <= sholdr_angle; pos += 1)
        { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          my_servo_two.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        sholdr_angle_old = sholdr_angle;
        sholdr_reachd = 1;
     }
     if(sholdr_angle_old > sholdr_angle)
     {
        for (int pos = sholdr_angle_old; pos >= sholdr_angle; pos -= 1)
        { // goes from 180 degrees to 0 degrees
          my_servo_two.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        sholdr_angle_old = sholdr_angle;
        sholdr_reachd = 1;
     }
     else
     {
      sholdr_reachd = 1;
     }
  }

    if(considr_rbt_arm_numbr == 1)
    {
     if(elbow_angle_old < elbow_angle)
     {
        for (int pos = elbow_angle_old; pos <= elbow_angle; pos += 1)
        { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          my_servo_three.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        elbow_angle_old = elbow_angle;
        elbow_reachd = 1;
     }
     if(elbow_angle_old > elbow_angle)
     {
        for (int pos = elbow_angle_old; pos >= elbow_angle; pos -= 1)
        { // goes from 180 degrees to 0 degrees
          my_servo_three.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        elbow_angle_old = elbow_angle;
        elbow_reachd = 1;
     }
     else
     {
      elbow_reachd = 1;
     }
 
  }

    if(considr_rbt_arm_numbr == 1)
    {
     if(wrst_angle_old < wrst_angle)
     {
        for (int pos = wrst_angle_old; pos <= wrst_angle; pos += 1)
        { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          my_servo_four.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        wrst_angle_old = wrst_angle;
        wrst_reachd = 1;
     }
     if(wrst_angle_old > wrst_angle)
     {
        for (int pos = wrst_angle_old; pos >= wrst_angle; pos -= 1)
        { // goes from 180 degrees to 0 degrees
          my_servo_four.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        wrst_angle_old = wrst_angle;
        wrst_reachd = 1;
     }
     else
     {
      wrst_reachd = 1;
     }
 
  }

      if(considr_rbt_arm_numbr == 1)
    {
     if(gripr_angle_old < gripr_angle)
     {
        for (int pos = gripr_angle_old; pos <= gripr_angle; pos += 1)
        { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          my_servo_five.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        gripr_angle_old = gripr_angle;
        gripr_reachd = 1;
     }
     if(gripr_angle_old > gripr_angle)
     {
        for (int pos = gripr_angle_old; pos >= gripr_angle; pos -= 1)
        { // goes from 180 degrees to 0 degrees
          my_servo_five.write(pos);    // tell servo to go to position in variable 'pos'
          delay(15);             // waits 15ms for the servo to reach the position
        }
        gripr_angle_old = gripr_angle;
        gripr_reachd = 1;
     }
     else
     {
      gripr_reachd = 1;
     }
 
  }

  if(waist_reachd == 1 && sholdr_reachd == 1 && elbow_reachd == 1 && wrst_reachd == 1 && gripr_reachd == 1)
  {
    considr_rbt_arm_numbr = 0;
  }
 
Serial.print("waist is ");
Serial.print(waist_angle_old); 
Serial.print("shoulder is ");
Serial.print(sholdr_angle_old); 
Serial.print("elbow is ");
Serial.print(elbow_angle_old);
Serial.print("shouldr is ");
Serial.print(sholdr_angle_old);
Serial.print(" waist is ");
Serial.print(waist_angle_old);
Serial.print(" howdy now is ");
Serial.println(considr_rbt_arm_numbr);
 
}