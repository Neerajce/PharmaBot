// #include <ESP32Servo.h>
// #include <Arduino.h>
 
// Servo my_servo_one;  // create servo object to control a servo
// // 16 servo objects can be created on the ESP32
// Servo my_servo_two;

// Servo my_servo_three;

// Servo my_servo_four;

// Servo my_servo_five;
 
// bool considr_rbt_arm_numbr = 0;
 
// int waist_angle = 0;
// int sholdr_angle = 0;
// int elbow_angle = 0;
// int wrst_angle = 0;
// int gripr_angle = 0;
 
// bool waist_reachd = 0;
// bool sholdr_reachd = 0;
// bool elbow_reachd = 0;
// bool wrst_reachd = 0;
// bool gripr_reachd = 0;
 
// // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// // Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// // Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// // Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
// #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
// int servoPin = 17;
// #elif defined(CONFIG_IDF_TARGET_ESP32C3)
// int servoPin = 7;
// #else
// int servo_pin_one = 12;
// int servo_pin_two = 13;
// int servo_pin_three = 14;
// int servo_pin_four = 26;
// int servo_pin_five = 27;
// #endif
 
// int waist_angle_old = 0;
// int sholdr_angle_old = 0;
// int elbow_angle_old = 0;
// int wrst_angle_old = 0;
// int gripr_angle_old = 0;
 
 
 
// void setup()
// {
//   // Allow allocation of all timers
//   ESP32PWM::allocateTimer(0);
//   ESP32PWM::allocateTimer(1);
//   ESP32PWM::allocateTimer(2);
//   ESP32PWM::allocateTimer(3);
  
//   my_servo_one.setPeriodHertz(50);    // standard 50 hz servo
//   my_servo_one.attach(servo_pin_one, 1000, 2000); // attaches the servo on pin 18 to the servo object
 
//   my_servo_two.setPeriodHertz(50);
//   my_servo_two.attach(servo_pin_two, 1000,2000);

//   my_servo_three.setPeriodHertz(50);
//   my_servo_three.attach(servo_pin_three,1000,2000);

//   my_servo_four.setPeriodHertz(50);
//   my_servo_four.attach(servo_pin_four,500,2400);

//   my_servo_five.setPeriodHertz(50);
//   my_servo_five.attach(servo_pin_five,500,2400);

//   Serial.begin(115200);
//   // using default min/max of 1000us and 2000us
//   // different servos may require different min/max settings
//   // for an accurate 0 to 180 sweep
// }
 
// void loop()
// {
 
//    if(Serial.available())
// {
//       String s =  Serial.readString();
 
//       for(auto d:s) //dis code which includes seeing if c is dere is now working
//       {
 
//         if(d == 'a')
//         {
//           considr_rbt_arm_numbr = 1;
//           waist_angle = atoi(s.substring(1,4).c_str());
//           sholdr_angle = atoi(s.substring(4,7).c_str());
//           elbow_angle = atoi(s.substring(7,10).c_str());
//           wrst_angle = atoi(s.substring(10,13).c_str());
//           gripr_angle = atoi(s.substring(13,16).c_str());
//           waist_reachd = 0;
//           sholdr_reachd = 0;
//           elbow_reachd = 0;
//           wrst_reachd = 0;
//           gripr_reachd = 0;
//           break;
//         }
//         else
//         {
//           considr_rbt_arm_numbr = 0;
//         }
//       }
// }
// delay(2);
// Serial.print("howdy earlier is ");
// Serial.print(considr_rbt_arm_numbr);
 
//   if(considr_rbt_arm_numbr == 1)
//   {
//      if(waist_angle_old < waist_angle)
//      {
//         for (int pos = waist_angle_old; pos <= waist_angle; pos += 1)
//         { // goes from 0 degrees to 180 degrees
//           // in steps of 1 degree
//           my_servo_one.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         waist_angle_old = waist_angle;
//         waist_reachd = 1;
//      }
//      if(waist_angle_old > waist_angle)
//      {
//         for (int pos = waist_angle_old; pos >= waist_angle; pos -= 1)
//         { // goes from 180 degrees to 0 degrees
//           my_servo_one.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         waist_angle_old = waist_angle;
//         waist_reachd = 1;
//      }
//      else
//      {
//       waist_reachd = 1;
//      }
//   }
 
//     if(considr_rbt_arm_numbr == 1)
//     {
//      if(sholdr_angle_old < sholdr_angle)
//      {
//         for (int pos = sholdr_angle_old; pos <= sholdr_angle; pos += 1)
//         { // goes from 0 degrees to 180 degrees
//           // in steps of 1 degree
//           my_servo_two.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         sholdr_angle_old = sholdr_angle;
//         sholdr_reachd = 1;
//      }
//      if(sholdr_angle_old > sholdr_angle)
//      {
//         for (int pos = sholdr_angle_old; pos >= sholdr_angle; pos -= 1)
//         { // goes from 180 degrees to 0 degrees
//           my_servo_two.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         sholdr_angle_old = sholdr_angle;
//         sholdr_reachd = 1;
//      }
//      else
//      {
//       sholdr_reachd = 1;
//      }
//   }

//     if(considr_rbt_arm_numbr == 1)
//     {
//      if(elbow_angle_old < elbow_angle)
//      {
//         for (int pos = elbow_angle_old; pos <= elbow_angle; pos += 1)
//         { // goes from 0 degrees to 180 degrees
//           // in steps of 1 degree
//           my_servo_three.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         elbow_angle_old = elbow_angle;
//         elbow_reachd = 1;
//      }
//      if(elbow_angle_old > elbow_angle)
//      {
//         for (int pos = elbow_angle_old; pos >= elbow_angle; pos -= 1)
//         { // goes from 180 degrees to 0 degrees
//           my_servo_three.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         elbow_angle_old = elbow_angle;
//         elbow_reachd = 1;
//      }
//      else
//      {
//       elbow_reachd = 1;
//      }
 
//   }

//     if(considr_rbt_arm_numbr == 1)
//     {
//      if(wrst_angle_old < wrst_angle)
//      {
//         for (int pos = wrst_angle_old; pos <= wrst_angle; pos += 1)
//         { // goes from 0 degrees to 180 degrees
//           // in steps of 1 degree
//           my_servo_four.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         wrst_angle_old = wrst_angle;
//         wrst_reachd = 1;
//      }
//      if(wrst_angle_old > wrst_angle)
//      {
//         for (int pos = wrst_angle_old; pos >= wrst_angle; pos -= 1)
//         { // goes from 180 degrees to 0 degrees
//           my_servo_four.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         wrst_angle_old = wrst_angle;
//         wrst_reachd = 1;
//      }
//      else
//      {
//       wrst_reachd = 1;
//      }
 
//   }

//       if(considr_rbt_arm_numbr == 1)
//     {
//      if(gripr_angle_old < gripr_angle)
//      {
//         for (int pos = gripr_angle_old; pos <= gripr_angle; pos += 1)
//         { // goes from 0 degrees to 180 degrees
//           // in steps of 1 degree
//           my_servo_five.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         gripr_angle_old = gripr_angle;
//         gripr_reachd = 1;
//      }
//      if(gripr_angle_old > gripr_angle)
//      {
//         for (int pos = gripr_angle_old; pos >= gripr_angle; pos -= 1)
//         { // goes from 180 degrees to 0 degrees
//           my_servo_five.write(pos);    // tell servo to go to position in variable 'pos'
//           delay(15);             // waits 15ms for the servo to reach the position
//         }
//         gripr_angle_old = gripr_angle;
//         gripr_reachd = 1;
//      }
//      else
//      {
//       gripr_reachd = 1;
//      }
 
//   }

//   if(waist_reachd == 1 && sholdr_reachd == 1 && elbow_reachd == 1 && wrst_reachd == 1 && gripr_reachd == 1)
//   {
//     considr_rbt_arm_numbr = 0;
//   }
 
// Serial.print("waist is ");
// Serial.print(waist_angle_old); 
// Serial.print("shoulder is ");
// Serial.print(sholdr_angle_old); 
// Serial.print("elbow is ");
// Serial.print(elbow_angle_old);
// Serial.print("shouldr is ");
// Serial.print(sholdr_angle_old);
// Serial.print(" waist is ");
// Serial.print(waist_angle_old);
// Serial.print(" howdy now is ");
// Serial.println(considr_rbt_arm_numbr);
 
// }
// /*********************************************************************
//  *  ROSArduinoBridge
 
//     A set of simple serial commands to control a differential drive
//     robot and receive back sensor and odometry data. Default 
//     configuration assumes use of an Arduino Mega + Pololu motor
//     controller shield + Robogaia Mega Encoder shield.  Edit the
//     readEncoder() and setMotorSpeed() wrapper functions if using 
//     different motor controller or encoder method.

//     Created for the Pi Robot Project: http://www.pirobot.org
//     and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
//     Authors: Patrick Goebel, James Nugen

//     Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
//     Software License Agreement (BSD License)

//     Copyright (c) 2012, Patrick Goebel.
//     All rights reserved.

//     Redistribution and use in source and binary forms, with or without
//     modification, are permitted provided that the following conditions
//     are met:

//      * Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//      * Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials provided
//        with the distribution.

//     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *********************************************************************/

// #define USE_BASE      // Enable the base controller code
// //#undef USE_BASE     // Disable the base controller code

// /* Define the motor controller and encoder library you are using */
// #ifdef USE_BASE
//    /* The Pololu VNH5019 dual motor driver shield */
//    //#define POLOLU_VNH5019

//    /* The Pololu MC33926 dual motor driver shield */
//    //#define POLOLU_MC33926

//    /* The RoboGaia encoder shield */
//    //#define ROBOGAIA
   
//    /* Encoders directly attached to Arduino board */
//    #define ARDUINO_ENC_COUNTER

//    /* L298 Motor driver*/
//    #define L298_MOTOR_DRIVER
// #endif

// //#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
// #undef USE_SERVOS     // Disable use of PWM servos

// /* Serial port baud rate */
// #define BAUDRATE     115200

// /* Maximum PWM signal */
// #define MAX_PWM        255

// #define RIGHT_MOTOR_ENABLE_PIN 4
// #define LEFT_MOTOR_ENABLE_PIN 2

// #define LEFT_MOTOR_FORWARD_PIN 25
// #define LEFT_MOTOR_BACKWARD_PIN 15

// #define RIGHT_MOTOR_FORWARD_PIN 5
// #define RIGHT_MOTOR_BACKWARD_PIN 18



// // #if defined(ARDUINO) && ARDUINO >= 100
// // #include "Arduino.h"
// // #else
// // #include "WProgram.h"
// // #endif

// /* Include definition of serial commands */
// #include "commands.h"
// #include <Arduino.h>
// #include<ESP32Servo.h>
// #include<cstring>

// #include <cstddef>
// /* Include servo support if required */
//   #include "motor_driver.h"

//   /* Encoder driver function definitions */
// //   #include "encoder_driver.h"

//   /* PID parameters and functions */
//   #include "diff_controller.h"

//   /* Run the PID loop at 30 times per second */
//   #define PID_RATE           30     // Hz

//   /* Convert the rate into an interval */
//   const int PID_INTERVAL = 1000 / PID_RATE;
  
//   /* Track the next time we make a PID calculation */
//   unsigned long nextPID = PID_INTERVAL;

//   /* Stop the robot if it hasn't received a movement command
//    in this number of milliseconds */
//   #define AUTO_STOP_INTERVAL 2000
//   long lastMotorCommand = AUTO_STOP_INTERVAL;


// /* Variable initialization */


// // A pair of varibles to help parse serial commands (thanks Fergs)
// int arg = 0;
// int indx = 0;

// // Variable to hold an input character
// char chr;

// // Variable to hold the current single-character command
// char cmd;

// // Character arrays to hold the first and second arguments
// char argv1[16];
// char argv2[16];

// // The arguments converted to integers
// long arg1;
// long arg2;

// /* Clear the current command parameters */

//   void initMotorController() 
//   {
//   //  ledcAttachPin();
//     digitalWrite(RIGHT_MOTOR_ENABLE_PIN, HIGH);
//     digitalWrite(LEFT_MOTOR_ENABLE_PIN, HIGH);
//   }
  
//   void setMotorSpeed(int motor, int spd) 
//   {
//     unsigned char reverse = 0;
  
//     if (spd < 0)
//     {
//       spd = -spd;
//       reverse = 1;
//     }
//     if (spd > 255)
//       spd = 255;
    
//     if (motor == LEFT) 
//     { 
//       if(reverse == 0) 
//       { 
//         analogWrite(LEFT_MOTOR_FORWARD_PIN, spd); 
//         analogWrite(LEFT_MOTOR_BACKWARD_PIN, 0); 
//       }
//       else if(reverse == 1) 
//       { 
//         analogWrite(LEFT_MOTOR_BACKWARD_PIN, spd); 
//         analogWrite(LEFT_MOTOR_FORWARD_PIN, 0); 
//       }
//     }
//     else /*if (motor == RIGHT) //no need for condition*/ 
//     {
//       if(reverse == 0) 
//       { 
//         analogWrite(RIGHT_MOTOR_FORWARD_PIN, spd); 
//         analogWrite(RIGHT_MOTOR_BACKWARD_PIN, 0); 
//       }
//       else if(reverse == 1) 
//       { 
//         analogWrite(RIGHT_MOTOR_BACKWARD_PIN, spd); 
//         analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0); 
//       }
//     }
//   }
  
//   void setMotorSpeeds(int leftSpeed, int rightSpeed) 
//   {
//     setMotorSpeed(LEFT, leftSpeed);
//     setMotorSpeed(RIGHT, rightSpeed);
//   }

// void resetCommand() 
// {
//   cmd = NULL;
//   memset(argv1, 0, sizeof(argv1));
//   memset(argv2, 0, sizeof(argv2));
//   arg1 = 0;
//   arg2 = 0;
//   arg = 0;
//   indx = 0;
// }

// /* Run a command.  Commands are defined in commands.h */
// int runCommand() 
// {
//   int i = 0;
//   char *p = argv1;
//   char *str;
//   int pid_args[4];
//   arg1 = atoi(argv1);
//   arg2 = atoi(argv2);
  
//   switch(cmd) 
//   {
//   case GET_BAUDRATE:
//     Serial.println(BAUDRATE);
//     break;
//   case ANALOG_READ:
//     Serial.println(analogRead(arg1));
//     break;
//   case DIGITAL_READ:
//     Serial.println(digitalRead(arg1));
//     break;
//   case ANALOG_WRITE:
//     analogWrite(arg1, arg2);
//     Serial.println("OK"); 
//     break;
//   case DIGITAL_WRITE:
//     if (arg2 == 0) digitalWrite(arg1, LOW);
//     else if (arg2 == 1) digitalWrite(arg1, HIGH);
//     Serial.println("OK"); 
//     break;
//   case PIN_MODE:
//     if (arg2 == 0) pinMode(arg1, INPUT);
//     else if (arg2 == 1) pinMode(arg1, OUTPUT);
//     Serial.println("OK");
//     break;
// //   case PING:
// //     Serial.println(Ping(arg1));
// //     break;
// #ifdef USE_SERVOS
//   case SERVO_WRITE:
//     servos[arg1].setTargetPosition(arg2);
//     Serial.println("OK");
//     break;
//   case SERVO_READ:
//     Serial.println(servos[arg1].getServo().read());
//     break;
// #endif
    
// #ifdef USE_BASE
// //   case READ_ENCODERS:
// //     Serial.print(readEncoder(LEFT));
// //     Serial.print(" ");
// //     Serial.println(readEncoder(RIGHT));
// //     break;
// //    case RESET_ENCODERS:
// //     resetEncoders();
// //     resetPID();
// //     Serial.println("OK");
// //     break;
//   case MOTOR_SPEEDS:
//     /* Reset the auto stop timer */
//     lastMotorCommand = millis();
//     if (arg1 == 0 && arg2 == 0) 
//     {
//       setMotorSpeeds(0, 0);
//       resetPID();
//       moving = 0;
//     }
//     else moving = 1;
//     leftPID.TargetTicksPerFrame = arg1;
//     rightPID.TargetTicksPerFrame = arg2;
//     Serial.println("OK"); 
//     break;
//   case MOTOR_RAW_PWM:
//     /* Reset the auto stop timer */
//     lastMotorCommand = millis();
//     resetPID();
//     moving = 0; // Sneaky way to temporarily disable the PID
//     setMotorSpeeds(arg1, arg2);
//     Serial.println("OK"); 
//     break;
//   case UPDATE_PID:
//     while ((str = strtok_r(p, ":", &p)) != NULL) 
//     {
//        pid_args[i] = atoi(str);
//        i++;
//     }
//     Kp = pid_args[0];
//     Kd = pid_args[1];
//     Ki = pid_args[2];
//     Ko = pid_args[3];
//     Serial.println("OK");
//     break;
// #endif
//   default:
//     Serial.println("Invalid Command");
//     break;
//   }
// }

// /* Setup function--runs once at startup. */
// void setup() 
// {
//   Serial.begin(BAUDRATE);

// // Initialize the motor controller if used */

//   initMotorController();
//   resetPID();


// /* Attach servos if used */
//   #ifdef USE_SERVOS
//     int i;
//     for (i = 0; i < N_SERVOS; i++) {
//       servos[i].initServo(
//           servoPins[i],
//           stepDelay[i],
//           servoInitPosition[i]);
//     }
//   #endif
// }

// /* Enter the main loop.  Read and parse input from the serial port
//    and run any valid commands. Run a PID calculation at the target
//    interval and check for auto-stop conditions.
// */
// void loop() 
// {
//   while (Serial.available() > 0) 
//   {
    
//     // Read the next character
//     chr = Serial.read();

//     // Terminate a command with a CR
//     if (chr == 13) 
//     {
//       if (arg == 1) argv1[indx] = NULL;
//       else if (arg == 2) argv2[indx] = NULL;
//       runCommand();
//       resetCommand();
//     }
//     // Use spaces to delimit parts of the command
//     else if (chr == ' ') 
//     {
//       // Step through the arguments
//       if (arg == 0) 
//       {
//         arg = 1;
//       }
//       else if (arg == 1)  
//       {
//         argv1[indx] = NULL;
//         arg = 2;
//         indx = 0;
//       }
//       continue;
//     }
//     else 
//     {
//       if (arg == 0) 
//       {
//         // The first arg is the single-letter command
//         cmd = chr;
//       }
//       else if (arg == 1) 
//       {
//         // Subsequent arguments can be more than one character
//         argv1[indx] = chr;
//         indx++;
//       }
//       else if (arg == 2) 
//       {
//         argv2[indx] = chr;
//         indx++;
//       }
//     }
//   }
//   Serial.print(atoi(argv1));
//   Serial.print(" ");
//   Serial.println(atoi(argv2));
  
// // If we are using base control, run a PID calculation at the appropriate intervals
// // #ifdef USE_BASE
// //   if (millis() > nextPID) 
// //   {
// //     updatePID();
// //     nextPID += PID_INTERVAL;
// //   }
  
// //   // Check to see if we have exceeded the auto-stop interval
// //   if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) 
// //   {
// //     setMotorSpeeds(0, 0);
// //     moving = 0;
// //   }
// // #endif

// // Sweep servos
// #ifdef USE_SERVOS
//   int i;
//   for (i = 0; i < N_SERVOS; i++) {
//     servos[i].doSweep();
//   }
// #endif
// }
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include<Arduino.h>
#include<Wire.h>
TaskHandle_t read_imu;
TaskHandle_t read_encdrs;

Adafruit_MPU6050 mpu;

void read_imu_functn(void *);
void read_encodr_functn(void *);


void setup()
{
  Serial.begin(115200);
  Serial.println();

  //create a task dat excts read_imu_functn wid prirty 1 and exctd on core 0
  // xTaskCreatePinnedToCore(read_imu_functn,"reads_d_imu",1000,NULL,1,&read_imu,0);
  // //create a task dat excts read_encoder_functn wid prirty 1 and exctd on core 1
  // xTaskCreatePinnedToCore(read_encodr_functn,"reads_encoders",1000,NULL,1,&read_encdrs,1);
}

void loop()
{
  Serial.println("howdy!");
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

// void read_imu_functn(void *get_imu)
// {
//   Serial.print("reading imu dta");
//   if (!mpu.begin()) 
//   {
//     Serial.println("Failed to find MPU6050 chip");
//   }
//     Serial.println("MPU6050 Found!");
//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//     Serial.print("Accelerometer range set to: ");
//   switch (mpu.getAccelerometerRange()) 
//   {
//   case MPU6050_RANGE_2_G:
//     Serial.println("+-2G");
//     break;
//   case MPU6050_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case MPU6050_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case MPU6050_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   }

//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   Serial.print("Gyro range set to: ");
//   switch (mpu.getGyroRange()) 
//   {
//   case MPU6050_RANGE_250_DEG:
//     Serial.println("+- 250 deg/s");
//     break;
//   case MPU6050_RANGE_500_DEG:
//     Serial.println("+- 500 deg/s");
//     break;
//   case MPU6050_RANGE_1000_DEG:
//     Serial.println("+- 1000 deg/s");
//     break;
//   case MPU6050_RANGE_2000_DEG:
//     Serial.println("+- 2000 deg/s");
//     break;
//   }

//    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
//   Serial.print("Filter bandwidth set to: ");
//   switch (mpu.getFilterBandwidth()) 
//   {
//   case MPU6050_BAND_260_HZ:
//     Serial.println("260 Hz");
//     break;
//   case MPU6050_BAND_184_HZ:
//     Serial.println("184 Hz");
//     break;
//   case MPU6050_BAND_94_HZ:
//     Serial.println("94 Hz");
//     break;
//   case MPU6050_BAND_44_HZ:
//     Serial.println("44 Hz");
//     break;
//   case MPU6050_BAND_21_HZ:
//     Serial.println("21 Hz");
//     break;
//   case MPU6050_BAND_10_HZ:
//     Serial.println("10 Hz");
//     break;
//   case MPU6050_BAND_5_HZ:
//     Serial.println("5 Hz");
//     break;
//   }

//     Serial.println("");
//   delay(100);

// }
// void read_encodr_functn(void *get_encdrs)
// {
//   Serial.print("reading encdr dta");
// }