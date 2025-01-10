/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
//Encoder works well
//   #include<Arduino.h>
//   #include<ESP32Servo.h>

// #define EncoderPinA 13 //was 18 // encoder A and B are Blue and Yellow for Left motor
// #define EncoderPinB 14  //was 21 
// #define EncoderPinC 27   //encoder C and D are Blue and Yellow for Right motor
// #define EncoderPinD 26

// //motor control pins for motor A
// #define motorpin1_a 25  //left motor
// #define motorpin2_a 15
// #define enablepin_a 2
// //motor control pins for motor B
// #define motorpin1_b 5 //right motor
// #define motorpin2_b 18
// #define enablepin_b 4 
// //motor control pins for 5 robot arm servos
// #define shldr_pin 19
// #define waist_pin 23
// #define elbw_pin 22
// #define wrst_pin 32
// #define gripr_pin 34

// Servo waist;
// Servo sholdr;
// Servo elbw;
// Servo wrst;
// Servo gripr;

// int pos_waist = 0;
// int pos_sholdr = 0;
// int pos_elbw = 0;
// int pos_wrst = 0;
// int pos_gripr = 0;

// void updateEncoder();

// void updateEncoder_two();

// bool flg = 0;
// int rpm_one;
// int rpm_two;
// int motor_left_to_int;
// int motor_left_to_int_old = 0;
// int motor_right_to_int_old = 0;
// int motor_right_to_int;

// volatile long Encodervalue=0;

// volatile long Encodervalue_two=0;

// bool considr_numbr = 0;

// void setup() 
// {

//   pinMode(EncoderPinA, INPUT);// Encoder left pins and it works well
//   pinMode(EncoderPinB, INPUT);
//   pinMode(EncoderPinC, INPUT); //Encoder right pins
//   pinMode(EncoderPinD, INPUT);
  
//   pinMode(motorpin1_a,OUTPUT);//Motor left works well although it lits up blue LED which MBS if needed
//   pinMode(motorpin2_a,OUTPUT);
//   pinMode(enablepin_a,OUTPUT);
//   pinMode(motorpin1_b,OUTPUT); //Motor Right  works well
//   pinMode(motorpin2_b,OUTPUT);
//   pinMode(enablepin_b,OUTPUT);
  
//   ESP32PWM::allocateTimer(0);
// 	ESP32PWM::allocateTimer(1);
// 	ESP32PWM::allocateTimer(2);
// 	ESP32PWM::allocateTimer(3);

//   waist.setPeriodHertz(50);
//   sholdr.setPeriodHertz(50);
//   elbw.setPeriodHertz(50);
//   wrst.setPeriodHertz(50);
//   gripr.setPeriodHertz(50);


//   ledcSetup(0, 1000, 8);
//   ledcAttachPin(enablepin_a, 0); //motor_left

//   ledcSetup(1, 1000, 8);
//   ledcAttachPin(enablepin_b, 1);//motor_right

//   attachInterrupt(digitalPinToInterrupt(EncoderPinA), updateEncoder, RISING); //Belongs to left motor
//   attachInterrupt(digitalPinToInterrupt(EncoderPinC), updateEncoder_two, RISING); //Belongs to right motor
//   Serial.begin(115200);
// }

// void loop() 
// {
//   if(Serial.available())
//  {
//       String s =  Serial.readString();
//       // Serial.print("s is ");
//       // Serial.print(s);
//       String motor_left;
//       String motor_right;

//       for(auto d:s) //dis code which includes seeing if c is dere is now working 
//       {
//         if(d == 'c') // here c stands for computer. Here we get data from our computer
//         {
//             considr_numbr = 1;
//         }
//         else
//         {
//           if(considr_numbr == 1)
//           {
//             if(flg == 1)
//             {
//               motor_right +=d; 
//             }
//             else
//             {
//               motor_left +=d;
//             }
//             if(d == ',')
//             {
//               flg = 1;
//             }
//             if(d == ' ')
//             {
//               break;
//             }
//           }
//         }
//       }
//       // Serial.print("considr_numbr flag is");
//       // Serial.print(considr_numbr);
//       // Serial.print("motor_right is inside serial.available() ");
//       // Serial.print(motor_right);
//       // Serial.print("motor_left is inside serial.available() ");
//       // Serial.println(motor_left);
//       if(considr_numbr == 0)
//       {
//         motor_left_to_int = motor_left_to_int_old;
//         motor_right_to_int = motor_right_to_int_old; 
//       }
//       else
//       {
//         motor_left_to_int = motor_left.toInt();
//         motor_right_to_int = motor_right.toInt();
//       }
//       motor_left_to_int_old = motor_left_to_int;
//       motor_right_to_int_old = motor_right_to_int;
//       flg = 0;
//       considr_numbr = 0;
//       if(motor_left_to_int >=0 && motor_right_to_int >=0)
//       {
//         digitalWrite(motorpin1_a,HIGH); //motor left
//         digitalWrite(motorpin2_a,LOW);
//         ledcWrite(0,motor_left_to_int);
//         //ledcWrite(0,128);
        
//         digitalWrite(motorpin1_b,HIGH); //motor right
//         digitalWrite(motorpin2_b,LOW);
//         ledcWrite(1,motor_right_to_int);
//       //ledcWrite(1,128);
//       }
//       else if(motor_left_to_int >=0 && motor_right_to_int <0)
//       {
//         digitalWrite(motorpin1_a,HIGH); //motor left
//         digitalWrite(motorpin2_a,LOW);
//         ledcWrite(0,motor_left_to_int);
//         //ledcWrite(0,128);
        
//         digitalWrite(motorpin1_b,LOW); //motor right
//         digitalWrite(motorpin2_b,HIGH);
//         ledcWrite(1,-motor_right_to_int);
//       //ledcWrite(1,128);
//       }
//       else if(motor_left_to_int < 0 && motor_right_to_int >=0)
//       {
//         digitalWrite(motorpin1_a,LOW); //motor left
//         digitalWrite(motorpin2_a,HIGH);
//         ledcWrite(0,-motor_left_to_int);
//         //ledcWrite(0,128);
        
//         digitalWrite(motorpin1_b,HIGH); //motor right
//         digitalWrite(motorpin2_b,LOW);
//         ledcWrite(1,motor_right_to_int);
//       //ledcWrite(1,128);
//       }
//       else if(motor_left_to_int < 0 && motor_right_to_int <0)
//       {
//         digitalWrite(motorpin1_a,LOW); //motor left
//         digitalWrite(motorpin2_a,HIGH);
//         ledcWrite(0,-motor_left_to_int);
//         //ledcWrite(0,128);
        
//         digitalWrite(motorpin1_b,LOW); //motor right
//         digitalWrite(motorpin2_b,HIGH);
//         ledcWrite(1,-motor_right_to_int);
//       //ledcWrite(1,128);
//       }


//   }

//  // delay(1);

//   // Serial.print("motor left is");
//   // Serial.print(motor_left_to_int);
//   // Serial.print("motor right is");
//   // Serial.println(motor_right_to_int);
//   Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
//   Serial.print(Encodervalue); //use dis for communictn wid computer
//   Serial.print(',');
//   Serial.println(Encodervalue_two);


// }
// void updateEncoder() //belongs to left motor
// {
//   if (digitalRead(EncoderPinA)> digitalRead(EncoderPinB))
//   {
//     Encodervalue++;
//   }
//   else
//   {
//     Encodervalue--;
//   }
// }

// void updateEncoder_two() //belongs to right motor
// {
//   if (digitalRead(EncoderPinC)> digitalRead(EncoderPinD))
//   {
//     Encodervalue_two++;
//   }
//   else
//   {
//     Encodervalue_two--;
//   }
// }

// #include<Arduino.h> d code works
// #include<ESP32Servo.h>

// Servo waist;
// Servo sholdr;
// Servo elbw;
// Servo wrst_one;
// Servo wrst_two;
// Servo gripr;

// int pos = 0;
// int pos_two = 0;
// int pos_thr = 0;

// #define shldr_pin 22 //22
// #define waist_pin 34
// #define elbw_pin 33
// #define wrst_one_pin 35
// #define gripr_pin 27

// void setup()
// {
//   ESP32PWM::allocateTimer(0);
// 	ESP32PWM::allocateTimer(1);
// 	ESP32PWM::allocateTimer(2);
// 	ESP32PWM::allocateTimer(3);

//   waist.setPeriodHertz(50);
//   sholdr.setPeriodHertz(50);
//   elbw.setPeriodHertz(50);
//   wrst_one.setPeriodHertz(50);
//   gripr.setPeriodHertz(50);

//   waist.attach(waist_pin,1000,2000);
//   sholdr.attach(shldr_pin,1000,2000);
//   elbw.attach(elbw_pin,1000,2000);
//   wrst_one.attach(wrst_one_pin,1000,2000);
//   gripr.attach(gripr_pin,1000,2000);
  
//   // waist.write(40);
//   // sholdr.write(40);
// }

// void loop()
// {
//   // waist.write(40);
//   // sholdr.write(40);
//   // elbw.write(90);
//   // wrst_one.write(90);
//   // gripr.write(90);
//   for (pos = 0; pos <= 300; pos += 1) 
//   { // goes from 0 degrees to 180 degrees
//   // in steps of 1 degree
//   elbw.write(pos);    // tell servo to go to position in variable 'pos'
//   delay(15);             // waits 15ms for the servo to reach the position
//   }
// 	for (pos = 300; pos >= 0; pos -= 1) 
//   { // goes from 180 degrees to 0 degrees
// 		elbw.write(pos);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	}
//   for (pos_two = 0; pos_two <= 300; pos_two += 1) 
//     { // goes from 0 degrees to 180 degrees
// 		// in steps of 1 degree
// 		sholdr.write(pos_two);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	  }
// 	for (pos_two = 300; pos_two >= 0; pos_two -= 1) 
//   { // goes from 180 degrees to 0 degrees
// 		sholdr.write(pos_two);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	}
//   for (pos_thr = 0; pos_thr <= 300; pos_thr += 1) 
//   { // goes from 0 degrees to 180 degrees
//   // in steps of 1 degree
//   elbw.write(pos_thr);    // tell servo to go to position in variable 'pos'
//   delay(15);             // waits 15ms for the servo to reach the position
//   }
// 	for (pos_thr = 300; pos_thr >= 0; pos_thr -= 1) 
//   { // goes from 180 degrees to 0 degrees
// 		elbw.write(pos);    // tell servo to go to position in variable 'pos'
// 		delay(15);             // waits 15ms for the servo to reach the position
// 	}
// //  elbw.write(270);
// }