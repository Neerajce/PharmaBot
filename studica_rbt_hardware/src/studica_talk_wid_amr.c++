//   #include<Arduino.h>
//   #include<Arduino.h>
//   #include<ESP32Servo.h>

// #define EncoderPinA 18
// #define EncoderPinB 21
// #define EncoderPinC 27
// #define EncoderPinD 26

// //motor control pins for motor A
// #define motorpin1_a 5
// #define motorpin2_a 4
// #define enablepin_a 2
// //motor control pins for motor B
// #define motorpin1_b 23
// #define motorpin2_b 32
// #define enablepin_b 19

// void updateEncoder();
// void updateEncoder_two();

// volatile long Encodervalue=0;
// volatile long Encodervalue_two=0;
// void setup() 
// {

//   pinMode(EncoderPinA, INPUT);// Encoder works well
//   pinMode(EncoderPinB, INPUT);
//   pinMode(EncoderPinC, INPUT);
//   pinMode(EncoderPinD, INPUT);
  
//   pinMode(motorpin1_a,OUTPUT);//Motor left works well although it lits up blue LED which MBS if needed
//   pinMode(motorpin2_a,OUTPUT);
//   pinMode(enablepin_a,OUTPUT);
//   pinMode(motorpin1_b,OUTPUT); //Motor Right  works well
//   pinMode(motorpin2_b,OUTPUT);
//   pinMode(enablepin_b,OUTPUT);
  
//   ledcSetup(0, 1500, 8);
//   ledcAttachPin(enablepin_a, 0);

//   ledcSetup(1, 1500, 8);
//   ledcAttachPin(enablepin_b, 1);

//   attachInterrupt(digitalPinToInterrupt(EncoderPinA), updateEncoder, RISING);
//   attachInterrupt(digitalPinToInterrupt(EncoderPinC), updateEncoder_two, RISING);
//   Serial.begin(9600);
// }
// bool flg = 0;
// int rpm_one;
// int rpm_two;
// int motor_left_to_int;
// int motor_right_to_int;
// void loop() 
// {
//   if(Serial.available())
//   {
//       String s =  Serial.readString();
//       String motor_left;
//       String motor_right;

//       for(auto d:s)
//       {
//         if(d == 'c') // here c stands for computer. Here we get data from our computer
//         {
//             //don't do anything
//         }
//         else
//         {
//           if(flg == 1)
//           {
//             motor_right +=d; 
//           }
//           else
//           {
//             motor_left +=d;
//           }
//           if(d == ',')
//           {
//             flg = 1;
//           }
//         }
//       }
//       motor_left_to_int = motor_left.toInt();
//       motor_right_to_int = motor_right.toInt();
//       flg = 0;
//       digitalWrite(motorpin1_a,LOW);
//       digitalWrite(motorpin2_a,HIGH);
//       ledcWrite(0,motor_left_to_int);
//       //ledcWrite(0,128);
//       digitalWrite(motorpin1_b,HIGH);
//       digitalWrite(motorpin2_b,LOW);
//      ledcWrite(1,motor_right_to_int);
//     //  ledcWrite(1,128);

//     // if(Serial.available())
//     // {

//     //     // //motor st
//     // }
//   }

//   delay(1000);
//   Serial.print(motor_left_to_int);
//   Serial.println(motor_right_to_int);
//   //Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
//   // Serial.print(Encodervalue); //use dis for communictn wid computer
//   // Serial.print(',');
//   // Serial.println(Encodervalue_two);


// }
// void updateEncoder()
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

// void updateEncoder_two()
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

