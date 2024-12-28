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

// //  delay(1000);

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

