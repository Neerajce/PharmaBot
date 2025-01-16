//   #include<Arduino.h>
//   #include<ESP32Servo.h>
//   #include <ESP32Time.h>

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
// double old_time;
// double new_time;
// double new_time_two;

// ESP32Time rtc(3600); // create an instance with a specifed offset in seconds
// //rtc.offset;	// get or modify the current offset

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

//   //attachInterrupt(digitalPinToInterrupt(EncoderPinA), updateEncoder, RISING); //Belongs to left motor
//   //attachInterrupt(digitalPinToInterrupt(EncoderPinC), updateEncoder_two, RISING); //Belongs to right motor
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
//       old_time = rtc.getMicros();//millis();
//       new_time_two = rtc.getMicros();
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
//         //delay(20);
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
//         //delay(20);
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
//         //delay(20);
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
//        // delay(20);
//       //ledcWrite(1,128);
//       }

//   new_time = rtc.getMicros();
//   }
// // delay(4);
// //delay(10);
// //  delay(1000);

//   // Serial.print("motor left is");
//   // Serial.print(motor_left_to_int);
//   // Serial.print("motor right is");
//   // Serial.println(motor_right_to_int);
// //   Encodervalue = Encodervalue + 1;
// //   Encodervalue_two = Encodervalue_two + 1;
// //   Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
// //   Serial.print(Encodervalue); //use dis for communictn wid computer
// //   Serial.print(',');
// //   Serial.println(Encodervalue_two);
// double diff = old_time - new_time_two;
// double diff_two = old_time - new_time;
// Serial.print(' ');
// Serial.println(diff_two,20);
// Serial.print(diff,20);
// Serial.print(' ');
// //  // delay(5);
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

//below imp prt omitd
#include<Arduino.h>
#include"commands.h"
//#include"sensors.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
#define PID_RATE 30
const int PID_INTERVAL = 1000/PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 2000
long last_mtr_cmnd = AUTO_STOP_INTERVAL;
#define BAUDRATE 115200
#define AUTO_STOP_INTERVAL 2000

#define RIGHT_MOTOR_ENABLE 4
// #define RIGHT_ENC_PIN_A 5
// #define RIGHT_ENC_PIN_B 18

#define LEFT_MOTOR_ENABLE 2
// #define LEFT_ENC_PIN_A 15 instd of dis, we use a difrnt convntn which is LEFT_MOTOR_FORWARD and LEFT_MOTOR_BACKARD
// #define LEFT_ENC_PIN_B 25

#define LEFT_MOTOR_FORWARD 15
#define LEFT_MOTOR_BACKWARD 25

#define RIGHT_MOTOR_FORWARD 5
#define RIGHT_MOTOR_BACKWARD 18

  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
//stuff for L298_MOTOR_DRIVER
void initMotorController()
{
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
    // pinMode(LEFT_MOTOR_PIN_DIR,OUTPUT);
    // pinMode(RIGHT_MOTOR_PIN_DIR,OUTPUT);
}
void setMotorSpeed(int i, int spd)
{
    unsigned char reverse = 0;
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
    {
      spd = 255;
    }
    if(i ==LEFT)
    {

      if(reverse == 0) 
      { 
        analogWrite(LEFT_MOTOR_FORWARD, spd); //LEFT_MOTOR_FORWARD is LEFT_MOTOR_FORWARD_PIN_NUMBER
        analogWrite(LEFT_MOTOR_BACKWARD, 0);  //LEFT_MOTOR_BACKWARD is LEFT_MOTOR_BACKWARD_PIN_NUMBER just wrtn hre for cnvence
      }
      else if(reverse == 1) 
      { 
        analogWrite(LEFT_MOTOR_BACKWARD, spd); 
        analogWrite(LEFT_MOTOR_FORWARD, 0); 
      }   
    }
    else
    {
      if(reverse == 0) 
      {
        analogWrite(RIGHT_MOTOR_FORWARD, spd); 
        analogWrite(RIGHT_MOTOR_BACKWARD, 0); 
      }
      else if (reverse == 1) 
      { 
        analogWrite(RIGHT_MOTOR_BACKWARD, spd); 
        analogWrite(RIGHT_MOTOR_FORWARD, 0); 
      }    
    }
}
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    setMotorSpeed(LEFT,leftSpeed);
    setMotorSpeed(RIGHT,rightSpeed);
}

//above imp prt omitd
//   /* Interrupt routine for LEFT encoder, taking care of actual counting */
//   ISR(PCINT2_vect)
//   {
//   	  static uint8_t enc_last=0;
        
//       enc_last <<=2; //shift previous state two places
//       enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
      
//         left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//   }
  
//   /* Interrupt routine for RIGHT encoder, taking care of actual counting */
//   ISR(PCINT1_vect)
//   {
//         static uint8_t enc_last=0;
          	
// 	enc_last <<=2; //shift previous state two places
// 	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
//   	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//   }

//   /* Wrap the encoder initializeing to be run in the main setup() function */
//   void initEncoders() 
//   {
//     //set as inputs
//     DDRD &= ~(1<<LEFT_ENC_PIN_A);
//     DDRD &= ~(1<<LEFT_ENC_PIN_B);
//     DDRC &= ~(1<<RIGHT_ENC_PIN_A);
//     DDRC &= ~(1<<RIGHT_ENC_PIN_B);
  
//     // Enable pull up resistors
//     PORTD |= (1<<LEFT_ENC_PIN_A);
//     PORTD |= (1<<LEFT_ENC_PIN_B);
//     PORTC |= (1<<RIGHT_ENC_PIN_A);
//     PORTC |= (1<<RIGHT_ENC_PIN_B);
  
//     // Tell pin change mask to listen to left encoder pins
//     PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
//     // Tell pin change mask to listen to right encoder pins
//     PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
  
//     // Enable PCINT1 and PCINT2 interrupt in the general interrupt mask
//     PCICR |= (1 << PCIE1) | (1 << PCIE2);
//   }
  
//   /* Wrap the encoder reading function */
//   long readEncoder(int i) 
//   {
//     if (i == LEFT) 
//     {
//       return left_enc_pos;
//     }
//     else 
//     {
//       return right_enc_pos;
//     }
//   }

//   /* Wrap the encoder reset function */
//   void resetEncoder(int i) 
//   {
//     if (i == LEFT)
//     {
//       left_enc_pos=0L;
//       return;
//     } 
//     else 
//     { 
//       right_enc_pos=0L;
//       return;
//     }
//   }

//   /* Wrap the encoder reset function */
//   void resetEncoders() 
//   {
//     resetEncoder(LEFT);
//     resetEncoder(RIGHT);
//   }


//below imp prt omitd
// // a pair of variables to help parse serial comnds
int arg = 0;
int indx = 0;

 long lastMotorCommand = AUTO_STOP_INTERVAL;
//varibl to hold an input charctr
char chr;

//chractr arrays to hold d frst and scnd argumnts
char argv1[32];
char argv2[32];

// The arguments converted to integers
long arg1;
long arg2;

char cmd;

void resetCommand()
{
    cmd = NULL;
    memset(argv1,0,sizeof(argv1));
    memset(argv2,0,sizeof(argv2));
    arg1 = 0;
    arg2 = 0;
    arg = 0;
    //indx = 0;
}

int runCommand()
{
    int i;
    char *p = argv1;
    char *str;
    int pid_args[4];
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    String output;
    switch(cmd)
    {
        case ANALOG_READ: //  ANALOG_READ is a
            Serial.println(analogRead(arg1));
            break;
        case DIGITAL_READ: //DIGITAL_READ is d
            Serial.println(digitalRead(arg1));
            break;
        case ANALOG_WRITE: //ANALOG_WRITE is x
            analogWrite(arg1, arg2);
            Serial.println(F("OK")); 
            break;
        case DIGITAL_WRITE: //digital write consists of only 0 and 1 | DIGITAL_WRITE  is 'w'
            if (arg2 == 0) 
            {
                digitalWrite(arg1, LOW);
            }
            else if (arg2 == 1) 
            {
                digitalWrite(arg1, HIGH);
            }
            Serial.println(F("OK")); 
            break;
        case PIN_MODE: // PIN_MODE    is   'c'
            if (arg2 == 0) 
            {
                pinMode(arg1, INPUT);
            }
            else if (arg2 == 1) 
            {
                pinMode(arg1, OUTPUT);
            }
            Serial.println(F("OK"));
            break;  
        // case READ_ENCODERS:
        //     Serial.print(readEncoder(LEFT));
        //     Serial.print(F(" "));
        //     Serial.println(readEncoder(RIGHT));
        //     break;
        // case RESET_ENCODERS:
        //     resetEncoders();
        //     resetPID();
        //     Serial.println(F("OK"));
        //     break;
        case MOTOR_SPEEDS:
            /* Reset the auto stop timer */
            lastMotorCommand = millis();
            if (arg1 == 0 && arg2 == 0) 
            {
                setMotorSpeeds(0, 0);
                resetPID();
                moving = 0;
            }
            else
            { 
                moving = 1;
            }
            leftPID.TargetTicksPerFrame = arg1;
            rightPID.TargetTicksPerFrame = arg2;
            Serial.println(F("OK")); 
            break;
        case UPDATE_PID:
            i = 0;
            while ((str = strtok_r(p, ":", &p)) != NULL) 
            {
            pid_args[i] = atoi(str);
            i++;
            }
            Kp = pid_args[0];
            Kd = pid_args[1];
            Ki = pid_args[2];
            Ko = pid_args[3];
            Serial.println(F("OK"));
            break;

        default:
            Serial.println(F("Invalid Command"));
            break;
    }
    return 0;
}

void setup()
{
    Serial.begin(BAUDRATE);
    //initEncoders();
    initMotorController();
    resetPID();

    #ifdef USE_SERVOS
    int i;
    for(i = 0; i < N_SERVOS;i++)
    {
        servos[i].initServo(servoPins[i],stepDelay[i],servoInitPosition[i]);
    }
    #endif
}

void loop()
{
    while(Serial.available() > 0)
    {
        chr = Serial.read();
        // Terminate a command with a CR
        if (chr == 13) 
        {
            if (arg == 1) 
            {
                argv1[indx] = NULL;
            }
            else if (arg == 2) 
            {
                argv2[indx] = NULL;
            }
            runCommand();
            resetCommand();
        }
        // Use spaces to delimit parts of the command
        else if (chr == ' ') 
        {
        // Step through the arguments
            if (arg == 0) 
            {
                arg = 1;
            }
            else if (arg == 1)  
            {
                argv1[indx] = NULL;
                arg = 2;
                indx = 0;
            }
            continue;
        }
        else 
        {
            if (arg == 0) 
            {
                // The first arg is the single-letter command
                cmd = chr;
            }
            else if (arg == 1) 
            {
                // Subsequent arguments can be more than one character
                argv1[indx] = chr;
                indx++;
            }
            else if (arg == 2) 
            {
                argv2[indx] = chr;
                indx++;
            }
        }
    }

    if (millis() > nextPID) 
    {
        updatePID();
        nextPID += PID_INTERVAL;        
    }
    if((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
    {
        setMotorSpeeds(0, 0);
        moving = 0;
    }

}