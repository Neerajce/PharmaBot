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

//         for(auto d:s)
//         {
//             if(d=='c')
//             {
//                 considr_numbr = 1;
//             }
//             else
//             {
//                 if(considr_numbr == 1)
//                 {
//                     digitalWrite(motorpin1_a,HIGH); //motor left
//                     digitalWrite(motorpin2_a,LOW);
//                     ledcWrite(0,motor_left_to_int);
//                     //ledcWrite(0,128);

//                     digitalWrite(motorpin1_b,HIGH); //motor right
//                     digitalWrite(motorpin2_b,LOW);
//                     ledcWrite(1,motor_right_to_int);
//                 }
//             }
//         }
//  }

//    Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
//   Serial.print(Encodervalue); //use dis for communictn wid computer
//   Serial.print(',');
//   Serial.println(Encodervalue_two);
//     //delay(0.5);
// }

// //       for(auto d:s) //dis code which includes seeing if c is dere is now working 
// //       {
// //         if(d == 'c') // here c stands for computer. Here we get data from our computer
// //         {
// //             considr_numbr = 1;
// //         }
// //         else
// //         {
// //           if(considr_numbr == 1)
// //           {
// //             if(flg == 1)
// //             {
// //               motor_right +=d; 
// //             }
// //             else
// //             {
// //               motor_left +=d;
// //             }
// //             if(d == ',')
// //             {
// //               flg = 1;
// //             }
// //             if(d == ' ')
// //             {
// //               break;
// //             }
// //           }
// //         }
// //       }
// //       // Serial.print("considr_numbr flag is");
// //       // Serial.print(considr_numbr);
// //       // Serial.print("motor_right is inside serial.available() ");
// //       // Serial.print(motor_right);
// //       // Serial.print("motor_left is inside serial.available() ");
// //       // Serial.println(motor_left);
// //       if(considr_numbr == 0)
// //       {
// //         motor_left_to_int = motor_left_to_int_old;
// //         motor_right_to_int = motor_right_to_int_old; 
// //       }
// //       else
// //       {
// //         motor_left_to_int = motor_left.toInt();
// //         motor_right_to_int = motor_right.toInt();
// //       }
// //       motor_left_to_int_old = motor_left_to_int;
// //       motor_right_to_int_old = motor_right_to_int;
// //       flg = 0;
// //       considr_numbr = 0;
// //       if(motor_left_to_int >=0 && motor_right_to_int >=0)
// //       {
// //         digitalWrite(motorpin1_a,HIGH); //motor left
// //         digitalWrite(motorpin2_a,LOW);
// //         ledcWrite(0,motor_left_to_int);
// //         //ledcWrite(0,128);

// //         digitalWrite(motorpin1_b,HIGH); //motor right
// //         digitalWrite(motorpin2_b,LOW);
// //         ledcWrite(1,motor_right_to_int);
// //         //delay(20);
// //       //ledcWrite(1,128);
// //       }
// //       else if(motor_left_to_int >=0 && motor_right_to_int <0)
// //       {
// //         digitalWrite(motorpin1_a,HIGH); //motor left
// //         digitalWrite(motorpin2_a,LOW);
// //         ledcWrite(0,motor_left_to_int);
// //         //ledcWrite(0,128);

// //         digitalWrite(motorpin1_b,LOW); //motor right
// //         digitalWrite(motorpin2_b,HIGH);
// //         ledcWrite(1,-motor_right_to_int);
// //         //delay(20);
// //       //ledcWrite(1,128);
// //       }
// //       else if(motor_left_to_int < 0 && motor_right_to_int >=0)
// //       {
// //         digitalWrite(motorpin1_a,LOW); //motor left
// //         digitalWrite(motorpin2_a,HIGH);
// //         ledcWrite(0,-motor_left_to_int);
// //         //ledcWrite(0,128);

// //         digitalWrite(motorpin1_b,HIGH); //motor right
// //         digitalWrite(motorpin2_b,LOW);
// //         ledcWrite(1,motor_right_to_int);
// //         //delay(20);
// //       //ledcWrite(1,128);
// //       }
// //       else if(motor_left_to_int < 0 && motor_right_to_int <0)
// //       {
// //         digitalWrite(motorpin1_a,LOW); //motor left
// //         digitalWrite(motorpin2_a,HIGH);
// //         ledcWrite(0,-motor_left_to_int);
// //         //ledcWrite(0,128);

// //         digitalWrite(motorpin1_b,LOW); //motor right
// //         digitalWrite(motorpin2_b,HIGH);
// //         ledcWrite(1,-motor_right_to_int);
// //        // delay(20);
// //       //ledcWrite(1,128);
// //       }


// //   }
// // delay(4);
// //delay(10);
// //  delay(1000);

//   // Serial.print("motor left is");
//   // Serial.print(motor_left_to_int);
//   // Serial.print("motor right is");
//   // Serial.println(motor_right_to_int);
// //   Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
// //   Serial.print(Encodervalue); //use dis for communictn wid computer
// //   Serial.print(',');
// //   Serial.println(Encodervalue_two);
// //  // delay(5);


// // }
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

// a pair of variables to help parse serial comnds
int arg = 0;
int index = 0;

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
    index = 0;
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
        case GET_BAUDRATE:
            Serial.print(F(" "));
            Serial.println(BAUDRATE);
            break;
        case ANALOG_READ:
            Serial.println(analogRead(arg1));
            break;
        case DIGITAL_READ:
            Serial.println(digitalRead(arg1));
            break;
        case ANALOG_WRITE:
            analogWrite(arg1, arg2);
            Serial.println(F("OK")); 
            break;
        case DIGITAL_WRITE:
            if (arg2 == 0) digitalWrite(arg1, LOW);
            else if (arg2 == 1) digitalWrite(arg1, HIGH);
            Serial.println(F("OK")); 
            break;
        case PIN_MODE:
            if (arg2 == 0) pinMode(arg1, INPUT);
            else if (arg2 == 1) pinMode(arg1, OUTPUT);
            Serial.println(F("OK"));
            break;  
        // case PING:
        //         Serial.println(Ping(arg1));
        //         break;
    #ifdef USE_SERVOS
    case SERVO_WRITE:
            servos[arg1].setTargetPosition(arg2);
            Serial.println(F("OK"));
            break;
    case SERVO_READ:
            Serial.println(servos[arg1].getServo().read());
            break;
    case DETACH_SERVO:
        myServos[arg1].getServo().detach();
        myServos[arg1].disable();
        Serial.println(F("OK"));
        break;
    case ATTACH_SERVO:
        if (!haveServo(arg1)) {
        myServos[arg1].initServo(arg1, 0);
        myServoPins[nServos] = arg1;
        nServos++;
        }
        else {
        myServos[arg1].getServo().attach(arg1);  
        }
        myServos[arg1].enable();
        Serial.println(F("OK"));
        break;

    #endif
        case READ_ENCODERS:
            Serial.print(readEncoder(LEFT));
            Serial.print(F(" "));
            Serial.println(readEncoder(RIGHT));
            break;
        case RESET_ENCODERS:
            resetEncoders();
            resetPID();
            Serial.println(F("OK"));
            break;
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
            while ((str = strtok_r(p, ":", &p)) != '\0') 
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
}

    void setup()
    {
        Serial.begin(BAUDRATE);
        initEncoders();
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
                    argv1[index] = NULL;
                }
                else if (arg == 2) 
                {
                    argv2[index] = NULL;
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
                    argv1[index] = NULL;
                    arg = 2;
                    index = 0;
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
                    argv1[index] = chr;
                    index++;
                }
                else if (arg == 2) 
                {
                    argv2[index] = chr;
                    index++;
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

        #ifdef USE_SERVOS
            int i;
            for (i = 0; i < N_SERVOS; i++) 
            {
                servos[i].doSweep();
            }
            
        #elif defined(USE_SERVOS2)
            int i, pin;
            for (i = 0; i < nServos; i++) 
            {
                pin = myServoPins[i];
                myServos[pin].moveServo();
            }
        #endif
    


}