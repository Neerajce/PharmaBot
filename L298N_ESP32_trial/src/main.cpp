  #include<Arduino.h>
  #include<ESP32Servo.h>

// Dis is USB0 ESP32 
//motor control pins for motor A below
#define motorpin1_a 25  //left motor
#define motorpin2_a 15
#define enablepin_a 2
//motor control pins for motor B below
#define motorpin1_b 5 //right motor
#define motorpin2_b 18
#define enablepin_b 4
//motor control pins for 5 robot arm servos below
#define shldr_pin 19
#define waist_pin 23
#define elbw_pin 22
#define wrst_pin 32
#define gripr_pin 34

Servo waist;
Servo sholdr;
Servo elbw;
Servo wrst;
Servo gripr;

std::string rbt_arm_strng;

bool flg = 0;
int rpm_one;
int rpm_two;
int motor_left_to_int;
int motor_left_to_int_old = 0;
int motor_right_to_int_old = 0;
int motor_right_to_int;
volatile long Encodervalue=0;
volatile long Encodervalue_two=0;
bool considr_numbr = 0;
bool considr_rbt_arm_numbr = 0;
int waist_angle;
int sholdr_angle;
int elbw_angle;
int wrst_angle;
int gripr_angle;

int waist_angle_old;
int sholdr_angle_old;
int elbw_angle_old;
int wrst_angle_old;
int gripr_angle_old;

bool waist_reachd = 1;
bool sholdr_reachd = 1;
bool elbw_reachd = 1;
bool wrst_reachd = 1;
bool gripr_reachd = 1;




//rtc.offset;	// get or modify the current offset

void setup() 
{
  pinMode(motorpin1_a,OUTPUT);//Motor left works well although it lits up blue LED which MBS if needed
  pinMode(motorpin2_a,OUTPUT);
  pinMode(enablepin_a,OUTPUT);
  pinMode(motorpin1_b,OUTPUT); //Motor Right  works well
  pinMode(motorpin2_b,OUTPUT);
  pinMode(enablepin_b,OUTPUT);

  waist.setPeriodHertz(50);
  sholdr.setPeriodHertz(50);
  elbw.setPeriodHertz(50);
  wrst.setPeriodHertz(50);
  gripr.setPeriodHertz(50);

  waist.attach(waist_pin,1000,2000);
  sholdr.attach(shldr_pin,1000,2000);
  elbw.attach(elbw_pin,1000,2000);
  wrst.attach(wrst_pin,1000,2000);
  gripr.attach(gripr_pin,1000,2000);


  ledcSetup(0, 1000, 8);
  ledcAttachPin(enablepin_a, 0); //motor_left

  ledcSetup(1, 1000, 8);
  ledcAttachPin(enablepin_b, 1);//motor_right

  Serial.begin(115200);
}

void loop() 
{
  if(Serial.available())
 {
      String s =  Serial.readString();
      // Serial.print("s is ");
      // Serial.print(s);
      String motor_left;
      String motor_right;
      for(auto d:s) //dis code which includes seeing if c is dere is now working 
      {
        if(d == 'c') // here c stands for computer. Here we get data from our computer
        {
            considr_numbr = 1;
        }
        else
        {
          if(considr_numbr == 1)
          {
            if(flg == 1)
            {
              motor_right +=d; 
            }
            else
            {
              motor_left +=d;
            }
            if(d == ',')
            {
              flg = 1;
            }
            if(d == ' ')
            {
              break;
            }
          }
        }
        if(d == 'a')
        {
          considr_rbt_arm_numbr = 1;
          waist_angle = atoi(s.substring(1,4).c_str());
          sholdr_angle = atoi(s.substring(4,7).c_str());
          elbw_angle = atoi(s.substring(7,10).c_str());
          wrst_angle = atoi(s.substring(10,13).c_str());
          gripr_angle = atoi(s.substring(13,16).c_str());
          waist_reachd = 0;
          sholdr_reachd = 0;
          elbw_reachd = 0;
          wrst_reachd = 0;
          gripr_reachd = 0;
        }
        else
        {
          considr_rbt_arm_numbr = 0;
        }
      }
      if(considr_numbr == 0)
      {
        motor_left_to_int = motor_left_to_int_old;
        motor_right_to_int = motor_right_to_int_old; 
      }
      else
      {
        motor_left_to_int = motor_left.toInt();
        motor_right_to_int = motor_right.toInt();
      }
      motor_left_to_int_old = motor_left_to_int;
      motor_right_to_int_old = motor_right_to_int;
      flg = 0;
      considr_numbr = 0;

      if(considr_rbt_arm_numbr == 0)
      {        
        waist_angle_old = waist_angle;
        sholdr_angle_old = sholdr_angle;
        elbw_angle_old = elbw_angle;
        wrst_angle_old = wrst_angle;
        gripr_angle_old = gripr_angle;
      }


      if(considr_rbt_arm_numbr == 1)
      {
            if(waist_angle_old < waist_angle)
            {
              for(int temp_waist = waist_angle_old;temp_waist <=waist_angle;temp_waist +=1)
              {
                waist.write(temp_waist);
                delay(15);
              }
              waist_reachd = 1;
            }
            else if(waist_angle_old > waist_angle)
            {
              for(int temp_waist = waist_angle_old;temp_waist >=waist_angle;temp_waist-=1)
              {
                waist.write(temp_waist);
                delay(15);
              }
              waist_reachd = 1;
            }
            else
            {
              waist_reachd = 1;
            }

            if(sholdr_angle_old < sholdr_angle)
            {
              for(int temp_sholdr = sholdr_angle_old;temp_sholdr <=sholdr_angle;temp_sholdr +=1)
              {
                sholdr.write(temp_sholdr);
                delay(15);
              }
              sholdr_reachd = 1;
            }
            else if(sholdr_angle_old > sholdr_angle)
            {
              for(int temp_sholdr = sholdr_angle_old;temp_sholdr >=sholdr_angle;temp_sholdr-=1)
              {
                sholdr.write(temp_sholdr);
                delay(15);
              }
              sholdr_reachd = 1;
            }
            else
            {
              sholdr_reachd = 1;
            }

            if(elbw_angle_old < elbw_angle)
            {
              for(int temp_elbw = elbw_angle_old;temp_elbw <=elbw_angle;temp_elbw +=1)
              {
                waist.write(temp_elbw);
                delay(15);
              }
              elbw_reachd = 1;
            }
            else if(elbw_angle_old > elbw_angle)
            {
              for(int temp_elbw = elbw_angle_old;temp_elbw >=elbw_angle;temp_elbw-=1)
              {
                elbw.write(temp_elbw);
                delay(15);
              }
              elbw_reachd = 1;
            }
            else
            {
              elbw_reachd = 1;
            }


            if(wrst_angle_old < wrst_angle)
            {
              for(int temp_wrst = wrst_angle_old;temp_wrst <=wrst_angle;temp_wrst +=1)
              {
                waist.write(temp_wrst);
                delay(15);
              }
              wrst_reachd = 1;
            }
            else if(wrst_angle_old > wrst_angle)
            {
              for(int temp_wrst = wrst_angle_old;temp_wrst >=wrst_angle;temp_wrst-=1)
              {
                wrst.write(temp_wrst);
                delay(15);
              }
              wrst_reachd = 1;
            }
            else
            {
              wrst_reachd = 1;
            }

            if(gripr_angle_old < gripr_angle)
            {
              for(int temp_gripr = gripr_angle_old;temp_gripr <=gripr_angle;temp_gripr +=1)
              {
                waist.write(temp_gripr);
                delay(15);
              }
              gripr_reachd = 1;
            }
            else if(gripr_angle_old > gripr_angle)
            {
              for(int temp_gripr = gripr_angle_old;temp_gripr >=gripr_angle;temp_gripr-=1)
              {
                gripr.write(temp_gripr);
                delay(15);
              }
              gripr_reachd = 1;
            }
            else
            {
              gripr_reachd = 1;
            }
      }

      if(waist_reachd == 1 && sholdr_reachd == 1 && elbw_reachd == 1 && wrst_reachd == 1 && gripr_reachd == 1)
      {
        considr_rbt_arm_numbr = 0;
      }

      if(motor_left_to_int >=0 && motor_right_to_int >=0)
      {
        digitalWrite(motorpin1_a,HIGH); //motor left
        digitalWrite(motorpin2_a,LOW);
        ledcWrite(0,motor_left_to_int);
        //ledcWrite(0,128);

        digitalWrite(motorpin1_b,HIGH); //motor right
        digitalWrite(motorpin2_b,LOW);
        ledcWrite(1,motor_right_to_int);
        //delay(20);
      //ledcWrite(1,128);
      }
      else if(motor_left_to_int >=0 && motor_right_to_int <0)
      {
        digitalWrite(motorpin1_a,LOW); //motor left
        digitalWrite(motorpin2_a,HIGH);
        ledcWrite(0,motor_left_to_int);
        //ledcWrite(0,128);

        digitalWrite(motorpin1_b,HIGH); //motor right
        digitalWrite(motorpin2_b,LOW);
        ledcWrite(1,-motor_right_to_int);
        //delay(20);
      //ledcWrite(1,128);
      }
      else if(motor_left_to_int < 0 && motor_right_to_int >=0)
      {
        digitalWrite(motorpin1_a,HIGH); //motor left
        digitalWrite(motorpin2_a,LOW);
        ledcWrite(0,-motor_left_to_int);
        //ledcWrite(0,128);

        digitalWrite(motorpin1_b,LOW); //motor right
        digitalWrite(motorpin2_b,HIGH);
        ledcWrite(1,motor_right_to_int);
        //delay(20);
      //ledcWrite(1,128);
      }
      else if(motor_left_to_int < 0 && motor_right_to_int <0)
      {
        digitalWrite(motorpin1_a,LOW); //motor left
        digitalWrite(motorpin2_a,HIGH);
        ledcWrite(0,-motor_left_to_int);
        //ledcWrite(0,128);

        digitalWrite(motorpin1_b,LOW); //motor right
        digitalWrite(motorpin2_b,HIGH);
        ledcWrite(1,-motor_right_to_int);
       // delay(20);
      //ledcWrite(1,128);
      }

  }
delay(2);
//delay(10);
//  delay(1000);
//printn stuf here works
      Serial.print(waist_angle);
      Serial.print(',');
      Serial.print(sholdr_angle);
      Serial.print(',');
      Serial.print(wrst_angle);
      Serial.print(',');
      Serial.print(gripr_angle);
      Serial.print(',');
      Serial.println(elbw_angle);
  // Serial.print("motor left is");
  // Serial.print(motor_left_to_int);
  // Serial.print("motor right is");
  // Serial.println(motor_right_to_int);
//   Encodervalue = Encodervalue + 1;
//   Encodervalue_two = Encodervalue_two + 1;
//   Serial.print('e'); //here e stands for esp32. W send data from esp32 to our computer
//   Serial.print(Encodervalue); //use dis for communictn wid computer
//   Serial.print(',');
//   Serial.println(Encodervalue_two);
//  // delay(5);
}
