void initMotorController()
{
    pinMode(LEFT_MOTOR_PIN_DIR,OUTPUT);
    pinMode(RIGHT_MOTOR_PIN_DIR,OUTPUT);
}
void setMotorSpeed(int i, int spd)
{
    if (i == LEFT)
    {
        if(spd < 0)
        {
            digitalWrite(LEFT_MOTOR_PIN_DIR,LOW);
        }
        else
        {
            digitalWrite(LEFT_MOTOR_PIN_DIR,HIGH);
        }
        analogWrite(LEFT_MOTOR_PIN_SPEED,spd);
        
    }
    else
    {
        if(spd < 0)
        {
          digitalWrite(RIGHT_MOTOR_PIN_DIR, LOW);
        }
        else
        {
            digitalWrite(RIGHT_MOTOR_PIN_DIR, HIGH);
        }
        analogWrite(RIGHT_MOTOR_PIN_SPEED,spd);
    }
}
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    setMotorSpeed(LEFT,leftSpeed);
    setMotorSpeed(RIGHT,rightSpeed);
}