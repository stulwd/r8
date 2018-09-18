#include <Arduino.h>
#include <servo.h>
#include <ps2.h>

#define PS2_DAT PA6
#define PS2_CMD PA7
#define PS2_CS  PA4
#define PS2_CLK PA5

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo motor;
#define AIN1 PB0
#define AIN2 PB1
#define BIN1 PB13
#define BIN2 PB14
#define CIN1 PC10
#define CIN2 PC11
#define DIN1 PC12
#define DIN2 PD2
#define PWMA PC6
#define PWMB PC7
#define PWMC PC8
#define PWMD PC9

void setup() {
    // put your setup code here, to run once:
    
    afio_init();
    afio_remap(AFIO_REMAP_TIM3_FULL);
    Serial1.begin(9600);
    servo1.attach(PA2);
    servo2.attach(PA3);
    servo3.attach(PB8);
    servo4.attach(PB9);
    motor.attach(PWMA);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(CIN1, OUTPUT);
    pinMode(CIN2, OUTPUT);
    pinMode(DIN1, OUTPUT);
    pinMode(DIN2, OUTPUT);
 
   // pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMC, OUTPUT);
    pinMode(PWMD, OUTPUT); 

    PS2::setupPS2(PS2_CLK, PS2_CS, PS2_CMD, PS2_DAT);
}


    int pos = 0;
    uint8_t vol = 0;
    bool dir = 1;
    int pos1=0, pos2=0, pos3=0, pos4=0;

void cap(int *pos)
{
    *pos = (*pos > 180) ? 180: *pos;
    *pos = (*pos < 0) ? 0: *pos;
}
void loop() {

    //put your main code here, to run repeatedly:
    PS2::updatePS2();



    //motor control
    if(PS2::getButton(PS2Up))
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);

        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);

        digitalWrite(CIN1, HIGH);
        digitalWrite(CIN2, LOW);

        digitalWrite(DIN1, HIGH);
        digitalWrite(DIN2, LOW);

        motor.write(180);
       // digitalWrite(PWMA, HIGH);
        digitalWrite(PWMB, HIGH);
        digitalWrite(PWMC, HIGH);
        digitalWrite(PWMD, HIGH);        

    }

    if(PS2::getButton(PS2Down))
    {
        digitalWrite(AIN2, HIGH);
        digitalWrite(AIN1, LOW);

        digitalWrite(BIN2, HIGH);
        digitalWrite(BIN1, LOW);
        
        digitalWrite(CIN2, HIGH);
        digitalWrite(CIN1, LOW);
        
        digitalWrite(DIN2, HIGH);
        digitalWrite(DIN1, LOW);

        motor.write(0);
        //digitalWrite(PWMA, HIGH);
        digitalWrite(PWMB, HIGH);
        digitalWrite(PWMC, HIGH);
        digitalWrite(PWMD, HIGH);  

    }

    if(PS2::getButton(PS2X))
    {
        digitalWrite(PWMA, LOW);
        digitalWrite(PWMB, LOW);
        digitalWrite(PWMC, LOW);
        digitalWrite(PWMD, LOW);        
    }


    //servo control
    if(PS2::getButton(PS2L1))
    { pos1 += 2; cap(&pos1); servo1.write(pos1);}
    if(PS2::getButton(PS2L2))
    { pos1 -= 2; cap(&pos1); servo1.write(pos1);}

    if(PS2::getButton(PS2R1))
    { pos4 += 2; cap(&pos4); servo4.write(pos4);}
    if(PS2::getButton(PS2R2))
    { pos4 -= 2; cap(&pos4); servo4.write(pos4);}

    if(PS2::getButton(PS2S))
    { pos3 += 2; cap(&pos3); servo3.write(pos3);}
    if(PS2::getButton(PS2O))
    { pos3 -= 2; cap(&pos3); servo3.write(pos3);}

    if(PS2::getButton(PS2Left))
    { pos2 += 2; cap(&pos2); servo2.write(pos2);}
    if(PS2::getButton(PS2Right))
    { pos2 -= 2; cap(&pos2); servo2.write(pos2);}





    delay(10);
    Serial.write('0');
    Serial1.write(pos);
    Serial1.print('a');
    if(pos == 180)
    {
        dir = 0;
    }
    if(pos == 0)
    {
        dir = 1;
    }
    if(dir == 1)
    {
        pos = pos + 5;
        vol += 7;
    }
    if(dir == 0)
    {
        pos = pos - 5;
        vol -= 7;
    }
    // digitalWrite(PWMA, HIGH);
    // digitalWrite(PWMB, HIGH);
    // digitalWrite(PWMC, HIGH);
    // digitalWrite(PWMD, HIGH);



}






