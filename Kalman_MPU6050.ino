// vcc -- 3.3v
// scl a5
// sda a4
// int d2

#include<Wire.h>
#include <Servo.h>

Servo servo1; 

#define DATA_LEN 14
#define DATA_IN_LEN 7
#define loopPin 4


// I2C address of the MPU-6050
const int MPU_addr=0x68;  
// Mediciones
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t datos[DATA_IN_LEN];

char out_msg[DATA_LEN];
bool kill=false;

char val, last_val;
char ch;
int servo_val = -180;
void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  pinMode(loopPin, INPUT);        
  
  servo1.attach(14);
  servo_val = 64;
  servo1.write(servo_val);
  
  Serial.begin(9600);

  last_val = -1;
}



void loop()
{
  /*
  delay(1000);
  servo1.write(servo_val);

  Serial.println(servo_val);
  servo_val = servo_val + 10;
  */
  
  if (Serial.available() > 0) 
  {
    // get incoming byte:
    ch = Serial.read();   

    if(kill == true)
    {
      kill = false;
      servo_val = 64;       
      servo1.write(servo_val);

      for (int idx = 0 ; idx < DATA_IN_LEN ; idx++)
      {
        out_msg[(idx*2)+0] = 0;
        out_msg[(idx*2)+1] = 0;  
      }
      for (int i = 0 ; i < DATA_LEN ; i++)
        Serial.write(out_msg[i]);

      
    }
    else
    {
  
      if(ch == 'a')
        servo_val = 64; 
      if(ch == 'r')
        servo_val = 58;
      if(ch == 'R')
        servo_val = 0;
      if(ch  == 'c')
        servo_val = 70;
      if(ch  == 'C')
        servo_val = 120;
  
     servo1.write(servo_val);
  
  
  
       // Muestreo
      // read sensor
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    
      for (int idx = 0 ; idx < DATA_IN_LEN ; idx++)
        datos[idx] = Wire.read()<<8|Wire.read();
    
      
      // Encode
      for (int idx = 0 ; idx < DATA_IN_LEN ; idx++)
      {
        out_msg[(idx*2)+0] = char((datos[idx] >> 8) & 0xFF);
        out_msg[(idx*2)+1] = char((datos[idx] ) & 0xFF);  
      }
      
      // Send
      for (int i = 0 ; i < DATA_LEN ; i++)
        Serial.write(out_msg[i]);
    
    }
  
  

    

  }
 
  // Controlo el stop
  val = digitalRead(loopPin);
  if (last_val != val)
  {
    last_val = val;
    if (!val)
    {
      kill = true;

    }
  }



    
  

}
