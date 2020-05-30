#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

#define SLAVE_ADDRESS_1 4
#define SLAVE_ADDRESS_2 5

#define FR_IN1 10
#define FR_IN2 11
#define FR_EN 12

#define FL_IN1 14
#define FL_IN2 15
#define FL_EN 13

#define BR_IN1 8
#define BR_IN2 9
#define BR_EN 10

#define BL_IN1 3
#define BL_IN2 4
#define BL_EN 5  

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

char ball = 0;
typedef union
{
    float f;
    int i;
    char c[4];
} floatIntBytes;

typedef struct
{
    float shaft1Freq;
    float shaft2Freq;
} shaftFrequencies;


void setupIMU() 
{
    // 1.) Set the accelerometer range
   lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G); // 4 8 16

    // 2.) Set the magnetometer sensitivity
   lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS); // 8 12 16

    // 3.) Setup the gyroscope
   lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // 500 2000
}

void writePWM(int pwm1,int pwm2,int pwm3,int pwm4){
  if(pwm1 >= 0){
    digitalWrite(BL_IN1, HIGH);
    digitalWrite(BL_IN2, LOW);
    analogWrite(BL_EN, pwm1);
    }
  else{
    digitalWrite(BL_IN1, LOW);
    digitalWrite(BL_IN2, HIGH);
    analogWrite(BL_EN, pwm1);
    }
  if(pwm2 >= 0){
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    analogWrite(FL_EN, pwm2);
    }
  else{
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
    analogWrite(FL_EN, pwm2);
    }
  if(pwm3 >= 0){
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    analogWrite(FR_EN, pwm3);
    }
  else{
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    analogWrite(FR_EN, pwm3);
    }
  if(pwm4 >= 0){
    digitalWrite(BR_IN1, HIGH);
    digitalWrite(BR_IN2, LOW);
    analogWrite(BR_EN, pwm4);
    }
  else{
    digitalWrite(BR_IN1, LOW);
    digitalWrite(BR_IN2, HIGH);
    analogWrite(BR_EN, pwm4);
    }
}

void readWritePWM(){
  /* Reads PWM values from TX2 and writes them to each motor using writePWM() */
  floatIntBytes pwm1, pwm2, pwm3, pwm4;
  char startRead = 0xDB;
  char endRead= 0xBD;
  char checkStart;
  char checkEnd;
 
  if(Serial.available() > 0){
    checkStart = Serial.read();
    if(checkStart == startRead){
      pwm1.c[0] = Serial.read();
      pwm1.c[1] = Serial.read();
      pwm1.c[2] = Serial.read();
      pwm1.c[3] = Serial.read();   

      pwm2.c[0] = Serial.read();
      pwm2.c[1] = Serial.read();
      pwm2.c[2] = Serial.read();
      pwm2.c[3] = Serial.read();      
      
      pwm3.c[0] = Serial.read();
      pwm3.c[1] = Serial.read();
      pwm3.c[2] = Serial.read();
      pwm3.c[3] = Serial.read();      
      
      pwm4.c[0] = Serial.read();
      pwm4.c[1] = Serial.read();
      pwm4.c[2] = Serial.read();
      pwm4.c[3] = Serial.read();      
      checkEnd = Serial.read();
      if(checkEnd == endRead){
        writePWM(pwm1.i,pwm2.i,pwm3.i,pwm4.i);
        ball = 0x10;
        }
      
      }
      else ball = 0x22;

    
    } 
}


shaftFrequencies getShaftFrequencies(int slaveAddress){
  /*
   * Get the shaft frequencies from the two motors.
   */
  shaftFrequencies shaftFreqs;

  Wire.requestFrom(slaveAddress, 8); //Request for bytes of data from encoder.

  int counter = 0;
  floatIntBytes fToi1, fToi2;
  fToi1.i = 0; fToi2.i = 0;
  
  while(Wire.available()){
    if(counter < 4)
      fToi1.c[counter] = Wire.read();
    else
      fToi2.c[counter - 4] = Wire.read();
    counter++;
    if(counter >= 8) break; 
  }

  shaftFreqs.shaft1Freq = fToi1.f;
  shaftFreqs.shaft2Freq = fToi2.f;
  return(shaftFreqs);
}

void setup() {
  
  //Initialize Master I2C.
  Wire.begin(); //No address needed for master.

  //Initialize serial
  Serial.begin(9600);

  Serial.println("LSM9DS1 data read demo");
 
// Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  setupIMU();
}

void loop() {
  
  char startDataWrite = 0xDA;
  char endDataWrite = 0xAD;
  readWritePWM();

  /* ask to read IMU data */
  lsm.read(); 

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  
  /* Get encoder info */
  shaftFrequencies motors12 = getShaftFrequencies(SLAVE_ADDRESS_1);
  
  
  /* Transmit serial package */
      
  Serial.write(startDataWrite); //1 byte
  
  Serial.write(loadFloat(motors12.shaft1Freq).c[0]); //4 bytes
  Serial.write(loadFloat(motors12.shaft1Freq).c[1]); 
  Serial.write(loadFloat(motors12.shaft1Freq).c[2]); 
  Serial.write(loadFloat(motors12.shaft1Freq).c[3]);

  Serial.write(loadFloat(motors12.shaft2Freq).c[0]); //4 bytes
  Serial.write(loadFloat(motors12.shaft2Freq).c[1]); 
  Serial.write(loadFloat(motors12.shaft2Freq).c[2]); 
  Serial.write(loadFloat(motors12.shaft2Freq).c[3]); 

  Serial.write(loadFloat(a.acceleration.x).c[0]); // 1 bytes  
  Serial.write(loadFloat(a.acceleration.x).c[1]); // 1 byte
  Serial.write(loadFloat(a.acceleration.x).c[2]); // 1 byte
  Serial.write(loadFloat(a.acceleration.x).c[3]); // 1 byte

  Serial.write(loadFloat(a.acceleration.y).c[0]); // 1 byte
  Serial.write(loadFloat(a.acceleration.y).c[1]); // 1 byte
  Serial.write(loadFloat(a.acceleration.y).c[2]); // 1 byte
  Serial.write(loadFloat(a.acceleration.y).c[3]); // 1 byte

  Serial.write(loadFloat(a.acceleration.z).c[0]); // 1 byte
  Serial.write(loadFloat(a.acceleration.z).c[1]); // 1 byte
  Serial.write(loadFloat(a.acceleration.z).c[2]); // 1 byte
  Serial.write(loadFloat(a.acceleration.z).c[3]); // 1 byte
  
  Serial.write(loadFloat(m.magnetic.x).c[0]); // 1    
  Serial.write(loadFloat(m.magnetic.x).c[1]); // 1 
  Serial.write(loadFloat(m.magnetic.x).c[2]); // 1 
  Serial.write(loadFloat(m.magnetic.x).c[3]); // 1 

  Serial.write(loadFloat(m.magnetic.y).c[0]); // 1  
  Serial.write(loadFloat(m.magnetic.y).c[1]); // 1
  Serial.write(loadFloat(m.magnetic.y).c[2]); // 1
  Serial.write(loadFloat(m.magnetic.y).c[3]); // 1

  Serial.write(loadFloat(m.magnetic.z).c[0]); // 1  
  Serial.write(loadFloat(m.magnetic.z).c[1]); // 1
  Serial.write(loadFloat(m.magnetic.z).c[2]); // 1
  Serial.write(loadFloat(m.magnetic.z).c[3]); // 1
  
  Serial.write(loadFloat(g.gyro.x).c[0]); // 1
  Serial.write(loadFloat(g.gyro.x).c[1]); // 1
  Serial.write(loadFloat(g.gyro.x).c[2]); // 1
  Serial.write(loadFloat(g.gyro.x).c[3]); // 1

  Serial.write(loadFloat(g.gyro.y).c[0]); // 1
  Serial.write(loadFloat(g.gyro.y).c[1]); // 1
  Serial.write(loadFloat(g.gyro.y).c[2]); // 1
  Serial.write(loadFloat(g.gyro.y).c[3]); // 1

  Serial.write(loadFloat(g.gyro.z).c[0]); // 1
  Serial.write(loadFloat(g.gyro.z).c[1]); // 1
  Serial.write(loadFloat(g.gyro.z).c[2]); // 1
  Serial.write(loadFloat(g.gyro.z).c[3]); // 1

  
  Serial.write(endDataWrite); // 1 byte

  delay(10);

}

floatIntBytes loadFloat(float val){
  floatIntBytes temp;
  temp.f = val;
  return temp;
  }
