#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Servo.h>

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

//Address set for the I2C slave arduino reading the encoders
#define SLAVE_ADDRESS_1 4

#define FR_PWM_PIN 6
#define FL_PWM_PIN 5
#define BR_PWM_PIN 10
#define BL_PWM_PIN 9
Servo FR_motor, FL_motor, BR_motor, BL_motor;  

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

void writePWM(int pwm_1,int pwm_2,int pwm_3,int pwm_4){
  //Remap the PWM signals from -255, 255 to 0, 180 for the servo library
  //Sum multiplied by a negative to account for motor directions
  pwm_1 = (int)map(pwm_1, -255, 255, 0, 180);
  pwm_2 = (int)map(pwm_2, -255, 255, 0, 180);
  pwm_3 = (int)map(pwm_3, -255, 255, 0, 180);
  pwm_4 = (int)map(-1*pwm_4, -255, 255, 0, 180);

  FL_motor.write(pwm_1);
  FR_motor.write(pwm_2);
  BR_motor.write(pwm_3);
  BL_motor.write(pwm_4);
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
  
  //Attach the PWM pins to control each motor using Serial Write.
  FR_motor.attach(FR_PWM_PIN);
  FL_motor.attach(FL_PWM_PIN);
  BR_motor.attach(BR_PWM_PIN);
  BL_motor.attach(BL_PWM_PIN);
  
}

void loop() {
  
  char startDataWrite = 0xDA;
  char endDataWrite = 0xAD;
  readWritePWM();
  
  //ask to read IMU data
  lsm.read(); 

  // Get a new sensor event 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  
  // Get encoder info
  shaftFrequencies motors12 = getShaftFrequencies(SLAVE_ADDRESS_1);
  
  
  // Transmit serial package
      
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
