#include <Wire.h>

//Update this to be different values for each slave
#define SLAVE_ADDRESS 4

//The Nano Interrupt pins the encoder is connected to
#define ENCODER_A_1 2
#define ENCODER_A_2 3
#define ENCODER_B_1 4
#define ENCODER_B_2 5

//Gear ratio of internal motor shaft to external shaft.
const float GEAR_RATIO = 3.0;

typedef union{
  float f;
  int i;
  char c[4];
} floatToIntBytes;

typedef struct{
  float shaft1Freq;
  float shaft2Freq;
} shaftFrequencies;


 //Keep timing for frequency estimation.
 unsigned long prevTime = 0, currTime = 0;

//The HIGH or LOW state of each encoder channel pulse.
volatile bool encoderA1State, encoderA2State;
volatile bool encoderB1State, encoderB2State;

//Counts for each time the interrupt occurs for a reading on encoder A1.
volatile long currEncoder1Count = 0.0, currEncoder2Count = 0.0;
long lastEncoder1Count = 0.0, lastEncoder2Count = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_A_2, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  pinMode(ENCODER_B_2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), interruptEncoderA1, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), interruptEncoderA2, CHANGE);
  
  //Initialize the I2C communication
  Wire.begin(SLAVE_ADDRESS);

  //Send the data of the encoder each time the master device requests
  Wire.onRequest(transmitShaftFrequencies);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //float shaftFreq = getMotorShaftFrequency();
  //serial.println(shaftFreq);
  delay(10);
}

void transmitShaftFrequencies(){
  //Transmit the shaft frequecies of each motor..
  floatToIntBytes fToi1, fToi2;
  shaftFrequencies shaftFreqs;
  shaftFreqs = getMotorShaftFrequencies();
  fToi1.f = shaftFreqs.shaft1Freq;
  fToi2.f = shaftFreqs.shaft2Freq;
  Wire.write(fToi1.c[0]);
  Wire.write(fToi1.c[1]);
  Wire.write(fToi1.c[2]);
  Wire.write(fToi1.c[3]);

  Wire.write(fToi2.c[0]);
  Wire.write(fToi2.c[1]);
  Wire.write(fToi2.c[2]);
  Wire.write(fToi2.c[3]);
}

shaftFrequencies getMotorShaftFrequencies(){

  //Frequencies of encoder and shaft
  //shaftFreq = encoderFreq / (16 * GEAR_RATIO);
  float encoder1Freq, encoder2Freq;
  shaftFrequencies shaftFreqs;
  float dt;
  
  currTime = micros();
  
  dt = currTime - prevTime;
  encoder1Freq = ((float)(currEncoder1Count - lastEncoder1Count) / (dt/1000000.0));
  encoder2Freq = ((float)(currEncoder2Count - lastEncoder2Count) / (dt/1000000.0));
  shaftFreqs.shaft1Freq = (encoder1Freq) / (16 * GEAR_RATIO);
  shaftFreqs.shaft2Freq = (encoder2Freq) / (16 * GEAR_RATIO);
  lastEncoder1Count = currEncoder1Count;
  lastEncoder2Count = currEncoder2Count;

  prevTime = micros();
  return(shaftFreqs);
  
}

void interruptEncoderA1(){
  encoderA1State = digitalRead(ENCODER_A_1);
  encoderB1State = digitalRead(ENCODER_B_1);
  
  if((encoderA1State == 1)){
    if(encoderB1State == 1){
      currEncoder1Count++;
    }
    else{
      currEncoder1Count--;
    }
  }

  else{
    if(encoderB1State == 1){
      currEncoder1Count--;
    }
    else{
      currEncoder1Count++;
    }
  }

}

void interruptEncoderA2(){
  encoderA2State = digitalRead(ENCODER_A_2);
  encoderB2State = digitalRead(ENCODER_B_2);
  
  if((encoderA2State == 1)){
    if(encoderB2State == 1){
      currEncoder2Count++;
    }
    else{
      currEncoder2Count--;
    }
  }

  else{
    if(encoderB2State == 1){
      currEncoder2Count--;
    }
    else{
      currEncoder2Count++;
    }
  }

}
