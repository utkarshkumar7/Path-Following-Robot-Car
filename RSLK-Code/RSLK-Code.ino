
#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;//left direction pin
const int left_pwm_pin=40;//left pwm pin

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;//right direction pin
const int right_pwm_pin=39;//right pwm pin
//sensor min and max
const int mins[8] = {668,621,597,621,574,620,643,643};
const int maxes[8] = {2500,2500,2500,2160,2444,2500,2500,2500};
double prevError= 0;
//controller constants
double kp;
double kd;
int timesAllBlack=0;//how many times in a row all sensors have given Highs
int readingsSinceStart = 0;//used to track start delay
int startDelay = 20;
bool hasTurned = false;//keeps track if the car has turned or not already
///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

//Sets each wheel to be awake and sets their directions to forward
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
    digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);



  
 ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
}

void loop() {
  // put your main code here, to run repeatedly: 

 int leftSpd = 40;
 int rightSpd = 40;



//runs slower at start so that it can find path

if(readingsSinceStart <startDelay){//start delay before the pathsensing starts
  readingsSinceStart++;
  int leftSpd = 20;
  int rightSpd = 20;
}


 kp = 0.01;//proportional controller constant
 kd = 0.04;//derivative controller constant
  ECE3_read_IR(sensorValues);//gets sensor values
double fusionVal= 0;
double sum[8]; 
for(int i =0; i<8;i++){//Normalizes each of the sensor values
  sum[i] =  1000*(sensorValues[i]-mins[i])/(maxes[i]-mins[i]);
}
fusionVal = sum[0]*(-15)+sum[1]*(-14)+ sum[2]*(-12)+sum[3]*(-8)+ sum[4]*8+sum[5]*12+ sum[6]*14+sum[7]*15;//calculating fusion value
//Serial.println(fusionVal);

double df = fusionVal-prevError;//calculating df
if(readingsSinceStart >=startDelay){//if its past the start delay
//implements proportional and derivative control
if(fusionVal<0){//turn left
  leftSpd-=kp*fusionVal;
}
else{//turn right
  rightSpd+=kp*fusionVal;
}

if(df<0){//slow down right
  rightSpd+=kd*df;
}
else{//slows down left
  leftSpd-=kd*df;
}

//boosts both speeds if the car is on track
if(fusionVal>-2000 && fusionVal<2000&& df>-1000 && df< 1000){
  leftSpd += 40;//40 for donut 50 for straight
  rightSpd +=40;
}

}


//checks for all high values
bool allBlack= true;
for(int i =0; i<8; i++){
  if(sensorValues[i]<maxes[i]){
    allBlack = false;
  }
}
if(allBlack){//if all high values increments timesAllBlack
timesAllBlack++;
}
else{
 timesAllBlack=0;
}
//Serial.println(timesAllBlack);
//delay(1200);

if(timesAllBlack==2){//if all sensors give HIGHs twice in a row
  if(hasTurned){//if the car has already turned around once, set speed to 0 and end the program since car reaches end of course
    analogWrite(left_pwm_pin,0);
analogWrite(right_pwm_pin,0);
    exit(0);
  }
  hasTurned = true;
  //Car does a donut to turn around
analogWrite(left_pwm_pin,0);
analogWrite(right_pwm_pin,0);
delay(10);
digitalWrite(left_dir_pin,HIGH);
analogWrite(left_pwm_pin,240);  
analogWrite(right_pwm_pin,240);
delay(250);
digitalWrite(left_dir_pin,LOW);
analogWrite(left_pwm_pin,0);
analogWrite(right_pwm_pin,0);
delay(10);
  }
  else{//if not turning around or ending, set the cars wheel speeds
analogWrite(left_pwm_pin,leftSpd);
analogWrite(right_pwm_pin,rightSpd);
  }
prevError = fusionVal;//get previous error to be used to calculate next df
  }
