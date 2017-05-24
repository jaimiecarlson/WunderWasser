//-----------------------------------------------------------------
//Tania Morimoto, Stanford University
//09.03.13
//Use key strokes to switch between different virtual environments
//-----------------------------------------------------------------

//Includes
#include <TimerOne.h>
#include <math.h>

//Pin Declares 
int pwmAPin = 5;
int dirAPin = 8;
int sensorPosPin = A2;

//Variables
int pos = 0;
int actualPos[3] = {0,0,0};  //keep track of 3 positions where 0 index = lastlast, 1 index = last, 2 = current
int flipCount = 0;           //keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int actualDiff = 0;
int lastactualDiff = 0;
int actualOffset = 0;
int lastactualOffset = 0;
const int flipThresh = 700;  

double x = 0;
double x_cm = 0;
double lastx = 0;
double vel = 0;
double lastVel = 0;
double theta_s = 0;    //sector pulley angle
double m = 0.00019;    //linear curve fit (theta_s = m*pos + b) to ME sensor data
double b = -0.095;
double lx = 0.065659;  //[m]
double rp = 0.004191;  //[m]
double rs = 0.073152;  //[m]

char virtualEnvCase; //for switching between different environments
double force = 0;
double Tm = 0;
double duty = 0;
unsigned int output = 0;

unsigned long time;
int counter = 0;
boolean flipped = false;

//for FSR data
int fsrPin = A0; //force-sensitive resistor pin
int fsrValue = 0;
double e = 2.718;
double fsrForce;
//-----------------------------------
//Initialize
//-----------------------------------
void setup() 
{
  // Set Up Serial
  Serial.begin(9600);
  
  // Set PWM frequency 
  //TCCR0B = _BV(CS00);
  //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  setPwmFrequency(pwmAPin,1); 
  
  // Input Pins
  pinMode(sensorPosPin, INPUT); // mag sensor input 
  
  // Output Pins
  pinMode(pwmAPin, OUTPUT);  // PWM for A
  pinMode(dirAPin, OUTPUT);  // dir for A
  
  // Init Motor 
  analogWrite(pwmAPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirAPin, LOW);  // set direction 
  
  // Init Position
  actualPos[0] = analogRead(sensorPosPin);
  actualPos[1] = analogRead(sensorPosPin);
  
}

//-----------------------------------
// Main Loop
//-----------------------------------
void loop()
{
  // Check if something was sent to serial and respond 
    if (Serial.available() > 0) 
    {
      char inByte = Serial.read();
      virtualEnvCase = inByte; 
  
      // Print what the enviroment is
      switch (virtualEnvCase) 
      {
        case 'w': // (virtual wall)
          Serial.println("virtual wall"); 
          break;
        case 's': // (spring) 
          Serial.println("spring"); 
          break;
        case 'd': // (damping) 
          Serial.println("damping"); 
          break;
        case 't': // (texture)
          Serial.println("texture");
          break;
        default: // (default)
          Serial.println("nothing"); 
          break; 
      }
    } // end if (Serial.available() > 0)
  
  //Get position of device
  actualPos[2] = analogRead(sensorPosPin);  //current position

  //calculate differences between MR sensor readings
  actualDiff = actualPos[2] - actualPos[1];      //difference btwn current pos and last pos
  lastactualDiff = actualPos[2] - actualPos[0];  //difference btwn current pos and last last pos
  actualOffset = abs(actualDiff);
  lastactualOffset = abs(lastactualDiff);
  
  //--------Update variables-----------
  actualPos[0] = actualPos[1];
  actualPos[1] = actualPos[2];
  
  //keep track of flips over 180 degrees
  if((lastactualOffset > flipThresh) && (!flipped))   //try different values here
  {
    if(lastactualDiff > 0)//(actualDiff > 0)
    {
      flipCount--;   //cw rotation for how I've been playing with MR sensor 
    } 
    else //if(actualDiff<0)
    {
      flipCount++;   //ccw rotation for how I've been playing with MR sensor
    }
    if(actualOffset > flipThresh)
    {
      pos = actualPos[2] + flipCount*actualOffset;      //if data was good and actual offset is above threshold
      tempOffset = actualOffset;
    }else    //in this case there was a blip in the data and we want to use lastactualOffset
    {
      pos = actualPos[2] + flipCount*lastactualOffset;    //update the pos value to account for any flips over 180deg *****changed this so that it is based on the ACTUAL OFFSET!!******
      tempOffset = lastactualOffset;
    }
    flipped = true;    //set boolean so that the next time through the loop won't trigger a flip
  }
  else
  {
    pos = actualPos[2] + flipCount*tempOffset;    //need to update pos based on what most recent offset is 
    flipped = false;
  }

  //--------Calculations----------
  theta_s = m*pos + b;                   //based on linear-fit (theta_s is in radians)
  x = lx*theta_s;                      //x-position of sector pulley in [m]
  x_cm = x*100;                    //x-position in cm for debugging purposes
  vel = (.95)*lastVel + (0.05)*(x-lastx)/(.0003); //divide by what value...? depends on frequency?
  
  //--------Parameters-------------
  double kp = 100.0;    //virtual wall stiffness [N/m]  
  double ks = 8.0;      //spring stiffness  [N/m]
  double B = 1.5;
  
  //-------*Environments*----------
  switch (virtualEnvCase)
  {
    case 'w': //virtual wall
      if(x < 0)          
        {force = -kp*x;}   //if inside the wall create a force that pushes back out of the wall
      else
        {force = 0;}       //if outside the wall then no force
      break;
    case 's': // (spring)
      force = -ks*x;      
      break;
    case 'd': //(damping)
      force = -B*vel;
      break;
    case 't': // (texture)
      force = 0.09*sin(2000*x);
      break;
    default:
      //do nothing
      break; 
  }  
  
  //--------Force output------------
  //Determine correct direction
  if(force < 0)  //[N]
  {
    digitalWrite(dirAPin, HIGH);
  } else
  {
    digitalWrite(dirAPin, LOW);
  } 
  //Determine duty cycle based on force & write
  force = abs(force);
  Tm = rp/rs * lx * force;         // corresponding motor torque based on force
  duty = sqrt(Tm/.0053);      // map motor torque to PWM duty cycle
  if (duty > .80) 
    {
      duty = .80;                   // output a maximum of 80% duty cycle
    }    
  if (duty < 0)
    { 
      duty = 0;
    }  
  output = (int)(duty* 255);         // convert duty cycle to output signal
  analogWrite(pwmAPin,output);     // output the signal

  time = millis();    //returns the number of ms since program started running
  counter++;

//------Update variables------
lastx = x;
lastVel = vel;

//------FSR data collection-------
fsrValue = analogRead(fsrPin);
fsrForce = 0.7991*pow(e,0.0033*fsrValue) - 0.7991;
}

// --------------------------
// Set PWM Freq 
// --------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
