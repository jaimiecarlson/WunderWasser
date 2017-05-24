#define ENCpin1_1 28
//J3
#define ENCpin1_2 29
#define ENCpin2_1 32 
//J4
#define ENCpin2_2 33

#include <Encoder.h>

long oldPosition1 = -999;
long newPosition1 = -999;
long oldPosition2 = -999;
long newPosition2 = -999;

Encoder myEnc1(ENCpin1_1, ENCpin1_2);
Encoder myEnc2(ENCpin2_1, ENCpin2_2);

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the Encoder sensor reading
int rawPos = 0;         // current raw reading from Encoder sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;

int updatedPos1 = 0;     // keeps track of the latest updated value of the Encoder sensor reading
int rawPos1 = 0;         // current raw reading from Encoder sensor
int lastRawPos1 = 0;     // last raw reading from MR sensor
int lastLastRawPos1 = 0; // last last raw reading from MR sensor
int tempOffset1 = 0;
int rawDiff1 = 0;
int lastRawDiff1 = 0;
int rawOffset1 = 0;
int lastRawOffset1 = 0;
//const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
//boolean flipped = false;

// Kinematics variables

double xh = 0;           // position of the handle [m]
double yh= 0;
double lastXh = 0;     //last x position of the handle
double lastYh= 0;
double vhx = 0;         //velocity of the handle
double vhy = 0;
double lastVhx = 0;     //last velocity of the handle
double lastVhy = 0;     //last velocity of the handle
double lastLastVhx = 0; //last last velocity of the handle
double lastLastVhy = 0; //last last velocity of the handle

// Initialize position valiables
long oldOldPosition= -999;

long oldOldPosition1= -999;

// Define kinematic parameters you may need
double rh = 0.0725;   //[m]
double l=0.067; 
double L=0.073;
double d=0.020; 
  
void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);
  while(!SerialUSB);
  SerialUSB.println("Basic Encoder Test");
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
}


void loop() {
  // put your main code here, to run repeatedly:
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
  if (newPosition1 != oldPosition1 || newPosition2 != oldPosition2){
    oldPosition1 = newPosition1;
    oldPosition2 = newPosition2;
    SerialUSB.print(newPosition1); 
    SerialUSB.print("\t"); 
    SerialUSB.println(newPosition2);
  

  //Forward Kinematics

  double ts1= -360.0/13824.0*newPosition2; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = -360.0/13824.0*newPosition1; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos

//       Serial.println("angles");
//            Serial.println(ts1);
//            Serial.println(ts);


  double      th1=ts*3.14159/180.0; 
  double     th2=ts1*3.14159/180.0; 
//       Serial.println("angles(rad)");
//            Serial.println(th1);
//            Serial.println(th2);

  //delay(1000); 
  // Forward Kinematics

  double c1=cos(th1);
  double c2=cos(th2);
  double s1=sin(th1);
  double s2=sin(th2);

  double xA=l*c1;
  double yA=l*s1;
  double xB=d+l*c2;
  double yB=l*s2;
  double R=pow(xA,2) +pow(yA,2);
  double S=pow(xB,2)+pow(yB,2);
  double M=(yA-yB)/(xB-xA);
  double N=0.5*(S-R)/(xB-xA);
  double a=pow(M,2)+1;
  double b=2*(M*N-M*xA-yA);
  double c=pow(N,2)-2*N*xA+R-pow(L,2);
  double Delta=pow(b,2)-4*a*c;
  double y=(-b+sqrt(Delta))/(2*a); 
  double x=M*y+N;
  //phi1=angle((x-l*c1)/L+1i*(y-l*s1)/L);
  //phi2=angle((x-d-l*c2)/L+1i*(y-l*s2)/L);
  double phi1=acos((x-l*c1)/L);
  double phi2=acos((x-d-l*c2)/L);
  double xP=x;
  double yP=y;

  xh=xP; 
  yh=yP; 

  // Step 2.7:
   // xh = rh*(cos(ts*3.14159/180)-cos(ts1*3.14159/180));       // Compute the position of the handle based on ts
   // yh= rh*(sin(ts*3.14159/180)+sin(ts1*3.14159/180));

      SerialUSB.println("Position");
  SerialUSB.print(xh*1000);
  SerialUSB.print("\t");
  SerialUSB.println(yh*1000);
  

  //Serial.println("postions");
   //      Serial.println(xh,6);
   ///     Serial.println(yh);

  //
  // Step 2.8: print xh via serial monitor
   // Serial.println(ts,5);
  // Lab 4 Step 2.3: compute handle velocity
//  vhx = -(.95*.95)*lastLastVhx + 2*.95*lastVhx + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
//  vhy = -(.95*.95)*lastLastVhy + 2*.95*lastVhy + (1-.95)*(1-.95)*(yh-lastYh)/.0001;  // filtered velocity (2nd-order filter)
//  vhx= (lastVhx+2*(xh-lastXh)/.0015)/3;
  vhx= (xh-lastXh)/.0015; ///.0015;
//  vhy=(lastVhy+2* (yh-lastYh)/.0015)/3;
    vhy=(yh-lastYh)/.0015; 
  lastXh = xh;
  lastYh = yh;
  lastLastVhx = lastVhx;
  lastLastVhy = lastVhy;
  lastVhx = vhx;
  lastVhy = vhy;

  }
  
 
  delay(100);
  
}
