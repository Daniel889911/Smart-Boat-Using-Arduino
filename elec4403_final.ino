//Links: 
//Ultrasonic Sensor Guide, Created by Rui Santos: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
//Compass repository, Created by David E Grayson: https://github.com/pololu/lsm303-arduino

#include <Wire.h>
#include <LSM303.h>

//Compass variables
LSM303 compass;
float og_dir; //original direction+desired direction after turn
float dir_thres=10; //allowed angle deviation from the original direction

//Distance Sensor variables
int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long duration, cm;
long dist_thres=40; //allowed distance from wall before reaction

//Set motor output pins
void set_motors() {DDRD=0b11000000;};

//Set compass variable and settings
void set_compass() 
{
  Wire.begin();
  bool success=compass.init();
  while (!success)
  {
    Serial.println("Big oof on the compass, check the wiring");
    delay(500);
  }
  compass.enableDefault();
  //Calibration values, run the 'Calibration' example in the library to obtain these:
  //min: {  +144,    +30,    +90}    max: {  +569,   +467,   +202}
  //min: {  +270,    -59,   +916}    max: {  +592,   +317,  +1038}
  //min: {   +50,   -385,   -159}    max: {  +975,   +589,  +1061}

  compass.m_min = (LSM303::vector<int16_t>){  +270,    -59,   +916};
  compass.m_max = (LSM303::vector<int16_t>){  +592,   +317,  +1038};
};

//Set the pins for the distance sensor
void set_distance() 
{
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  
};

//Check the current direction
float check_dir() 
{
  compass.read();
  float heading = compass.heading();
  float fixedxDegrees;
  //Map degrees to be between -180 to 180 from the original 0-360 range
  fixedxDegrees=map(heading*100,0,36000,-18000,18000)/100;
  return fixedxDegrees;
};

//Adjust the tilt of the boat if needed, returns true if tilted too far, false otherwise
bool adjust_dir(float thres, float original) 
{
  float current_dir=check_dir();
  
  //Original and current direction outputs - Debug statements
//  Serial.print("Original: ");
//  Serial.print(original);
//  Serial.print(" Current: ");
//  Serial.print(current_dir);
    
  float upper=original+thres;
  float lower=original-thres;
  //Account for overflow/underflow of angle
  if (upper>180) {upper=-180+(upper-180);};
  if (lower<-180){lower= 180+(lower+180);};
  
  //Threshold outputs - Debug statements
//  Serial.print(" Lower: ");
//  Serial.print(lower);
//  Serial.print(" Upper: ");
//  Serial.print(upper);

  if (current_dir>upper)
  {
    //Insert motor control
    Serial.print(" Upper threshold exceeded");
    motor(1,1,8,10); //l, r, l%, r%
    return true;
  }
  else if (current_dir<lower)
  {
    //Insert motor control
    Serial.print(" Lower threshold exceeded");
    motor(1,1,10,8);  //l, r, l%, r%
    return true;
  }  
  return false;
};

//Check the current distance sensor measurement
long check_dist() 
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  //Recorded distance in cm - Debug statements
//  Serial.print(" ");
//  Serial.print(cm);
//  Serial.print("cm");
//  Serial.println();  
  return cm;
};

//Check for an existence of an object within the threshold
bool check_wall(long thres) 
{
  long cur_dist=check_dist();
  if (cur_dist<thres) {return true;}
  else {return false;}
};

//Motor control function, _m set each one to on/off, _s controls the relative speed
void motor(bool left_m, bool right_m, int left_s, int right_s) 
{
    //if in doubt: millis(); for timing the pulses
    int wave=10;//Total time per on/off cycle
    //Left
    if (left_m) 
    {
      PORTD=0b10000000;
      delay(left_s);
      PORTD=0b00000000;
      delay(wave-left_s);
    }     
    else PORTD=0b00000000;
    
    //Right
    if (right_m) 
    {
      PORTD=0b01000000;
      delay(right_s);
      PORTD=0b00000000;
      delay(wave-right_s);
    }
    else PORTD=0b00000000;
};

void setup()
{
  Serial.begin(9600);
  delay(1000); //Delay between turning the boat on and it starting operation, leeway for adjusting the original direction
  set_motors();
  set_compass();
  og_dir=check_dir();
  set_distance();
}

bool wall1=false;  //turn
bool wall2=false;  //switch off 
bool turned=false; //turn done
bool adjust=false; //adjust direction

void loop()
{
  bool adjust=adjust_dir(dir_thres, og_dir);
  if (!wall1) 
  {
    //go until wall1
    if (!adjust) {motor(1,1,9,9);}
    //Debug print for adjusting flag
    //else {Serial.println(" Adjusting");}  
    wall1=check_wall(dist_thres);
  }
  else 
  {
    //turn - record new direction
    if (!turned) {
      //Debug print for turning flag
      //Serial.println(" Turning");
      og_dir=og_dir-180;
      //Correct for overflow/underflow
      if (og_dir>180) {og_dir=-180+(og_dir-180);}
      else if (og_dir<-180){og_dir= 180+(og_dir+180);};
      //Turn until done
      int turn_thres=5;
      float upper_turn=og_dir+turn_thres;
      float lower_turn=og_dir-turn_thres;
      //Account for overflow/underflow of angle
      if (upper_turn>180) {upper_turn=-180+(upper_turn-180);};
      if (lower_turn<-180){lower_turn= 180+(lower_turn+180);};
      float c_d=check_dir();

      while(!(c_d>lower_turn && c_d<upper_turn)) 
      {
        //Debug print for entering the turn while loop
        //Serial.println("Still turning");
        motor(1,0,10,0);
        c_d=check_dir();
      };
      turned=true;
    };
    
    //Debug print for turned flag
    //Serial.println(" Finished turning");
    //go untill wall2
    if (!wall2) 
    {
      if (!adjust) {motor(1,1,9,9);}  
      //Debug print for adjusting flag
      //else {Serial.println(" Adjusting2");}  
      wall2=check_wall(dist_thres);
    }
    //stop
    else motor(0,0,0,0);
  }
  //Serial.println();
  //delay(10);
}
