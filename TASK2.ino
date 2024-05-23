/***************************************************
M103Lab6BucketLiftOnly (v1.2)

  Original by D. Ells 27/8/2021
  Revised by B. Surgenor, 25/04/2021

Lift and then lower the bucket once. Start with
bucket resting (at an angle) touching ground, drive
link parallel to the ground.  
*****************************************************/

#include <Servo.h>  // Includes the library
Servo myServoA;  // Makes a servo object to control servo A
Servo myServoB;// Makes a servo object to control servo B
Servo leftWheel;
Servo rightWheel;
// Pin Assignments
const int LSENSOR = A1; // Left Sensor on Analog Pin 1
const int RSENSOR = A2; // Right Sensor on Analog Pin 2
float Lvoltage = analogRead(A1);
float Rvoltage = analogRead(A2);
const boolean PLOT = false;  //true=plot sensor reading; false=serial monitor output.

int MOTOR_R = 3;        // right motor signal pin
int MOTOR_L = 4;        // left motor signal pin

const int stopPulse = 148;  // stop speed for motors (default = 150))
const int delta = 15;       // pulse differential (default = 15)
const int offset = 0;       // offset, slows left wheel (default = 0)

//global variables
int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value

// Pin Assignments
int SHARP = A3;     // Sharp Sensor on Analog Pin 3
int value = 0;
int mv_value = 0;

   

// Pin Assignments
int GRN = 9;            // Green LED Pin
int YLW = 5;            // Yellow LED Pin
int RED = 10;           // Red LED Pin
int BUTTON = 7;         // Pushbutton Pin

int servoPinA = 11;     // Bucket servomotor #1 pin
int myAngleA1 = 155;    // initial angle, bucket lifts off ground if too high
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
int myAngleA2 = 110;     // highest angle (lift), puts almost straight, set to 110
                        //    still bent (i.e. not as high)

// this code is added
int servoPinB= 12;    
int myAngleB1 = 100;    
int posB = myAngleB1;  
int myAngleB2 = 130;  
int flag;  
int wallflag;
                
// Set-up routine
void setup() {

// Set-up LED pins as outputs
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);
 
// Set-up button pin as input
  pinMode(BUTTON, INPUT);

// Set-up servo motors
  myServoA.write(posA);         // Servo A starting position
  myServoA.attach(servoPinA);   // Attaches the servo to the servo object

  //this code is new
  myServoB.write(posB);         // Servo B starting position
  myServoB.attach(servoPinB);   // Attaches the servo to the servo object
  // Initialize line following sensor pins as inputs
  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);

  //initialize motor control pins as servo pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);
               
// Initialize led pins as outputs.
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);
 
// Initialize pins as inputs
  pinMode(BUTTON, INPUT);
  pinMode(SHARP, INPUT);
 
// Initialize serial printout
  Serial.begin(9600);    // default 9600
  turnOnLED(YLW);
  digitalWrite(YLW, LOW);
  digitalWrite(GRN, HIGH);


flag = 1;
wallflag = 1;
  raise();


}


// Main routine
void loop() {

  Serial.println(flag);

         //read the sensor value
      lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);



value = analogRead(SHARP);

    mv_value = map(value,0,1023,0,3300); //convert AtoD count to millivolts
  


 if(lvalue>700 && rvalue>700){
     
      turn90();

      if (wallflag == 0){
        wallflag = 1;
      }

      else if(wallflag == 1){
        wallflag = 0;
      }
  }


else if(mv_value>1000 && rvalue<600 && lvalue < 600 && wallflag == 0){


if(flag == 1){
  
 pickUp();

 


}



else if(flag == 0){
  
dumpOff();

  }

}

else{

lineFollow();

}


}


//********************* Functions (subroutines)*****************


//Toggle an LED on/off
void toggleLED(int colour){
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250);
}

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

void runMotors(int deltaL, int deltaR){

  int pulseL = (stopPulse + deltaL)*10;    //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR)*10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}

void turn180(){
  
  delay(1000);
      runMotors(-13,7);
      delay(500);

  while(lvalue<600) {
           //read the sensor value
      lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);

      runMotors(-13,7);
  }
runMotors(0,0);
        runMotors(-13,7);
        delay(200);

  while(lvalue>600){
        //read the sensor value
      lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);
runMotors(-13,7);
   }

runMotors(0,0);
 

}

void turn90(){


   runMotors(10,-13);
   delay(770);

  runMotors(0,0);
  delay(2000);

while(lvalue < 600){
   
   runMotors(10,-12);

} 

  
 // runMotors(0,0);
  //delay(2000);
  //digitalWrite(GRN, LOW);
    //  digitalWrite(RED, LOW);
     // digitalWrite(YLW, LOW);
      //runMotors(-10,-10);
     // delay(550);
     // runMotors(10,-10);
     // delay(1000);
     // runMotors(0,0);
     // delay(100);

      while(lvalue > 600 || rvalue > 600){

 lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);

      lineFollow();

}
   

}

void raise(){
  delay (2000);           // A couple seconds to stand back
  for (posA = myAngleA1; posA >= myAngleA2; posA--) { // Lift action
    myServoA.write(posA);
    delay(20);
  }
}
void lower(){
  delay(1000);
  for (posA = myAngleA2; posA <= myAngleA1; posA++) {  // Drop action
    myServoA.write(posA);
    delay(20);
  }
}

void lineFollow(){
 
       if(lvalue<600){
         digitalWrite(YLW, HIGH);
      
         runMotors(10, 0);
      }else if(lvalue>600){
        digitalWrite(YLW, LOW);
      };

       if(rvalue<600){
 
         digitalWrite(RED, HIGH);
          runMotors(0, 10);
      }else if(rvalue>600){
        digitalWrite(RED, LOW);
      }else;
 
      if(lvalue<600 && rvalue<600){
     
      digitalWrite(GRN, HIGH);
      digitalWrite(RED, LOW);
      digitalWrite(YLW, LOW);
             runMotors(10, 10);
      }else{
        digitalWrite(GRN, LOW);
      }

}

void dumpbucket(){
  delay (2000);           // A couple seconds to stand back
  for (posB = myAngleB1; posB <= myAngleB2; posB++) { // Lift action
    myServoB.write(posB);
    delay(40);
  }

}

void returnbucketfromDown(){
  delay (2000);           // A couple seconds to stand back
  for (posB = myAngleB2; posB >= myAngleB1; posB--) { // Lift action
    myServoB.write(posB);
    delay(20);
  }
}



void liftbucket(){
  delay (2000);           // A couple seconds to stand back
  for (posB = myAngleB1; posB >= 60; posB--) { // Lift action
    myServoB.write(posB);
    delay(50);
  }
}

void returnbucketfromUp(){
delay (2000);           // A couple seconds to stand back
  for (posB = 60; posB <= myAngleB1; posB++) { // Lift action
    myServoB.write(posB);
    delay(40);
  }

}

void dumpOff(){
       runMotors(0,0);
Serial.println("Ran 2");
  delay(500);
  runMotors(0,0); 
  turn180();

//weight adjustment
  runMotors(-13,7);
   delay(170);
       runMotors(0,0);
  
  delay(500);
   runMotors(-16,-16);
   delay(550);
    runMotors(0,0);
Serial.println("dump");
dumpbucket();
returnbucketfromDown();
Serial.println("Flag is: ");

  runMotors(10,0);
   delay(200);
    runMotors(-10,10);
   delay(120);

while(lvalue > 600 || rvalue > 600){

 lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);

      lineFollow();

}

flag = 1;

}

void pickUp(){

 runMotors(0,0);
  delay(500);
  turn180();

  // adjustment
  runMotors(-13,7);
   delay(320);
       runMotors(0,0);
  delay(400);

  lower();
   runMotors(-16,-16);
   delay(670);
    runMotors(0,0); 
liftbucket();
Serial.println("");
raise();
Serial.println("Flag is: ");

 runMotors(10,0);
   delay(240);
    runMotors(-10,10);
   delay(110);

while(lvalue > 600 || rvalue > 600){

 lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

          //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);

      lineFollow();

}

flag = 0;

}


