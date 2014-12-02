#include <enes100.h>
#include <enes100_marker.h>
#include <enes100_rf_client.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial sSerial(8, 9);

enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//constants to represent angles
const int RIGHT = 0;
const int LEFT = 180;
const int UP = 90;
const int DOWN = 270;
//other constants
const int FIELD_LENGTH = 4000;
const int FIELD_HEIGHT = 2000;


void setup() {
  sSerial.begin(9600);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}


void moveTo(int desiredAngle) {
  if(rf.receiveMarker(&marker, 5)) {
    int phi = fmod(marker.theta - desiredAngle, 360);    //Rotates angle
    if (phi < 0) phi = phi + 360;                //Eliminates negative angles
    if (phi <= 20) { 
      moveSlightRightForward();    //Turns and moves if angle is within
    }
    else if (phi >= 340) { 
      moveSlightLeftForward();    //20 degrees of desired angle 
    }
    else if (phi <= 180) {
      turnRight();
    }
    else if (phi < 340) {
      turnLeft();        //Turns if the angle is not within tolerances
    }
    else moveForward();        //Moves forward if vehicle is facing correct angle
  }
}

//Algorithm to decide angle to travel at.
int decideAngle(int x, int y) {
  int desiredAngle = 0;
  int n = 0;
  //First four conditionals cover "buffer zones." If the Vehicle is within 166
  //units of the wall, it tries to navigate away.
  
  //Buffer 1
  if (x <= FIELD_LENGTH / 24) {
    desiredAngle += RIGHT;
    n++;
  }
  //Buffer 2
  if (y <= FIELD_HEIGHT / 12) {
    desiredAngle += UP;
    n++;
  }
  //buffer 3
  if (x >= FIELD_LENGTH * 23 / 24) {
    desiredAngle += LEFT;
    n++;
  }
  //buffer 4
  if (y >= FIELD_HEIGHT * 11 / 12) {
    desiredAngle += DOWN;
    n++;
  }
  //Zone 1
  else if (x > FIELD_LENGTH / 24 && y > FIELD_HEIGHT / 12 && x < FIELD_LENGTH * 23 / 24 && y <= FIELD_HEIGHT / 3) {
    desiredAngle += RIGHT;
    n++;
  }
  //Zone 2
  else if (x > FIELD_LENGTH / 24 && y > FIELD_HEIGHT / 3 && x <= FIELD_LENGTH * 5 / 24) { 
    desiredAngle += DOWN;
    n++;
  }
  //Zone 3
  else if (x > FIELD_LENGTH * 5 / 24 && x <= FIELD_LENGTH / 4 && y > FIELD_HEIGHT / 3) {
    desiredAngle += LEFT;
    n++;
  }
  //Zone 4
  else if ( x > FIELD_LENGTH / 4 && y > FIELD_HEIGHT / 3) {
    desiredAngle += DOWN;
    n++;
  }
  //returns averge angle of zones
  return (desiredAngle / n);
}  

int distance(int currX, int currY, int x, int y) {
  int horizontalDistance = pow((currX - x), 2);
  int verticalDistance = pow((currY - y), 2);
  return pow(horizontalDistance + verticalDistance, .5);
}


//Implement these later
void turnRight() {
  Serial.println("turning right");
}

void turnLeft() {
  Serial.println("turning left");
}

void moveSlightLeftForward() {
  Serial.println("moving forward and slightly to the left");
}

void moveSlightRightForward() {
  Serial.println("moving forward and slightly to the right");
}

void moveForward() {
  Serial.println("moving forward");
}

