#include <enes100.h>
#include <enes100_marker.h>
#include <enes100_rf_client.h>
#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial sSerial(8, 9);

enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//constants to represent angles
const float RIGHT = 0;
const float LEFT = PI;
const float UP = PI/2;
const float DOWN = 3*PI/2;
//other constants
const float FIELD_LENGTH = 4000;
const float FIELD_HEIGHT = 2000;
const float ROCK_LOCATION_X = 3400;
const float ROCK_LOCATION_Y = 550;
const float SEARCH_DISTANCE = 250;
const float RIGHT_ANGLE = PI/2;
const float STRAIGHT_ANGLE = PI;
const float CIRCLE = 2*PI;

void setup() {
  sSerial.begin(9600);
  Serial.begin(9600);
  rf.resetServer();
  rf.sendMessage("Team Alpha Wolf Squadron has connected");
  Serial.println("Setup Complete");
}

void loop() {
  delay(100);
  if (rf.receiveMarker(&marker, 169)) {
    boolean generalNavigation = distance(marker.x, marker.y, ROCK_LOCATION_X, ROCK_LOCATION_Y) > SEARCH_DISTANCE;
    if (generalNavigation) {
      moveTo(decideAngle(marker.x, marker.y));
    }
    else {
      //TODO sweep out area;
      float startingAngle = tangent(marker.x, marker.y);
      turnTo(startingAngle);
    }
  }
}

float tangent(float currX, float currY) {
  float xComponent = ROCK_LOCATION_X - currX;
  float yComponent = ROCK_LOCATION_Y - currY;
  float tangentAngle = atan2(yComponent, xComponent);
  return tangentAngle + RIGHT_ANGLE;
}   
  
void moveTo(float desiredAngle) {
  if(rf.receiveMarker(&marker, 5)) {
    float phi = fmod(marker.theta - desiredAngle, CIRCLE);    //Rotates angle
    if (phi < 0) phi = phi + CIRCLE;                //Eliminates negative angles
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

void turnTo(float startingAngle) {
  float phi = fmod(marker.theta - startingAngle, CIRCLE);    //Rotates angle
  if (phi < 0) phi = phi + CIRCLE;                         //Eliminates negative angles
  if (phi < 180) turnLeft();
  else turnRight();
}

//Algorithm to decide angle to travel at.
float decideAngle(float x, float y) {
  float desiredAngle = 0;
  float n = 0;
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

float distance(float currX, float currY, float x, float y) {
  float horizontalDistance = pow((currX - x), 2);
  float verticalDistance = pow((currY - y), 2);
  return pow(horizontalDistance + verticalDistance, .5);
}


//Implement these later
void turnRight() {
  Serial.print("turning right"); Serial.println(fmod(marker.theta + CIRCLE, CIRCLE));
}

void turnLeft() {
  Serial.print("turning left"); Serial.println(fmod(marker.theta + CIRCLE, CIRCLE));
}

void moveSlightLeftForward() {
  Serial.print("moving forward and slightly to the left"); Serial.println(fmod(marker.theta + CIRCLE, CIRCLE));
}

void moveSlightRightForward() {
  Serial.print("moving forward and slightly to the right"); Serial.println(fmod(marker.theta + CIRCLE, CIRCLE));
}

void moveForward() {
  Serial.print("moving forward"); Serial.println(fmod(marker.theta + CIRCLE, CIRCLE));
}

