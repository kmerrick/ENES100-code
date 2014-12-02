#include <enes100.h>
#include <enes100_marker.h>
#include <enes100_rf_client.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>

const int sSerialPin1 = 8;
const int sSerialPin2 = 9;
SoftwareSerial sSerial(sSerialPin1, sSerialPin2);

enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

//constants to represent angles
const float RIGHT = 0;
const float LEFT = PI;
const float UP = PI/2;
const float DOWN = 3*PI/2;
const float RIGHT_ANGLE = PI/2;
const float STRAIGHT_ANGLE = PI;
const float CIRCLE = 2*PI;
const float MAXIMUM_ANGLE_DIFFERENCE = PI/9;
//other constants
const float FIELD_LENGTH = 4.0;
const float FIELD_HEIGHT = 2.0;
const float ROCK_LOCATION_X = 3.400;
const float ROCK_LOCATION_Y = .550;
const float SEARCH_DISTANCE = .250;
const int MARKER_ID = 169;
const int servoPin = 2;

Servo myServo;

void setup() {
  sSerial.begin(9600);
  Serial.begin(9600);
  rf.resetServer();
  rf.sendMessage("Team Alpha Wolf Squadron has connected!!");
  Serial.println("Setup Complete");
  myServo.attach(servoPin);
}

float startAngle = -10;
 
void loop() {
  if (rf.receiveMarker(&marker, MARKER_ID)) {
    if (distance(marker.x, marker.y, ROCK_LOCATION_X, ROCK_LOCATION_Y) > .100) {
      //moveTo(circleMod(angle(marker.x, marker.y)));
      float resultAngle = decideAngle(marker.x, marker.y);
      Serial.print("Zone angle is: ");
      Serial.println(resultAngle);
      moveTo(circleMod(resultAngle));
      Serial.println(marker.theta);
    }
    else {
       //if (startAngle < 0
    }
  }
}

float angle(float currX, float currY) {
  float xComponent = ROCK_LOCATION_X - currX;
  float yComponent = ROCK_LOCATION_Y - currY;
  return atan2(yComponent, xComponent);
}

float tangent(float currX, float currY) {
  float xComponent = ROCK_LOCATION_X - currX;
  float yComponent = ROCK_LOCATION_Y - currY;
  float tangentAngle = atan2(yComponent, xComponent);
  return tangentAngle + RIGHT_ANGLE;
}   
  
void moveTo(float desiredAngle) {
  if(rf.receiveMarker(&marker, MARKER_ID)) {
    float phi = circleMod(marker.theta - desiredAngle);    //Rotates angle
    if (phi < 0) phi = phi + CIRCLE;                //Eliminates negative angles //
    if (phi <= MAXIMUM_ANGLE_DIFFERENCE) { 
      moveSlightRightForward();    //Turns and moves if angle is within
    }
    else if (phi >= CIRCLE - MAXIMUM_ANGLE_DIFFERENCE) { 
      moveSlightLeftForward();    //20 degrees of desired angle 
    }
    else if (phi <= STRAIGHT_ANGLE) {
      turnRight();
    }
    else if (phi < CIRCLE - MAXIMUM_ANGLE_DIFFERENCE) {
      turnLeft();        //Turns if the angle is not within tolerances
    }
    else moveForward();        //Moves forward if vehicle is facing correct angle
  }
}

void turnTo(float startingAngle) {
  float phi = circleMod(startingAngle);    //Rotates angle
  if (phi < 0) phi = phi + CIRCLE;                         //Eliminates negative angles
  if (phi < STRAIGHT_ANGLE) turnLeft();
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
    desiredAngle = desiredAngle + RIGHT;
    n++;
  }
  //Buffer 2
  if (y <= FIELD_HEIGHT / 12) {
    desiredAngle = desiredAngle + UP;
    n++;
  }
  //buffer 3
  if (x >= FIELD_LENGTH * 23 / 24) {
    desiredAngle = desiredAngle + LEFT;
    n++;
  }
  //buffer 4
  if (y >= FIELD_HEIGHT * 11 / 12) {
    desiredAngle = desiredAngle + DOWN;
    n++;
  }
  //Zone 1
  
  //THIS ZONE Is WRONG
  else if (x > FIELD_LENGTH / 24 && y > FIELD_HEIGHT / 3 && x <= FIELD_LENGTH * 5 / 24) { 
    desiredAngle += DOWN;
    n++;
  }
  //Zone 2
  else if (x > FIELD_LENGTH / 24 && y > FIELD_HEIGHT / 12 && x < FIELD_LENGTH * 23 / 24 && y <= FIELD_HEIGHT / 3) {
    desiredAngle += RIGHT;
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
  Serial.print("turning right"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("turning right");
}

void turnLeft() {
  Serial.print("turning left"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("turning left");
}

void moveSlightLeftForward() {
  Serial.print("moving forward and slightly to the left"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("moving forward and slightly to the left");
}

void moveSlightRightForward() {
  Serial.print("moving forward and slightly to the right"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("moving forward and slightly to the right");
}

void moveForward() {
  Serial.print("moving forward"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("moving forward");
}

float circleMod(float angle) {
  angle = fmod(angle, CIRCLE);
  if (angle < 0) angle = angle + CIRCLE;
  return angle;
}

