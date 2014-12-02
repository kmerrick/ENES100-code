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
const float FIRST_LOCATION_X = .500;
const float FIRST_LOCATION_Y = .550;
const float SEARCH_DISTANCE = .250;
const float FIRST_LOCATION_TOLERANCE = .050;
const float ROCK_LOCATION_TOLERANCE = .100;
const int MARKER_ID = 169;
const int servoPin = 2;
const int ultrasonicPin = 10;
const int rightMotorPin = 11;
const int leftMotorPin = 12;
const int RH_FORWARD = 3;
const int RH_REVERSE = 4;

//Non-constant global variables
Servo myServo;
boolean rockFound = false;
float rockLocation[2];
float angles[] = {-10, -10};

//enumerated type to signify what phase of the mission the vehicle is in.
enum phase {
  landing,
  navigation,
  acquisition
};

phase current;

void setup() {
  //sets up communication with Serial and software Serial
  sSerial.begin(9600);
  Serial.begin(9600);
  rf.resetServer();
  
  //Sends test message to software serial and then regular serial
  rf.sendMessage("Team Alpha Wolf Squadron has connected!!");
  Serial.println("Setup Complete");
  
  //initializes variables
  myServo.attach(servoPin);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  current = landing;
}

void loop() {
  //Only runs if marker is detected
  if (rf.receiveMarker(&marker, MARKER_ID)) {
    //decides which set of directions to follow
    if (current == landing) {
      phaseOneMovement();
    }
    else if (current == navigation) {
      phaseTwoMovement();
    }
    else if (current == acquisition) {
      phaseThreeAcquisition();
    }
  }
}

//dictates movement to first desired location
void phaseOneMovement() {
  //moves until within a certain distance of destination
  if (distance(marker.x, marker.y, FIRST_LOCATION_X, FIRST_LOCATION_Y) > FIRST_LOCATION_TOLERANCE) {
    //determines angle between OSV and location
    float resultAngle = angle(marker.x, marker.y, FIRST_LOCATION_X, FIRST_LOCATION_Y);
    Serial.print("Desired Angle is: "); Serial.println(resultAngle);
    //Moves to position
    moveTo(circleMod(resultAngle));
    Serial.println(marker.theta);
  }
  else {
    //Switches phases once within a certain distance
    current = navigation;
  }
}

//dictates movement to location of rock
void phaseTwoMovement() {
  //Moves to within 250cm of location
  if (distance(marker.x, marker.y, ROCK_LOCATION_X, ROCK_LOCATION_Y) > ROCK_LOCATION_TOLERANCE) {
    //determines angle between OSV and rock location
    float resultAngle = angle(marker.x, marker.y, ROCK_LOCATION_X, ROCK_LOCATION_Y);
    Serial.print("Desired Angle is: "); Serial.println(resultAngle);
    //Moves to rock location
    moveTo(circleMod(resultAngle));
    Serial.println(marker.theta);
  }
  else {
    //Switches phases once within 250cm
    current = acquisition;
  }
}

//Dictates acquisiton of rock
void phaseThreeAcquisition() {
  //detects if this is the first run through of the acquisiton phase
  if (angles[0] < 0 || angles[1] < 0) {
    //initializes angles
    angles[0] = tangent(marker.x, marker.y);
    angles[1] = angles[0];
  }
  //Code for detecting rock
  if (!rockFound) {
    float turnDegrees = PI/10;
    turnRight(turnDegrees);
    long rockDistance = ultrasonicDistance();
    const long bufferDistance = .200;
    if (rockDistance < bufferDistance) {
     rockFound == true;
     //Store location of rock EXPERIMENTAL
     float xDist = rockDistance*cos(marker.theta);
     float yDist = rockDistance*sin(marker.theta);
     rockLocation[0] = marker.x + xDist;
     rockLocation[1] = marker.y + yDist; 
    }
    angles[1] = angles[1] + turnDegrees;
    //if angles[0] and angles[1] differ by pi, then the entire area has been swept and no rock found. Initiate search grid patten
  }
  else {
    //lower magnet
    //pick up rock
    //leave with rock
  }
}

//returns linear distance between two points
float distance(float currX, float currY, float otherX, float otherY) {
  float xComponent = pow(currX-otherX, 2);
  float yComponent = pow(currY-otherY, 2);
  return pow(xComponent + yComponent, .5);
}

//gives angle between two points
float angle(float currX, float currY, float desiredX, float desiredY) {
  float xComponent =desiredX - currX;
  float yComponent = desiredY - currY;
  return circleMod(atan2(yComponent, xComponent));
}

//Gives angle tangent to the circle
float tangent(float currX, float currY) {
  float xComponent = ROCK_LOCATION_X - currX;
  float yComponent = ROCK_LOCATION_Y - currY;
  float tangentAngle = atan2(yComponent, xComponent);
  return circleMod(tangentAngle + RIGHT_ANGLE);
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

//Turns turnDegrees degrees. Negative degrees indicate left turns
void turnTo(float turnDegrees) {
  if (turnDegrees < 0) turnLeft(abs(turnDegrees));
  else turnRight(turnDegrees);
}

//turns right 90 degrees
void turnRight() {
  Serial.print("FIRST METHOD: turning right"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("turning right");
  turnRight(RIGHT_ANGLE);
}

//turns right at a specified intensity
void turnRight(float turnDegrees) {
  Serial.print("SECOND METHOD: turning right "); Serial.print(turnDegrees); Serial.println(" degrees.");
  int leftMotorSpeed = map(turnDegrees, 0, STRAIGHT_ANGLE, 255, 0);
  int rightMotorSpeed = 255;
  analogWrite(leftMotorPin, leftMotorSpeed);
  analogWrite(rightMotorPin, rightMotorSpeed);
  int delayAmount = 100;
  delay(delayAmount);
}
  
  //turns left at 90 degrees
void turnLeft() {
  Serial.print("FIRST METHOD: turning left"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("turning left");
  turnLeft(RIGHT_ANGLE);
}

  //turns left at turnDegrees intensity
void turnLeft(float turnDegrees) {
  Serial.print("SECOND METHOD: turning left "); Serial.print(turnDegrees); Serial.println(" degrees.");
  int rightMotorSpeed = map(turnDegrees, 0, STRAIGHT_ANGLE, 255, 0);
  int leftMotorSpeed = 255;
  analogWrite(leftMotorPin, leftMotorSpeed);
  analogWrite(rightMotorPin, rightMotorSpeed);
  int delayAmount = 100;
  delay(delayAmount);
  analogWrite(leftMotorPin, 0);
  analogWrite(rightMotorPin, 0);
}

void moveSlightLeftForward() {
  Serial.print("moving forward and slightly to the left"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("moving forward and slightly to the left");
  int rightMotorTurn = 220;
  int leftMotorTurn = 250;
  analogWrite(leftMotorPin, leftMotorTurn);
  analogWrite(rightMotorPin, rightMotorTurn);
  int delayAmount = 100;
  delay(delayAmount);
  analogWrite(leftMotorPin, 0);
  analogWrite(rightMotorPin, 0);
}

void moveSlightRightForward() {
  Serial.print("moving forward and slightly to the right"); Serial.println(circleMod(marker.theta)); Serial.print(marker.x); Serial.print(" "); Serial.println(marker.y);
  rf.sendMessage("moving forward and slightly to the right");
  int rightMotorTurn = 250;
  int leftMotorTurn = 230;
  analogWrite(leftMotorPin, leftMotorTurn);
  analogWrite(rightMotorPin, rightMotorTurn);
  int delayAmount = 100;
  delay(delayAmount);
  analogWrite(leftMotorPin, 0);
  analogWrite(rightMotorPin, 0);
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

//code taken from http://arduino.cc/en/Tutorial/ping
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long ultrasonicDistance() {
  long duration = 0;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(ultrasonicPin, OUTPUT);
  digitalWrite(ultrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultrasonicPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ultrasonicPin, INPUT);
  duration = pulseIn(ultrasonicPin, HIGH);

  // convert the time into a distance
  //distance is returned in millimeters
  return microsecondsToCentimeters(duration)/10;
}

//TODO write method to turn forward and backward
void turnRightForward() {

}

void turnRightBackward() {

}

//Code I might need later
/*
if (distance(marker.x, marker.y, ROCK_LOCATION_X, ROCK_LOCATION_Y) > .100) {
      //moveTo(circleMod(angle(marker.x, marker.y)));
      float resultAngle = decideAngle(marker.x, marker.y);
      Serial.print("Zone angle is: ");
      Serial.println(resultAngle);
      moveTo(circleMod(resultAngle));
      Serial.println(marker.theta);
    }
    else {
    //TODO write specific navigation code
    }
  }
*/

