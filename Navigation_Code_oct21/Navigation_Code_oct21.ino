enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

void setup() {
  sSerial.begin(9600);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void moveTo(int desiredAngle) {
  if(rf.receiveMarker(&marker, 5)) {
    int phi = (marker.theta - desiredAngle) mod 360    //Rotates angle
    if (phi < 0) phi = phi + 360;                //Eliminates negative angles
    if (phi <= 20) moveSlightRightForward();    //Turns and moves if angle is within
    else if (phi >= 340) moveSlightLeftForward();    //20 degrees of desired angle 
    else if (phi <= 180) turnRight();
    else if (phi < 340) turnLeft();        //Turns if the angle is not within tolerances
    else moveForward();        //Moves forward if vehicle is facing correct angle
  }
}

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

