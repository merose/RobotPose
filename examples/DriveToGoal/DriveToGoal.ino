#include <Servo.h>
#include <AnalogScanner.h>
#include <AnalogEncoder.h>
#include <RobotPose.h>

const float GOALX = 1.0;
const float GOALY = 1.0;

AnalogScanner scanner;
AnalogEncoder leftEncoder;
AnalogEncoder rightEncoder;

Servo leftServo;
Servo rightServo;
RobotPose pose(0.069, 0.187, 30);

void setup() {
  Serial.begin(38400);
  
  int scanOrder[] = {A0, A1};
  scanner.setScanOrder(sizeof(scanOrder) / sizeof(scanOrder[0]), scanOrder);
  scanner.setCallback(A0, updateEncoder);
  scanner.setCallback(A1, updateEncoder);
  scanner.beginScanning();
  delay(100);

  leftServo.attach(A4);
  rightServo.attach(A5);

  setSpeed(90, 90);
  delay(100);
}

void updateEncoder(int index, int pin, int value) {
  if (index == 0) {
    leftEncoder.update(value);
  } else {
    rightEncoder.update(value);
  }
}

void loop() {
  pose.updatePose(leftEncoder.getTicks(), rightEncoder.getTicks());
  
  // ADD YOUR CODE HERE
  
  // Stop the robot if we've reached the goal.
  float distanceToGoal = 0.0; // Change this to calculate the actual distance.
  if (distanceToGoal < 0.1) {
    setSpeed(0, 0);
    delay(100);
    leftServo.detach();
    rightServo.detach();
    for (;;) {
      // do nothing, forever
    }
  }
  
  // Otherwise, turn the robot toward the goal.
  float goalHeading = 0.0; // Change this to calculate the heading.
  float headingError = 0.0; // Change this to calculate the actual error.
  int motorDifference = 0; // Change this to calculate the motor speed diff.
  
  // END ADD YOUR CODE HERE

  Serial.print("lticks=");
  Serial.print(leftEncoder.getTicks());
  Serial.print("  rticks=");
  Serial.print(rightEncoder.getTicks());
  Serial.print("  x: ");
  Serial.print(pose.getX());
  Serial.print("  y: ");
  Serial.print(pose.getY());
  Serial.print("  goal heading: ");
  Serial.print(goalHeading);
  Serial.print("  actual heading: ");
  Serial.print(pose.getHeading());
  Serial.print("  heading error: ");
  Serial.print(headingError);
  Serial.print("  motor difference: ");
  Serial.println(motorDifference);
  
  setSpeed(20 - motorDifference, 20 + motorDifference);
}

void setSpeed(int leftSpeed, int rightSpeed) {
  leftServo.write(90 + leftSpeed);
  rightServo.write(90 - rightSpeed);
}

