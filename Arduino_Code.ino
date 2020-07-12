#include <Servo.h>
#include <Keypad.h>
#include <math.h>

// Create servo motors objects
Servo servoMotor1;
Servo servoMotor2;
// Keypad configurations
const byte ROWS = 2; 
const byte COLS = 1;
char hexaKeys[ROWS][COLS] = {
  {'A'},
  {'B'},
};
byte rowPins[ROWS] = {11, 10}; 
byte colPins[COLS] = {2}; 
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

// Define the rotaion angles of the motors(joints) in degrees
const int THETA1_Degree = 30;
const int THETA2_Degree = 60;
const int THETA_Degree = THETA1_Degree+THETA2_Degree;
// Convert the angles to radian
double THETA1 = THETA1_Degree*M_PI/180;
double THETA2 = THETA2_Degree*M_PI/180;
double THETA = THETA_Degree*M_PI/180;
// Define the end effector coordinates
const double X = 8.66;;
const double Y = 10;
// Define the length of the arms
const int L1 = 10;
const int L2 = 5;


void setup() {
  
  // Attache servo motors to Arduino pins
  servoMotor1.attach(5);
  servoMotor2.attach(6);  
  // Initialize the serial monitor
  Serial.begin(9600);  
}

void loop(){
  char customKey = customKeypad.getKey();
  
  if (customKey == 'A'){
    Serial.println("Forward Kinematic");
    Forward(L1, L2, THETA1, THETA2);
  }
  else if (customKey == 'B'){
    Serial.println("Inverse Kinematic");
    Inverse(L1, L2, X, Y);
  }
}

void Forward(int L1, int L2, double THETA1, double THETA2){
  // Rotate the servo motors with the specified angles
  servoMotor1.write(THETA1_Degree);
  servoMotor2.write(THETA2_Degree); 
  // Calculate the X coordinate of the End Effector
  double x = L1*cos(THETA1)+L2*cos(THETA1+THETA2);
  // Calculate the Y coordinate of the End Effector
  double y = L1*sin(THETA1)+L2*sin(THETA1+THETA2);
  // Display the location of the End Effector
  Serial.print("X = ");
  Serial.println(x);
  Serial.print("Y = ");
  Serial.println(y);
}

void Inverse(int L1, int L2,double X, double Y){
  // Display the rotation angles of the joints 
  double theta2 = acos((sq(X)+sq(Y)-sq(L1)-sq(L2))/(2*L1*L2));
  double theta1 = THETA-theta2;
  // Convert the angles into degress
  double theta1_degree = theta1*180/M_PI;
  double theta2_degree = theta2*180/M_PI;
  Serial.print("Theta1 = ");
  Serial.println(theta1_degree);
  Serial.print("Theta2 = ");
  Serial.println(theta2_degree);
  // Rotate the servo motors with the resulted angles
  servoMotor1.write(theta1_degree);
  servoMotor2.write(theta2_degree); 
}
