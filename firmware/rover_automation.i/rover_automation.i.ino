#include <Servo.h>

// =======================================================
// Hardware Configuration
// =======================================================

//pins designated to the front right TT motor 
const int motor_1_pin_1 = A0;
const int motor_1_pin_2 = A1;
const int motor_1_pin_EN = 2;

//pins designated to the front left TT motor 
const int motor_2_pin_1 = A2;
const int motor_2_pin_2 = A3;
const int motor_2_pin_EN = 3;

//pins designated to the back right TT motor 
const int motor_3_pin_1 = A4;
const int motor_3_pin_2 = A5;
const int motor_3_pin_EN = 5;

//pins designated to the back left TT motor 
const int motor_4_pin_1 = A6;
const int motor_4_pin_2 = A7;
const int motor_4_pin_EN = 6;

//pins designated to the front ultrasonic sensor 
const int trig_pin_1 = 23;
const int echo_pin_1 = 22;

//pins designated to the rear left ultrasonic sensor 
const int trig_pin_2 = 25;
const int echo_pin_2 = 24;

//pins designated to the rear right ultrasonic sensor 
const int trig_pin_3 = 27;
const int echo_pin_3 = 26;

// =======================================================
// State and Kinematics
// =======================================================

//monitor state of the rover
enum RoverState {DRIVE, TURN, STOP};
RoverState rover_state = DRIVE;

//constants for deriving the x, y and angular coordinates for the rover
const float Pi = 3.1415926535; 
const float wheel_seperation = 13; 
const float wheel_diameter = 6.5; 
const float turn_circumference = wheel_seperation * Pi;
const float wheel_circumference = wheel_diameter * Pi;
const float rpm = 100; 
const float wheel_velocity = (rpm / 60) * wheel_circumference; 

//time for the servo motor to physically reach the commanded position
const int interval = 50; 

//variables 
float wheel_angular_velocity = 0;
float angle = 0; 
float x = 0; 
float y = 0; 
float r = 0; 

unsigned long time; 
unsigned long turn_time; 

//monitors the previous time the rover was at the particular state
unsigned long travel_timestamp = 0; 
unsigned long turn_timestamp = 0; 
unsigned long regular_timestamp = 0; 

// =======================================================
// Initialization
// =======================================================

//monitors round-trip travel time of the produced sound wave 
long duration = 0; 

//distance detected by each of the ultrasonic sensors 
float distance_1 = 0; 
float distance_2 = 0; 
float distance_3 = 0; 

//the sweep increments for the servo motors
int increment_1 = 10; 
int increment_2 = 10; 

//dictionary to store the surrounding distance data during fulfillment phase
float angle_data[360]; 

//front ultrasonic sensor 
Servo servo1;
//rear left ultrasonic sensor 
Servo servo2;
//rear right ultrasonic sensor 
Servo servo3;

int servo_position_1 = 0; 
int servo_position_2 = 0; 

void setup() {
  // put your setup code here, to run once:
  pinMode(motor_1_pin_1, OUTPUT);
  pinMode(motor_1_pin_2, OUTPUT);

  pinMode(motor_2_pin_1, OUTPUT);
  pinMode(motor_2_pin_2, OUTPUT);

  pinMode(motor_1_pin_EN, OUTPUT);
  pinMode(motor_2_pin_EN, OUTPUT);
    
  pinMode(motor_3_pin_1, OUTPUT);
  pinMode(motor_3_pin_2, OUTPUT);

  pinMode(motor_4_pin_1, OUTPUT);
  pinMode(motor_4_pin_2, OUTPUT);

  pinMode(motor_3_pin_EN, OUTPUT);
  pinMode(motor_4_pin_EN, OUTPUT);


  pinMode(trig_pin_1, OUTPUT);
  pinMode(echo_pin_1, INPUT);

  pinMode(trig_pin_2, OUTPUT);
  pinMode(echo_pin_2, INPUT);

  pinMode(trig_pin_3, OUTPUT);
  pinMode(echo_pin_3, INPUT);

  //assign PWM signal pin of each servo to a digital pin
  servo1.attach(28);
  servo2.attach(29);
  servo3.attach(30);

  //initialize initial position of the servo motor to align with the y-axis 
  servo_position_1 = 90; 
  servo1.write(90);

  Serial.begin(9600);
}

// =======================================================
// Main Control Loop
// =======================================================

void loop() {
  //put your main code here, to run repeatedly:
  time = millis(); 

  //PWM adjusts the speed of the TT motors as a ratio
  //as the PWM ranges from 0 to 255, a half value of 127 allows the RPM of the TT motors to be equal to 100 (200 divided by 2)
  analogWrite(motor_1_pin_EN, 127);
  analogWrite(motor_2_pin_EN, 127);
  analogWrite(motor_3_pin_EN, 127);
  analogWrite(motor_4_pin_EN, 127);

  // reservation condition
  if (distance_1 < 25) {
    //initialize coordinates
    r =((time - travel_timestamp) / 1000)* wheel_velocity; 
    y = r * sin((angle - 90) * (Pi/180));
    x = r * cos((angle - 90) * (Pi/180));

    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);

    //brake the rover
    rover_state = STOP;
    digitalWrite(motor_1_pin_1, HIGH);
    digitalWrite(motor_1_pin_2, HIGH);

    digitalWrite(motor_2_pin_1, HIGH);
    digitalWrite(motor_2_pin_2, HIGH);

    digitalWrite(motor_3_pin_1, HIGH);
    digitalWrite(motor_3_pin_2, HIGH);

    digitalWrite(motor_4_pin_1, HIGH);
    digitalWrite(motor_4_pin_2, HIGH);

    //fulfillment step
    //rotate the rear servo motors and log the data 
    for(int servo_position_2 = 0; servo_position_2 < 180; servo_position_2+=10){
      servo2.write(servo_position_2);
      servo3.write(179 - servo_position_2);
      distance_2 = getDistance(trig_pin_2, echo_pin_2, duration);
      distance_3 = getDistance(trig_pin_3, echo_pin_3, duration);
      angle_data[servo_position_2] = distance_2;
      angle_data[(servo_position_2 + 180) % 360] = distance_3;
      delay(10);
    }
      
    rover_state = TURN;
    float target_angle = getMaxAngle(angle_data);
    Serial.print("target_angle: ");
    Serial.println(target_angle);
    float current_angle = angle;

    turn_timestamp = millis();
    //rotate the rover until its degree of turn is off by 5 degrees from the target angle 
    while (abs(target_angle - current_angle) > 5.0) {    
      time = millis(); 

      //drive the right-side wheels forward
      //drive the left-side wheels reverse
      digitalWrite(motor_1_pin_1, LOW);
      digitalWrite(motor_1_pin_2, HIGH);

      digitalWrite(motor_2_pin_1, HIGH);
      digitalWrite(motor_2_pin_2, LOW);

      digitalWrite(motor_3_pin_1, LOW);
      digitalWrite(motor_3_pin_2, HIGH);

      digitalWrite(motor_4_pin_1, HIGH);
      digitalWrite(motor_4_pin_2, LOW);
      current_angle = getAngle(time - turn_timestamp);
      if (current_angle < 360) {
        Serial.print("current_angle: ");
        Serial.println(current_angle);
      }
    }

  //update angle and time stamp for the turn event
    angle = current_angle;
    travel_timestamp = time;
  } else {
    //drive all wheels forward
    rover_state = DRIVE;
    digitalWrite(motor_1_pin_1, LOW);
    digitalWrite(motor_1_pin_2, HIGH);

    digitalWrite(motor_2_pin_1, LOW);
    digitalWrite(motor_2_pin_2, HIGH);

    digitalWrite(motor_3_pin_1, LOW);
    digitalWrite(motor_3_pin_2, HIGH);

    digitalWrite(motor_4_pin_1, LOW);
    digitalWrite(motor_4_pin_2, HIGH);
  }

  //rotate the front servo motors to sweep between 45 degrees and 135 degrees 
  for(int servo_position_1 = 45; servo_position_1 < 135; servo_position_1+=10){
    servo1.write(servo_position_1);
    distance_1 = getDistance(trig_pin_1, echo_pin_1, duration);
    delay(10);
  }

  for(int servo_position_1 = 135; servo_position_1 > 45; servo_position_1-=10){
    servo1.write(servo_position_1);
    distance_1 = getDistance(trig_pin_1, echo_pin_1, duration);
    delay(10);
  }
}

// =======================================================
// Sensing Utilities
// =======================================================

//retreive the distance to the nearest foreign object using an ultrasonic sensor
float getDistance(int trig_pin, int echo_pin, long duration) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds (2);
  // Sets the trig_pin on HIGH state for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds (10);
  digitalWrite(trig_pin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo_pin, HIGH);
  // Calculating the distance in (cm)
  return duration * 0.034 / 2;
}

// =======================================================
// Motion and Geometry Utilities
// =======================================================

float getAngle(float time) {
  return ((2 * wheel_velocity) / wheel_seperation) * (time / 1000) * (180/Pi); 
}

//retreive the angle thatrecorded the maximum distance to the nearest foreign object 
float getMaxAngle(float angle_data[]) {
  float angle = 0; 
  float maximum_distance = 0; 
  for (int i = 0; i < 360; i++) {
    if (maximum_distance < angle_data[i]) {
      maximum_distance = angle_data[i];
      angle = i; 
    }
  }
  return angle; 
}