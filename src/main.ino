#include <Arduino.h>
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can Motor_1_Motor_2_ENle debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "The_Turd_Breaker"
#define REMOTEXY_WIFI_PASSWORD "123456789"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ACCESS_PASSWORD "123456789"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 86 bytes
  { 255,5,0,0,0,79,0,19,0,0,0,0,28,2,106,200,200,84,1,1,
  5,0,1,20,32,24,24,124,36,19,19,0,12,31,0,1,49,95,45,45,
  156,36,19,19,0,12,31,0,4,60,160,42,14,44,46,59,16,176,12,26,
  1,49,91,45,45,13,16,19,19,0,12,31,0,1,246,43,45,45,13,43,
  19,19,0,12,31,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t button_lft; // =1 if button pressed, else =0
  uint8_t button_rgt; // =1 if button pressed, else =0
  int8_t slider_centre; // from -100 to 100
  uint8_t button_fwd; // =1 if button pressed, else =0
  uint8_t button_bwd; // =1 if button pressed, else =0
  
    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0
  
} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define Motor_1_EN  2  // D4
#define Motor_1_in1 4  // D2
#define Motor_1_in2 5  // D1

#define Motor_2_EN  14  // D5
#define Motor_2_in1 12  // D6
#define Motor_2_in2 13  // D7

#define Lift_EN 2 
#define Servo_in1 18   //D0
#define Servo_in2 19   //D8

#define Micro_Adjustment 50
#define MAX_SPEED 255 
#define LIFT_MOTOR_SPEED 170


void setup() {
  RemoteXY_Init (); 
  init_pinout(); 
  Serial.begin(115200);
}

void loop() { 
  RemoteXY_Handler ();
  Handler(RemoteXY.button_fwd, RemoteXY.button_bwd, RemoteXY.button_lft, RemoteXY.button_rgt, RemoteXY.slider_centre);
}

void init_pinout() {
  // motor 1
  pinMode(Motor_1_EN,  OUTPUT);
  pinMode(Motor_1_in1, OUTPUT);
  pinMode(Motor_1_in2, OUTPUT);

  // motor 2
  pinMode(Motor_2_EN,  OUTPUT);
  pinMode(Motor_2_in1, OUTPUT);
  pinMode(Motor_2_in2, OUTPUT);
  pinMode(Servo_in1, OUTPUT);
  pinMode(Servo_in2, OUTPUT);

}


/////////////////////////////////////////////
//             The Driver                  //
/////////////////////////////////////////////

void Handler(int fwd, int bwd, int lft, int rgt, int ctr_val) {
  if (fwd && lft) {
      arcLeft(MAX_SPEED);
  } else if (fwd && rgt) {
      arcRight(MAX_SPEED);
  } else if (bwd && lft) {
      arcLeftBackward(MAX_SPEED);
  } else if (bwd && rgt) {
      arcRightBackward(MAX_SPEED);
  } else if (fwd) {
      forward(MAX_SPEED);
  } else if (bwd) {
      backward(MAX_SPEED);
  } else if (lft) {
      left(MAX_SPEED);
  } else if (rgt) {
      right(MAX_SPEED);
  } else {
      stopMotors();
  }
  

//********** Lift Slider **********//
  if(!ctr_val){
    stop_lift_motor();     // Lift Motor stops
  }else if(ctr_val > 0){
    cw();                  // Lift Motor lifts

  }else if(ctr_val < 0){
    ccw();                 // Lift Motor puts down

  }else {
    stop_lift_motor();     // Lift Motor stops
  }
}


/////////////////////////////////////////////
//        Motor controller Funtions        //
/////////////////////////////////////////////

// High-level movement functions
void forward(int speed) {
  forward_motor1(speed);
  forward_motor2(speed);
  Serial.println("Moving Forward");
}

void backward(int speed) {
  backward_motor1(speed);
  backward_motor2(speed);
  Serial.println("Moving Backward");
}
void right(int speed) {
  backward_motor1(speed);
  forward_motor2(speed);
  Serial.println("Turning Right");
}

void left(int speed) {
  forward_motor1(speed);
  backward_motor2(speed);
  Serial.println("Turning Left");
}

void arcLeft(int speed) {
  forward_motor1(speed / 3);
  forward_motor2(speed);
  Serial.println("Arc Left Forward");
}

void arcRight(int speed) {
  forward_motor1(speed);
  forward_motor2(speed / 3);
  Serial.println("Arc Right Forward");
}

void arcLeftBackward(int speed) {
  backward_motor1(speed / 3);
  backward_motor2(speed);
  Serial.println("Arc Left Backward");
}

void arcRightBackward(int speed) {
  backward_motor1(speed);
  backward_motor2(speed / 3);
  Serial.println("Arc Right Backward");
}

// Low Level Motor Control Function
void forward_motor1(int speed) {
  analogWrite(Motor_1_EN, speed);   
  digitalWrite(Motor_1_in1, LOW);    
  digitalWrite(Motor_1_in2, HIGH);
}

void backward_motor1(int speed) {
  analogWrite(Motor_1_EN, speed);    
  digitalWrite(Motor_1_in1, HIGH);   
  digitalWrite(Motor_1_in2, LOW);
}

void forward_motor2(int speed) {
  analogWrite(Motor_2_EN, speed);    
  digitalWrite(Motor_2_in1, LOW);    
  digitalWrite(Motor_2_in2, HIGH);
}

void backward_motor2(int speed) {
  analogWrite(Motor_2_EN, speed);    
  digitalWrite(Motor_2_in1, HIGH);   
  digitalWrite(Motor_2_in2, LOW);
}

void stopMotors() {
  digitalWrite(Motor_1_in1, LOW);
  digitalWrite(Motor_1_in2, LOW);
  analogWrite(Motor_1_EN, 0);

  digitalWrite(Motor_2_in1, LOW);
  digitalWrite(Motor_2_in2, LOW);
  analogWrite(Motor_2_EN, 0);
}

/////////////////////////////////////////////
//           LIft Motor Funtions           //
/////////////////////////////////////////////

void lift_up() {
  
  digitalWrite(Servo_in1, HIGH);
  digitalWrite(Servo_in2, LOW);  

}

void put_down() {

  digitalWrite(Servo_in1, LOW);
  digitalWrite(Servo_in2, HIGH);  

}


void stop_lift_motor() {
  digitalWrite(Servo_in1, LOW);
  digitalWrite(Servo_in2, LOW);
}

void cw() {
   // Set PWM speed for motor 2
  analogWrite(Lift_EN, LIFT_MOTOR_SPEED);
  digitalWrite(Servo_in1, LOW);    
  digitalWrite(Servo_in2, HIGH);
}

void ccw() {
    // Set PWM speed for motor 2
  analogWrite(Lift_EN, LIFT_MOTOR_SPEED);
  digitalWrite(Servo_in1, HIGH);   
  digitalWrite(Servo_in2, LOW);
}


