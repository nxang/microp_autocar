#include <PID_v1.h>
#include <TimerEvent.h>

#define BUTTON1_PIN A4
#define BUTTON digitalRead(BUTTON1_PIN)

#define IRCL_LED 4 //Centre left IR LED pin
#define IRCR_LED 3 //Centre Right IR LED pin
#define IRL_LED 7 // left ir led pin
#define IRR_LED 2 //right ir led pin
#define LMOTOR_LED 5 //left motor led pin
#define RMOTOR_LED 6 //right motor led pin

#define EN1 9
#define EN2 10

#define IN1 8
#define IN2 11
#define IN3 12
#define IN4 13

#define IRSL A3  // side left IR
#define IRCL A1  // centre left IR
#define IRCR A2  // centre right IR
#define IRSR A0  // Side right IR

/*
you can check the current value and tune it based on the value you get 
Ex:
the value range you read is from 100 - 600
you can set the value as 300/200 etc
*/
#define Left_IR_Target 300        // tune this value 
#define Right_IR_Target 300       // tune this value
#define Side_Left_IR_Target 300   // tune this value
#define Side_Right_IR_Target 300  // tune this value

TimerEvent timer;
TimerEvent status;

double SideLeftInput, SideRightInput;
double LeftSetpoint, LeftInput, LeftOutput;
double RightSetpoint, RightInput, RightOutput;

double L_Kp = 4.3, L_Ki = 0, L_Kd = 0;
double R_Kp = 4.3, R_Ki = 0, R_Kd = 0;

PID LEFT_PID(&LeftInput, &LeftOutput, &LeftSetpoint, L_Kp, L_Ki, L_Kd, REVERSE);
PID RIGHT_PID(&RightInput, &RightOutput, &RightSetpoint, R_Kp, R_Ki, R_Kd, REVERSE);

uint8_t flag = 0;

void Turn_Right() {
  /* Turn right
  -  It will only turn right when the right most IR sensors sensed the black line
  -  The Left wheel will be going at full speed and the right wheel will stop moving in order to turn right.
  -  The turn right motion will only stops when the centre left IR sensed the black line ( which means that it already finish the turning motion)
  - the last line will keep on checking the centre left IR input so that it knows when to exit the condition
  */
  while (LeftInput < Left_IR_Target) {
    digitalWrite(LMOTOR_LED, HIGH);   
    digitalWrite(RMOTOR_LED, LOW);  
    digitalWrite(IN1, LOW);  
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 255);
    LeftInput = analogRead(IRCL); //keep updating the left centre IR input
  }
}

void Turn_Left() { //same as turn right just opposite direction and ir sensors
  while (RightInput < Right_IR_Target) {
    digitalWrite(LMOTOR_LED, LOW);   
    digitalWrite(RMOTOR_LED, HIGH); 

    digitalWrite(IN1, LOW);  
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(EN1, 255);
    analogWrite(EN2, 0);
    RightInput = analogRead(IRCR); //Keep updating the right centre IR Input
  }

}


void Straight() { 
  digitalWrite(LMOTOR_LED, HIGH);   
  digitalWrite(RMOTOR_LED, HIGH); 

  analogWrite(EN1, 150 + (LeftOutput / 255) * 100); //adjust the moving straight motion with basespeed + pid, to ensure it moves straight
  analogWrite(EN2, 150 + (RightOutput / 255) * 100);
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Boost() { //to rush off from the black line
  digitalWrite(LMOTOR_LED, HIGH);  
  digitalWrite(RMOTOR_LED, HIGH); 

  analogWrite(EN1, 255);
  analogWrite(EN2, 255);
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(100);
}



void timerCallback() { // loops that determine which IR have scanned the black line 
  if (LeftInput > Left_IR_Target) {  // centre left ir detected
    digitalWrite(IRCL_LED, HIGH);
    Straight();
  } else {
    digitalWrite(IRCL_LED, LOW);
  }

  // if the value of the RIGHT IR sensor is greater than the target value
  if (RightInput > Right_IR_Target) {  // centre right ir  detected
    digitalWrite(IRCR_LED, HIGH);
    Straight();
  } else {
    digitalWrite(IRCR_LED, LOW);
  }

  if (LeftInput > Left_IR_Target && RightInput > Right_IR_Target && (SideRightInput > Side_Right_IR_Target || SideLeftInput > Side_Left_IR_Target)) {  //side left or side right ir + centre two ir detected then we know need to stop
    while (!BUTTON) { //stuck until u press the button 
      Serial.println(SideLeftInput);
      digitalWrite(LMOTOR_LED, LOW); 
      digitalWrite(RMOTOR_LED, LOW);
      digitalWrite(IN1, HIGH);        // all the IN high to emergency break
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      analogWrite(EN1, 200);
      analogWrite(EN2, 200);          // since all the IN high so dont care about pwm
      digitalWrite(IRCL_LED, HIGH);
      digitalWrite(IRCR_LED, HIGH);
      digitalWrite(IRL_LED, HIGH);
      digitalWrite(IRR_LED, HIGH);
    };
    Boost();
  }

  // // if the value of the LEFT IR sensor is greater than the target value
  else if (SideLeftInput > Side_Left_IR_Target) {  // side left ir  detected
    digitalWrite(IRL_LED, HIGH);
    Turn_Left();
  }
  // if the value of the RIGHT IR sensor is greater than the target value
  else if (SideRightInput > Side_Right_IR_Target) {  // side right ir  detected
    digitalWrite(IRR_LED, HIGH);
    Turn_Right();
  } else {
    digitalWrite(IRL_LED, LOW);
    digitalWrite(IRR_LED, LOW);
  }
}

void StatusCallback() { //keep checking the ir status when not inside the loop
  LeftInput = analogRead(IRCL);
  RightInput = analogRead(IRCR);
  SideLeftInput = analogRead(IRSL);
  SideRightInput = analogRead(IRSR);

  LEFT_PID.Compute();
  RIGHT_PID.Compute();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  timer.set(2, timerCallback); //setup call back every 2ms to update everything
  status.set(2, StatusCallback);

  // setup the pin for me
  pinMode(BUTTON1_PIN, INPUT);

  pinMode(IRCL_LED, OUTPUT);
  pinMode(IRCR_LED, OUTPUT);
  pinMode(IRL_LED, OUTPUT);
  pinMode(IRR_LED, OUTPUT);
  pinMode(LMOTOR_LED, OUTPUT);
  pinMode(RMOTOR_LED, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  LeftSetpoint = Left_IR_Target;
  RightSetpoint = Right_IR_Target;
  // timer is used in here because im using pid so that i have a fix sample time if not unable to get a stable and fix sampling time
  LEFT_PID.SetMode(AUTOMATIC);
  LEFT_PID.SetSampleTime(2);

  RIGHT_PID.SetMode(AUTOMATIC);
  RIGHT_PID.SetSampleTime(2);

  while(!BUTTON); //will only start when button is pressed 
  /*
  when you call BUTTON - it will  check the status of the button(look into the define)
  */
}

void loop() {
  timer.update(); //timer update
  status.update();
}
