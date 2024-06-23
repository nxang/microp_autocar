#include <PID_v1.h>
#include <TimerEvent.h>

#define BUTTON1_PIN A4
#define BUTTON digitalRead(BUTTON1_PIN)

#define IRCL_LED 4
#define IRCR_LED 3
#define IRL_LED 7
#define IRR_LED 2
#define Button_LED 6
#define Power_LED 5

#define EN1 9
#define EN2 10

#define IN1 8
#define IN2 11
#define IN3 12
#define IN4 13

#define IRCL A1  // left IR
#define IRCR A2  // right IR
#define IRS A3   // Side IR
// #define IRS A3  // Side IR

#define Left_IR_Target 1023   // tune this value
#define Right_IR_Target 1023  // tune this value
#define Side_IR_Target 1000   // tune this value

TimerEvent timer;
TimerEvent button;
TimerEvent status;

int SideInput;
double LeftSetpoint, LeftInput, LeftOutput;
double RightSetpoint, RightInput, RightOutput;

double L_Kp = 1, L_Ki = 0, L_Kd = 0;
double R_Kp = 1, R_Ki = 0, R_Kd = 0;

PID LEFT_PID(&LeftInput, &LeftOutput, &LeftSetpoint, L_Kp, L_Ki, L_Kd, DIRECT);
PID RIGHT_PID(&RightInput, &RightOutput, &RightSetpoint, R_Kp, R_Ki, R_Kd, DIRECT);

uint8_t flag = 0;

void timerCallback() {
  if (SideInput < Side_IR_Target && !flag) {  // stop line  detected
    flag = 1;                                 // stop line detected
    digitalWrite(IRL_LED, HIGH);
    if (flag == 1) {
      digitalWrite(IN1, LOW);  // stop the motor
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
    // while (!BUTTON);                      
    digitalWrite(IN1, HIGH);  // turn on the motor see which direction it goes
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 255);  //small boost
    analogWrite(EN2, 255);  //small boost
    // delay(1000);
    flag = 0;  // reset the flag
  } else {
    digitalWrite(IRL_LED, LOW);
  }

  // if the value of the LEFT IR sensor is greater than the target value
  if (LeftInput < Left_IR_Target) {  // left ir  detected
    digitalWrite(IRCL_LED, HIGH);
  } else {
    digitalWrite(IRCL_LED, LOW);
  }

  // if the value of the RIGHT IR sensor is greater than the target value
  if (RightInput < Right_IR_Target) {  // right ir  detected
    digitalWrite(IRCR_LED, HIGH);
  } else {
    digitalWrite(IRCR_LED, LOW);
  }

}

void ButtonCallback() {

  if (BUTTON) {  // if button is pressed
    digitalWrite(Button_LED, HIGH);
  } else {  // if button is not pressed
    digitalWrite(Button_LED, LOW);
  }

}


void StatusCallback() {
  SideInput = analogRead(IRS);
  LeftInput = analogRead(IRCL);
  RightInput = analogRead(IRCR);

  LEFT_PID.Compute();
  RIGHT_PID.Compute();

  analogWrite(EN1, 190+(LeftOutput/255));
  analogWrite(EN2, 190+(RightOutput/255));

  Serial.print("Left:");
  Serial.print(LeftInput);
  Serial.print(",Right:");
  Serial.print(RightInput);
  Serial.print(",side:");
  Serial.print(SideInput);
  Serial.print(",LeftOutput:");
  Serial.print(190+(LeftOutput/255)*65);
  Serial.print(",RightOutput:");
  Serial.println(190+(RightOutput/255)*65);

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  timer.set(2, timerCallback);    
  button.set(1, ButtonCallback);  
  status.set(2, StatusCallback);

  // setup the pin for me
  pinMode(BUTTON1_PIN, INPUT);

  pinMode(IRCL_LED, OUTPUT);
  pinMode(IRCR_LED, OUTPUT);
  pinMode(IRL_LED, OUTPUT);
  pinMode(IRR_LED, OUTPUT);
  pinMode(Button_LED, OUTPUT);
  pinMode(Power_LED, OUTPUT);

  // pinMode(EN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // pinMode(EN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  LeftSetpoint = Left_IR_Target;
  RightSetpoint = Right_IR_Target;

  LEFT_PID.SetMode(AUTOMATIC);
  LEFT_PID.SetSampleTime(2);

  RIGHT_PID.SetMode(AUTOMATIC);
  RIGHT_PID.SetSampleTime(2);

  digitalWrite(Power_LED, HIGH);  // turn on the power LED
}

void loop() {
  timer.update();
  button.update();
  status.update();
  digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}
