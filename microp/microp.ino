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

#define IRSL A3  // left IR
#define IRCL A1  // left IR
#define IRCR A2  // right IR
#define IRSR A0  // Side IR
// #define IRS A3  // Side IR

#define Left_IR_Target 200        // tune this value
#define Right_IR_Target 200       // tune this value
#define Side_Left_IR_Target 200   // tune this value
#define Side_Right_IR_Target 200  // tune this value

TimerEvent timer;
TimerEvent button;
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
  // delay(100);
  uint8_t i=50;

  while (LeftInput < Left_IR_Target) {
    digitalWrite(Power_LED, HIGH);   // turn on the power LED
    digitalWrite(Button_LED, HIGH);  // turn on the power LED

    // Serial.print("lefttt");
    digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 200+(i--));
    LeftInput = analogRead(IRCL);
  }

  // while(RightInput < Right_IR_Target){
  //   digitalWrite(Power_LED, HIGH);  // turn on the power LED
  //   digitalWrite(Button_LED, HIGH);  // turn on the power LED

  //     digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
  //   digitalWrite(IN2, HIGH);
  //   digitalWrite(IN3, LOW);
  //   digitalWrite(IN4, HIGH);

  //   RightInput = analogRead(IRCR);
  //   analogWrite(EN1, 255);
  //   analogWrite(EN2, 150);
  //   }
}

void Turn_Left() {
  uint8_t i=50;
  // delay(100);
  while (RightInput < Right_IR_Target) {
    digitalWrite(Power_LED, HIGH);   // turn on the power LED
    digitalWrite(Button_LED, HIGH);  // turn on the power LED

    digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(EN1, 200+(i--));
    analogWrite(EN2, 0);
    RightInput = analogRead(IRCR);
  }

  // while(LeftInput < Left_IR_Target){
  // digitalWrite(Power_LED, HIGH);  // turn on the power LED
  // digitalWrite(Button_LED, HIGH);  // turn on the power LED

  // // Serial.print("lefttt");
  // LeftInput = analogRead(IRCL);
  // digitalWrite(IN1, HIGH);  // turn on the motor see which direction it goes
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // analogWrite(EN1, 190);
  // analogWrite(EN2, 255);
  // }
}


void Straight() {
  digitalWrite(Power_LED, HIGH);   // turn on the power LED
  digitalWrite(Button_LED, HIGH);  // turn on the power LED

  analogWrite(EN1, 150 + (LeftOutput / 255) * 100);
  analogWrite(EN2, 150 + (RightOutput / 255) * 100);
  digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Boost() {
  digitalWrite(Power_LED, HIGH);   // turn on the power LED
  digitalWrite(Button_LED, HIGH);  // turn on the power LED

  analogWrite(EN1, 255);
  analogWrite(EN2, 255);
  digitalWrite(IN1, LOW);  // turn on the motor see which direction it goes
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(100);
}



void timerCallback() {
  if (LeftInput > Left_IR_Target) {  // left ir  detected
    digitalWrite(IRCL_LED, HIGH);
    Straight();
  } else {
    digitalWrite(IRCL_LED, LOW);
  }

  // if the value of the RIGHT IR sensor is greater than the target value
  if (RightInput > Right_IR_Target) {  // right ir  detected
    digitalWrite(IRCR_LED, HIGH);
    Straight();
  } else {
    digitalWrite(IRCR_LED, LOW);
  }

  if (LeftInput > Left_IR_Target && RightInput > Right_IR_Target && (SideRightInput > Side_Right_IR_Target || SideLeftInput > Side_Left_IR_Target)) {  // left ir  detected
    while (!BUTTON) {
      Serial.println(SideLeftInput);
      digitalWrite(Power_LED, LOW);   // turn on the power LED
      digitalWrite(Button_LED, LOW);  // turn on the power LED
      digitalWrite(IN1, LOW);         // turn on the motor see which direction it goes
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(EN1, 0);
      analogWrite(EN2, 0);
      digitalWrite(IRCL_LED, HIGH);
      digitalWrite(IRCR_LED, HIGH);
      digitalWrite(IRL_LED, HIGH);
      digitalWrite(IRR_LED, HIGH);
    };
    Boost();
  }

  // // if the value of the LEFT IR sensor is greater than the target value
  else if (SideLeftInput > Side_Left_IR_Target) {  // left ir  detected
    digitalWrite(IRL_LED, HIGH);
    Turn_Left();
  }
  // if the value of the RIGHT IR sensor is greater than the target value
  else if (SideRightInput > Side_Right_IR_Target) {  // right ir  detected
    digitalWrite(IRR_LED, HIGH);
    Turn_Right();
  } else {
    digitalWrite(IRL_LED, LOW);
    // digitalWrite(IRCL_LED, LOW);
    // digitalWrite(IRCR_LED, LOW);
    digitalWrite(IRR_LED, LOW);
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
  LeftInput = analogRead(IRCL);
  RightInput = analogRead(IRCR);
  SideLeftInput = analogRead(IRSL);
  SideRightInput = analogRead(IRSR);

  LEFT_PID.Compute();
  RIGHT_PID.Compute();


  //   LeftInput = analogRead(IRCL);
  //   RightInput = analogRead(IRCR);
  //   SideLeftInput = analogRead(IRSL);
  //   SideRightInput = analogRead(IRSR);

   Serial.print("Left:");
    Serial.print(LeftInput);
    Serial.print(",Right:");
    Serial.print(RightInput);
    Serial.print(",side right:");
    Serial.print(SideRightInput);
     Serial.print(",side left:");
  Serial.println(",LeftOutput:");
  // Serial.print(0+(LeftOutput/255)*255);
  // Serial.print(",RightOutput:");
  // Serial.println(0+(RightOutput/255)*255);
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

  // digitalWrite(Power_LED, HIGH);  // turn on the power LED
  while(!BUTTON);
}

void loop() {
  timer.update();
  button.update();
  status.update();
}
