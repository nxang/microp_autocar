#include <PID_v1.h>
#include <TimerEvent.h>

#define BUTTON1_PIN A5
#define BUTTON digitalRead(BUTTON1_PIN)

#define IRS1_LED 2
#define IRS2_LED 3
#define IRL_LED 4
#define IRR_LED 7
#define Button_LED 6
#define Power_LED 5

#define EN1 9
#define EN2 10

#define IN1 8
#define IN2 11
#define IN3 12
#define IN4 13

#define IRS A2  // Side IR
#define IRL A3  // left IR
#define IRR A4  // right IR

#define Left_IR_Target 900   // tune this value
#define Right_IR_Target 900  // tune this value
#define Side_IR_Target 900   // tune this value

TimerEvent timer;
TimerEvent timer1;

int SideInput;
double LeftSetpoint, LeftInput, LeftOutput;
double RightSetpoint, RightInput, RightOutput;

double L_Kp = 2, L_Ki = 1, L_Kd = 0;
double R_Kp = 2, R_Ki = 1, R_Kd = 0;

PID LEFT_PID(&LeftInput, &LeftOutput, &LeftSetpoint, L_Kp, L_Ki, L_Kd, REVERSE);
PID RIGHT_PID(&RightInput, &RightOutput, &RightSetpoint, R_Kp, R_Ki, R_Kd, REVERSE);

uint8_t flag = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  timer.set(5, timerCallback);  // Call timerCallback every 2 milliseconds (1 second)
  timer1.set(2, ButtonCallback);  // Call timerCallback every 2 milliseconds (1 second)

  // setup the pin for me
  pinMode(BUTTON1_PIN, INPUT);

  pinMode(IRS1_LED, OUTPUT);
  pinMode(IRS2_LED, OUTPUT);
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

  pinMode(IRS, INPUT);
  pinMode(IRL, INPUT);
  pinMode(IRR, INPUT);

  LeftSetpoint = Left_IR_Target;
  RightSetpoint = Right_IR_Target;

  LEFT_PID.SetMode(AUTOMATIC);
  LEFT_PID.SetSampleTime(2);

  RIGHT_PID.SetMode(AUTOMATIC);
  RIGHT_PID.SetSampleTime(2);

  digitalWrite(Power_LED, HIGH);  // turn on the power LED
  // digitalWrite(Button_LED, HIGH);  // turn on the power LED
  // digitalWrite(IRR_LED, HIGH);  // turn on the power LED
  // digitalWrite(IRL_LED, HIGH);  // turn on the power LED
}

void loop() {

  Serial.println(digitalRead(BUTTON1_PIN));
  // digitalWrite(IN1, HIGH);  // turn on the motor see which direction it goes
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);

  // /***************************************************** IR START *****************************************************/
  // // read the value of the IR sensor
  // SideInput = analogRead(IRS);
  // LeftInput = analogRead(IRL);
  // RightInput = analogRead(IRR);

  // // if the value of the SIDE IR sensor is greater than the target value
  // if (SideInput < Side_IR_Target && !flag) {  // stop line  detected
  //   flag = 1;                                 // stop line detected
  //   digitalWrite(IRS_LED, HIGH);
  //   if (flag == 1) {
  //     digitalWrite(IN1, LOW);  // stop the motor
  //     digitalWrite(IN2, LOW);
  //     digitalWrite(IN3, LOW);
  //     digitalWrite(IN4, LOW);
  //   }
  //   while (!BUTTON)
  //     ;                       // wait for the button to be pressed
  //   digitalWrite(IN1, HIGH);  // turn on the motor see which direction it goes
  //   digitalWrite(IN2, LOW);
  //   digitalWrite(IN3, HIGH);
  //   digitalWrite(IN4, LOW);
  //   analogWrite(EN1, 255);  //small boost
  //   analogWrite(EN2, 255);  //small boost
  //   delay(1000);
  //   flag = 0;  // reset the flag
  // } else {
  //   digitalWrite(IRS_LED, LOW);
  // }

  // // if the value of the LEFT IR sensor is greater than the target value
  // if (LeftInput < Left_IR_Target) {  // left ir  detected
  //   LEFT_PID.SetControllerDirection(DIRECT);
  //   digitalWrite(IRL_LED, HIGH);
  // } else {
  //   LEFT_PID.SetControllerDirection(REVERSE);
  //   digitalWrite(IRL_LED, LOW);
  // }

  // // if the value of the RIGHT IR sensor is greater than the target value
  // if (RightInput < Right_IR_Target) {  // right ir  detected

  //   RIGHT_PID.SetControllerDirection(DIRECT);
  //   digitalWrite(IRR_LED, HIGH);
  // } else {
  //   RIGHT_PID.SetControllerDirection(REVERSE);
  //   digitalWrite(IRR_LED, LOW);
  // }

  /***************************************************** IR END *****************************************************/
  /***************************************************** END *****************************************************/
}

void timerCallback() {

  LEFT_PID.Compute();
  RIGHT_PID.Compute();
  analogWrite(EN1, LeftOutput);
  analogWrite(EN2, RightOutput);
}

void ButtonCallback() {
  // put your main code here, to run repeatedly:


}
