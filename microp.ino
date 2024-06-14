#define BUTTON1 2
#define LED1 3
#define LED2 4
#define LED3 5
#define LED4 6
#define LED5 7
#define IN1 8
#define IN2 9
#define EN1 10
#define EN2 11
#define IN3 12
#define IN4 13

#define IR1 A0
#define IR2 A1
#define IR3 A2
// #define IN4 13


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //setup the pin for me
  pinMode(BUTTON1, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // pinMode(EN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  pinMode(IR3, INPUT);


  // pinMode(IN4, OUTPUT);
  // pinMode(EN2, OUTPUT);
  //setup the serial communication
}

void loop() {
  int buttonState = digitalRead(IR1);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);

  Serial.println(buttonState);  

  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}
