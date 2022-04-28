#include<string.h>

void EstSnX(void);
int readingSharpLeft;
int readingSharpRight;
int readingLeft;
int readingRight;
int readingStraight;

#define enA 9
#define enB 10
#define in1 6
#define in2 7
#define in3 11
#define in4 12
#define button 4


#define sensorSharpLeft A0
#define sensorSharpRight A1
#define sensorLeft A2
#define sensorRight A3
#define sensorStraight A4




int speedP;
int pwmOutputA = 0;
int pwmOutputB = 0;




void turnSharpLeft()
{
  pwmOutputA = 40;
  pwmOutputB = 0;
  analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin
  delay(20);
}


void turnSharpRight()
{
  pwmOutputB = 40;
  pwmOutputA = 0;
  analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin
  delay(20);
}


void turnLeft()
{
  pwmOutputA = 80;
  pwmOutputB = 30;
  analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin
  delay(20);
}


void turnRight()
{
  pwmOutputB = 80;
  pwmOutputA = 30;
  analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin
  delay(20);
}

void goStraight()
{
  pwmOutputB = 100;
  pwmOutputA = 100;
  analogWrite(enA, pwmOutputA); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutputB); // Send PWM signal to L298N Enable pin
  delay(20);
}



void setup(){
  Serial.begin(9600);
  delay(50);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  
  //digitalWrite(7, HIGH);


  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.println("hello1");

//ASSUMING ONE DIRECTION ONLYYYYY
//IE (WHEN SHARP TURNING LEFT, WE HOLD ONE MOTOR AT SPEED 0 WHILE THE OTHER AT ANY SMALL SPEED)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  Serial.println("hello");

  
}
unsigned long t0 = 0;
unsigned long t1 = 0;
bool flag = false;

void loop(){

   
  readingSharpLeft = analogRead(sensorSharpLeft);
  delay(10);
  readingSharpRight = analogRead(sensorSharpRight);
  delay(10);
  readingLeft = analogRead(sensorLeft);
  delay(10);
  readingRight = analogRead(sensorRight);
  delay(10);
  readingStraight = analogRead(sensorStraight);
  delay(10);
  
  Serial.println(readingSharpLeft);
  Serial.print("   ");
  Serial.print(readingLeft);
  Serial.print("    ");
  Serial.print(readingStraight);
  Serial.print("    ");
  Serial.print(readingRight);
  Serial.print("    ");
  Serial.print(readingSharpRight);
  Serial.print("    ");
  delay(20);

while((readingSharpLeft != 0) && (readingSharpRight != 0))
{

while((readingLeft != 0) && (readingRight != 0))
{
  goStraight();
  
}

  if(readingLeft == 0)
  {
    turnLeft();  
  }
  else if(readingRight == 0)
  {
    turnRight();
  }
}



  if(readingSharpLeft == 0)
  {
    turnSharpLeft();
  }
  else if(readingSharpRight == 0)
  {
    turnSharpRight();
  }

      
      
    
}
