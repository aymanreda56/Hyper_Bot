#include<string.h>

int reading1;
int reading2;
int reading3;
int reading4;
int reading5;
void setup(){
  Serial.begin(9600);
  delay(50);
  //DDRC = 0b00000000;
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  digitalWrite(7, HIGH);
}
unsigned long t0 = 0;
unsigned long t1 = 0;
bool flag = false;

void loop(){

   
  reading1 = analogRead(A0);
  delay (20);
  reading2 = analogRead(A1);
  delay (20);
  reading3 = analogRead(A2);
  delay (20);
  reading4 = analogRead(A3);
  delay (20);
  reading5 = analogRead(A4);
  delay (20);
  Serial.println(" ");
  Serial.print(reading1);
  Serial.print("   ");
  Serial.print(reading2);
  Serial.print("   ");
  Serial.print(reading3);
  Serial.print("   ");
  Serial.print(reading4);
  Serial.print("   ");
  Serial.print(reading5);
  Serial.print("   ");


      
      
    
}
