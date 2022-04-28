#include<string.h>

void EstSnX(void);
int reading1;
int reading2;
void setup(){
  Serial.begin(9600);
  delay(50);
  //DDRC = 0b00000000;
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  digitalWrite(7, HIGH);
}
unsigned long t0 = 0;
unsigned long t1 = 0;
bool flag = false;

void loop(){

   
  reading1 = analogRead(A3);
  Serial.print(reading1);
      Serial.print("   ");
      Serial.print(reading2);
      Serial.println();
  delay(100);
  reading2 = analogRead(A4);
  Serial.print(reading1);
      Serial.print("   ");
      Serial.print(reading2);
      Serial.println();
      delay(100);

      
      
    
}
