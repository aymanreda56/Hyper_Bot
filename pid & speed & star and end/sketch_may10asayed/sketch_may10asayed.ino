

#define enA 9                    //enable motor A  (left)
#define enB 10                   //enable motor B  (right)
#define in1 7                    //for motor A
#define in2 6                    //for motor A
#define in3 12                   //for motor B
#define in4 11                   //for motor B
#define button 4


#define sensorSharpLeft A0
#define sensorSharpRight A4
#define sensorLeft A1
#define sensorRight A3



#define sensorStraight A2

// this byte array to read the line sensors  { sharpLeft, left, center, right, sharpRight } 
short sensorReading[] = {0, 0, 0, 0, 0};

// the mode represents the state of the car    0 -> idle    1 -> moving   2 -> stopping   (3 -> intersection ??)
int mode = 0;
int previousPress;
int currentPress;
int error= 0;


// this function returns an integer representing the error type
  // 0 -> goStraight  00100
  // -1 -> goLeft  01100                   1 -> goRight    00110            
  // -2 -> goVeryLeft  01000               2 -> goVeryRight  00010
  // -3 -> goLittleLeftSharp  11000        3 -> goLittleRightSharp  00011
  // -4 -> goVeryLeftSharp  10000          4 -> goVeryRightSharp  00001
int PID(short* LFSensor){

  //thresholdSensor();

  if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) return 4;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) return 3; 

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 2;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return 0;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -2;

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -3;

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -4;

  return 5;

}

// thresholding the sensorReading array
void thresholdSensor(){
  int threshold = 250;
  for (int i=0 ; i<5; i++){
    sensorReading[i] = (sensorReading[i] < threshold)? 1 : 0;
    Serial.print(sensorReading[i]);
  } 
 
  Serial.println();
}

void checkButton(){
  previousPress = currentPress;
  currentPress = digitalRead(button);
}


// check the mode of the car every iteration
// note (case of intersection still need to be handled)
int checkMode(){
  
  if(mode == 0){
    checkButton();
    if(previousPress == HIGH && currentPress == LOW){
      return 1;
    }
    return 0;
  }
  else if(mode == 1){
    if ((sensorReading[0]== 0 )&&(sensorReading[1]== 1 )&&(sensorReading[2]== 1 )&&(sensorReading[3]== 1 )&&(sensorReading[4]== 0 )) {
      return 2;
    }
      
    return 1;
  }
  return 2;
}



void decideSpeed(int error) {
  int initSpeed = 70;
  int initSpeed_after_corr = initSpeed;
  int kp = 20;
  if(error == 4 || error == 5 || error == -4 || error == -5)
  {
    initSpeed_after_corr = 50;
  }
  if(error != 5){
    analogWrite(enA, initSpeed_after_corr + error*kp);   // left motor
    //delay(20);
    analogWrite(enB, initSpeed_after_corr - error*kp);   // right motor
  }
  else{
    analogWrite(enA, 0);   // left motor
    // delay(20);
    analogWrite(enB, 0);   // right motor
  }
}


unsigned long t1, t0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // line sensor pins
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);

  // motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  previousPress = LOW;
  currentPress = LOW;

  // making the two motors at zero speed
  analogWrite(enA, 0);   // left motor
  analogWrite(enB, 0);   // right motor

//ASSUMING ONE DIRECTION ONLYYYYY
//IE (WHEN SHARP TURNING LEFT, WE HOLD ONE MOTOR AT SPEED 0 WHILE THE OTHER AT ANY SMALL SPEED)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);


  t1 = 0;
  t0 = 0;

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

   pinMode(button, OUTPUT);
   digitalWrite(button, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  t1 = millis();
  delay(40);
  
  sensorReading[2] = analogRead(sensorStraight);
  t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }
  t1 = millis();
  sensorReading[4] = 255;
  t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }
  t1 = millis();
  
  sensorReading[1] = analogRead(sensorLeft);
  t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[3] = analogRead(sensorRight);
   t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[0] = analogRead(sensorSharpLeft);
   t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[4] = analogRead(sensorSharpRight);
   t1 = millis();
  while(abs(t1 - t0) > 500){
    t0 = millis();
  }

  // printing the line sensor readings
  /*
  Serial.print(sensorReading[0]);
  Serial.print("   ");
  Serial.print(sensorReading[1]);
  Serial.print("    ");
  Serial.print(sensorReading[2]);
  Serial.print("    ");
  Serial.print(sensorReading[3]);
  Serial.print("    ");
  Serial.print(sensorReading[4]);
  Serial.print("    ");
  Serial.println();
  delay(20);
  */

  thresholdSensor();
  //mode = checkMode();

  // case of moving
 
//  if (mode == 1){
    error = PID(sensorReading);
    decideSpeed(error);
//    Serial.println(error);
//    delay(1000);
 // }

// case of stopping
 // else if(mode == 2) {
//    analogWrite(enA, 0);   // left motor
//    analogWrite(enB, 0);   // right motor
 // }*/
  

}
