

#define enA 9                    //enable motor A  (left)
#define enB 10                   //enable motor B  (right)
#define in1 7                    //for motor A right
#define in2 6                    //for motor A
#define in3 12                   //for motor B left
#define in4 11                   //for motor B
#define button 2
#define kd_read A5


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
int previousError = 0;
bool hasStarted = true;
int p=0;
int d=0;
int kp=7;
int kd =0;
int cnt = 0;
int dir = 0;



unsigned long t1, t0;

// this function returns an integer representing the error type
  // 0 -> goStraight  00100
  // -1 -> goLeft  01100                   1 -> goRight    00110            
  // -2 -> goVeryLeft  01000               2 -> goVeryRight  00010
  // -3 -> goLittleLeftSharp  11000        3 -> goLittleRightSharp  00011
  // -4 -> goVeryLeftSharp  10000          4 -> goVeryRightSharp  00001
int PID(short* LFSensor){


  if((LFSensor[0]== 1 )) return -4;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) return 3; 

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 2;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return 0;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -2;

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -3;

  else if(LFSensor[4]== 1 ) return 4;
  
  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return 5;

  return error;
 

}


int new_pid() {
  int err =  error;
  p = err;
  d = err - previousError;

  previousError = err;
  return kp * p + kd * d;
  
}

// thresholding the sensorReading array
void thresholdSensor(){
  int threshold = 200;
  for (int i=0 ; i<5; i++){
    sensorReading[i] = (sensorReading[i] < threshold)? 1 : 0;
    //Serial.print(sensorReading[i]);
  } 
 
  //Serial.println();
}

void checkButton(){
  previousPress = currentPress;
  currentPress = digitalRead(button);
}


// check the mode of the car every iteration
// note (case of intersection still need to be handled)
int checkMode(){
  
  
  if(mode == 0){
    //checkButton();
    /*if(previousPress == HIGH && currentPress == LOW){
      hasStarted = false;
      return 1;
    }
    return 0;*/
    t1 = millis();
  while(abs(t1 - t0) > 1){
    t0 = millis();
    }

    hasStarted = false;

    return 1;
  }
  
  else if(mode == 1){
    if ((sensorReading[0]== 0 )&&(sensorReading[1]== 1 )&&(sensorReading[2]== 1 )&&(sensorReading[3]== 1 )&&(sensorReading[4]== 0 ) && hasStarted) {
        return 2;
    }
    if(cnt==10)
    {
      hasStarted = true;
      cnt =0;
    }
    else
      cnt++;
    
    return 1;
  }
    
  

  else if(mode == 2){
      
    return 2;
  }
  return 2;

}



void decideSpeed(int speed_1) {
  int initSpeed = 60;
  //int initSpeed_after_corr = 55;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  const int initialLeft = 80 * 1.2;
  const int initialRight = 80;

 int speed_A = 0;
 int speed_B = 0;

 if(error == -4 )
  {
    
    analogWrite(enA, initialLeft);   // left motor
     
     analogWrite(enB, 0);   // right motor
    dir = 1;
  }
  else if ( error == 4)
   {
     analogWrite(enA, 0 );   // left motor
    analogWrite(enB, initialRight );   // right motor
    dir = 0;
  }
 else if (error == 1 || error == 2 || error == -2 || error == -1) {
     analogWrite(enA, initialLeft - speed_1 );   // left motor
    analogWrite(enB, initialRight + speed_1 );   // right motor
  }
  else if(error == 5)
  {
    if(dir == 0){
      digitalWrite(in2, HIGH);
      digitalWrite(in1, LOW);
      t1 = millis();
    while(abs(t1 - t0) > 1500){
      t0 = millis();
      analogWrite(enA, initialLeft );   // left motor
      analogWrite(enB, initialRight );   // right motor
    }
  }
  else if(dir == 1){
      digitalWrite(in4, HIGH);
      digitalWrite(in3, LOW);
      t1 = millis();
    while(abs(t1 - t0) > 1500){
      t0 = millis();
      analogWrite(enA, initialLeft );   // left motor
      analogWrite(enB, initialRight );   // right motor
    }
  }
    
  }
  else if (error == 0){
    analogWrite(enA, initialLeft);   // left motor

    analogWrite(enB, initialRight);   // right motor

    /*Serial.println("left motor: ");  Serial.println(initSpeed + speed_1);  
     Serial.println("right motor: "); Serial.println(initSpeed - speed_1); */
  }
  

 
  
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // line sensor pins
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A5, INPUT);

  // motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //attachInterrupt(1, checkButton, RISING);


  previousPress = LOW;
  currentPress = LOW;

  // making the two motors at zero speed
  analogWrite(enA, 0);   // left motor
  analogWrite(enB, 0);   // right motor

//ASSUMING ONE DIRECTION ONLYYYYY
//IE (WHEN SHARP TURNING LEFT, WE HOLD ONE MOTOR AT SPEED 0 WHILE THE OTHER AT ANY SMALL SPEED)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);


  t1 = 0;
  t0 = 0;

  //kp = map(analogRead(kd_read), 0, 1023, 0, 30);

  //Serial.println(kd);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  pinMode(button, INPUT);
  digitalWrite(button, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  //kp = map(analogRead(kd_read), 0, 1023, 0, 30);
  //Serial.println(kp);
  
  t1 = millis();
  //delay(40);
  
  sensorReading[2] = analogRead(sensorStraight);
  t1 = millis();
  while(abs(t1 - t0) > 1){
    t0 = millis();
  }
  t1 = millis();
 

  
  sensorReading[1] = analogRead(sensorLeft);
  t1 = millis();
  while(abs(t1 - t0) > 1){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[3] = analogRead(sensorRight);
   t1 = millis();
  while(abs(t1 - t0) > 1){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[0] = analogRead(sensorSharpLeft);
   t1 = millis();
  while(abs(t1 - t0) > 1){
    t0 = millis();
  }
 
  t1 = millis();
  sensorReading[4] =analogRead(sensorSharpRight); //1000;
   t1 = millis();
  while(abs(t1 - t0) > 1){
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
  Serial.println();*/
  //delay(20);
  

 
  thresholdSensor();
  //mode = checkMode();

  // case of moving
 
  //if (mode == 1){
    error = PID(sensorReading);
    
    decideSpeed(new_pid());
//    Serial.println(error);
  //}

// case of stopping
 /* else if(mode == 2) {
   analogWrite(enA, 0);   // left motor
   analogWrite(enB, 0);   // right motor
 }*/

 //Serial.print("mode = ");  Serial.print(mode);  Serial.println();
  

}
