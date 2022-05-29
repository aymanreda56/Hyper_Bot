#define enA 10                    //enable motor A  (left)
#define enB 9                   //enable motor B  (right)
#define in1 7                    //for motor A
#define in2 6
//for motor A
#define in3 12                   //for motor B
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
char temp;
bool exitMaze = false;


// these variables are for maze solver
char path [100];
byte path_index = 0;


unsigned long t1, t0;

// this function returns an integer representing the error type
  // 0 -> goStraight  00100
  // -1 -> goLeft  01100                   1 -> goRight    00110            
  // -2 -> goVeryLeft  01000               2 -> goVeryRight  00010
  // -3 -> goLittleLeftSharp  11000        3 -> goLittleRightSharp  00011
  // -4 -> goVeryLeftSharp  10000          4 -> goVeryRightSharp  00001
int PID(short* LFSensor){


  if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -4;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) return 3; 

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 2;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) return 1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return 0;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -1;

  else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -2;

  else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) return -3;

  else if((LFSensor[4]== 1)&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[0]== 0 ) ) return 4;
  

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
  int threshold = 90;
  Serial.println(" ");
  for (int i=0 ; i<5; i++){
    sensorReading[i] = (sensorReading[i] < threshold)? 1 : 0;
    Serial.print(sensorReading[i]);
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
  //int initSpeed = 60;
  //int initSpeed_after_corr = 55;



  const int initialLeft = 60 * 1.2;
  const int initialRight = 60;

 int speed_A = 0;
 int speed_B = 0;

 if(error == -4 )
  {
    
    analogWrite(enA, initialLeft);   // left motor
     
     analogWrite(enB, 0);   // right motor
    
  }
  else if ( error == 4)
   {
     analogWrite(enA, 0 );   // left motor
    analogWrite(enB, initialRight );   // right motor
  }
 else if (error == 1 || error == 2 || error == -2 || error == -1) {
     analogWrite(enA, initialLeft + speed_1 );   // left motor
    analogWrite(enB, initialRight - speed_1 );   // right motor
  }
 
  else if (error == 0){
    analogWrite(enA, initialLeft);   // left motor

    analogWrite(enB, initialRight);   // right motor

    /*Serial.println("left motor: ");  Serial.println(initSpeed + speed_1);  
     Serial.println("right motor: "); Serial.println(initSpeed - speed_1); */
  }  
}


// this section is for mazeSolver
void readSensors() {
   t1 = millis();
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
}


void followLineUntilDeadOrIntersect() {
  while(1) {
    readSensors();
    thresholdSensor();

    error = PID(sensorReading);
    
    decideSpeed(new_pid());

    if(sensorReading[1] == 0 && sensorReading[2] == 0 && sensorReading[3] == 0 && sensorReading[0] == 0 && sensorReading[4] == 0)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }

    else if(sensorReading[0] == 1 || sensorReading[4] == 1)
    {
      // Found an intersection.
      return;
    }
  }
  
}

void followLineUntilZeroError_andStop() {
  while(1) {
    readSensors();
    thresholdSensor();

    error = PID(sensorReading);
    
    decideSpeed(new_pid());

    if(sensorReading[1] == 0 && sensorReading[2] == 1 && sensorReading[3] == 0)
    {
      analogWrite(enA, 0);   // left motor
      analogWrite(enB, 0);   // right motor
      return;
    }
  }
}

void turnLeft_until_ZeroError() {
  
   analogWrite(enA, 0);   // left motor
   analogWrite(enB, 80);   // right motor
  //Serial.println("sharp left");
   readSensors();
   thresholdSensor();

   while(sensorReading[0] != 1) {
     readSensors();
     thresholdSensor();
   }

  followLineUntilZeroError_andStop();
  
}


void turnRight_until_ZeroError() {
  
   analogWrite(enA, 80*1.2);   // left motor
   analogWrite(enB, 0);   // right motor

  //Serial.println("sharp right");
   readSensors();
   thresholdSensor();

   while(sensorReading[4] != 1) {
     readSensors();
     thresholdSensor();
   }

  followLineUntilZeroError_andStop();
  
}


char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_left)
    return 'L';
  else if(found_straight)
    return 'S';
  else if(found_right)
    return 'R';
  else
    return 'B';
}


void turn(char dir)
{
  switch(dir)
  {
    // Turn left 90deg
    case 'L':    
      turnLeft_until_ZeroError();
      break;
      
    // Turn right 90deg
    case 'R':        
      turnRight_until_ZeroError();
      break;
    
    // Turn right 180deg to go back
    case 'B':    
      turnRight_until_ZeroError();          
      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
  }
}

void REARRANGE(int idx)
{
    for (int i = idx; i < path_index-2; i++)
    {
        temp = path[i];
        path[i] = path[i+2];
        path[i+2]=temp;
    }
    
}


void simplifyPath()
{
  /* LBL = S
     LBR = B
     LBS = R
     RBL = B
     SBL = R
     SBS = B */

  char ACTION;
  
  for(int i = 0; i <= path_index-2; i++)
    {
      ACTION = path[i];
           
      if(ACTION == 'B')
        {
          if(path[i-1]== 'L' && path[i+1] == 'R')
            {
              path[i-1] = 'B';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i); 
            }

           if(path[i-1]== 'L' && path[i+1] == 'S')
            {
              path[i-1] = 'R';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i);
            }

            if(path[i-1]== 'R' && path[i+1] == 'L')
            {
              path[i-1] = 'B';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i);         
            }

            if(path[i-1]== 'S' && path[i+1] == 'L')
            {
              path[i-1] = 'R';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i);             
            }

            if(path[i-1]== 'S' && path[i+1] == 'S')
            {
              path[i-1] = 'B';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i);             
            }

            if(path[i-1]== 'L' && path[i+1] == 'L')
            {
              path[i-1] = 'S';
              path[i] = 0;
              path[i+1] = 0;
              REARRANGE(i);
            }
            
          i = -1;
        }
         
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
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(!exitMaze)
  {
    MazeSolve();
    exitMaze = true;
  }
  if(digitalRead(button))
  {
     afterMazeSolved();
  }
}






void MazeSolve()
{
  // Loop until we have solved the maze.
  while(1)
  {
    // FIRST MAIN LOOP BODY  
    followLineUntilDeadOrIntersect();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    /*digitalWrite(dir_a, LOW);  
    analogWrite(pwm_a, 200);
    digitalWrite(dir_b, LOW);  
    analogWrite(pwm_b, 200);   
    delay(25); */

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;

    // Check for left and right exits.
    if(sensorReading[0] == 1)
      found_left = 1;
    if(sensorReading[4] == 1)
     found_right = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    // increase the delay if the car didn't good step.
    // analogWrite(enA, 50);   // left motor
    // analogWrite(enB, 25);   // right motor
    // delay(20); 
  
    readSensors();
    thresholdSensor();
    if(sensorReading[1] == 1 || sensorReading[2] == 1 || sensorReading[3] == 1)
      found_straight = 1;

    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
    if(sensorReading[1] == 1 && sensorReading[2] == 1 && sensorReading[3] == 1) {
      analogWrite(enA, 0);   // left motor
      analogWrite(enB, 0);   // right motor
      break;
    }
     

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_index] = dir;
    path_index ++;

    // Simplify the learned path.
    //simplify_path();
  }

    simplifyPath();
}


void afterMazeSolved() {
  path_index = 0;
  while(1)
  {
    // FIRST MAIN LOOP BODY  
    followLineUntilDeadOrIntersect();

    readSensors();
    thresholdSensor();
    if(sensorReading[1] == 1 && sensorReading[2] == 1 && sensorReading[3] == 1) {
      analogWrite(enA, 0);   // left motor
      analogWrite(enB, 0);   // right motor
      break;
    }
     
    turn(path[path_index]);
    path_index++;
  }
}
