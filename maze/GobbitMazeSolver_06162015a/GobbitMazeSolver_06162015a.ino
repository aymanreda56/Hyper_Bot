// The Pololu QTR Library and portions of the Pololu sample maze solving
// code have been used here.

#include <QTRSensors.h>  // Pololu QTR Library 
//QRTSensors folder must be placed in your arduino libraries folder

//line sensor defines
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN  // emitter control pin not used.  If added, replace QTR_NO_EMITTER_PIN with pin#

//line sensor declarations
// sensors 0 through 7 are connected to digital pins 2 through 10, respectively (pin 3 is skipped and use for motor control)
QTRSensorsRC qtrrc((unsigned char[]) {2, 4, 5, 6, 7, 8, 9,10},
NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
unsigned int line_position=0; // value from 0-7000 to indicate position of line between sensor 0 - 7

// motor driver vars
// pwm_a/b sets speed.  Value range is 0-255.  For example, if you set the speed at 100 then 100/255 = 39% duty cycle = slow 
int pwm_a = 3;  //PWM control for motor outputs 1 and 2 is on digital pin 10  (Left motor)
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11  (Right motor)
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12  (Left motor)
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13  (Right motor)

// motor tuning vars for maze navigating
int calSpeed = 165;   // tune value motors will run while auto calibration sweeping turn (0-255)
int turnSpeed = 200;  // tune value motors will run while turning (0-255)
int turnSpeedSlow = 125;  // tune value motors will run as they slow down from turning cycle to avoid overrun (0-255)
int drivePastDelay = 300; // tune value in mseconds motors will run past intersection to align wheels for turn

// pid loop vars
float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;  // tune value in follow_line() function
//float ki = 0; // ki is not currently used
float kd =0;   // tune value in follow_line() function
int m1Speed=0;
int m2Speed=0;
int motorspeed=0;


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your 
// maze design does not exceed the bounds of the array.
char path[100] = "";
unsigned char path_length = 0; // the length of the path

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  analogWrite(pwm_a, 0);  //set both motors to stop at (100/255 = 39)% duty cycle (slow)
  analogWrite(pwm_b, 0);
  
  Serial.begin(115200);   
  
  Serial.println("Zagros Robotics, Inc.");
  Serial.println("Maze Solver Demo\n ");
  Serial.println("Version 06162015a - PD");
  Serial.println("Calibrating sensor");
  
  // calibrate line sensor, determines min/max range of sensed values for the current course
  delay(2000);
  
  for (int i = 0; i <= 100; i++)  // begin calibration cycle to last about 2.5 seconds (100*25ms/call)
  {
    
    // auto calibration sweeping left/right, tune 'calSpeed' motor speed at declaration 
    if (i==0 || i==60)  // slow sweeping turn right to pass sensors over line
    {
       digitalWrite(dir_a, LOW); 
       analogWrite(pwm_a, calSpeed);
       digitalWrite(dir_b, HIGH);  
       analogWrite(pwm_b, calSpeed);   
    }
    
    else if (i==20 || i==100)  // slow sweeping turn left to pass sensors over line
    {
       digitalWrite(dir_a, HIGH); 
       analogWrite(pwm_a, calSpeed);
       digitalWrite(dir_b, LOW);  
       analogWrite(pwm_b, calSpeed);   
    }
    
    qtrrc.calibrate();       // reads all sensors with the define set 2500 microseconds (25 milliseconds) for sensor outputs to go low.
    
  }  // end calibration cycle
  
  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  line_position = qtrrc.readLine(sensorValues);
  
  while (sensorValues[6] < 200)  // wait for line position to near center
  {
    line_position = qtrrc.readLine(sensorValues);
  }
  
  // slow down speed
  analogWrite(pwm_a, turnSpeedSlow);
  analogWrite(pwm_b, turnSpeedSlow); 
  
  // find center
  while (line_position > 4350)  // wait for line position to find center
  {
    line_position = qtrrc.readLine(sensorValues);
  }
 
  // stop both motors
  analogWrite(pwm_b, 0);  // stop right motor first which kinda helps avoid over run
  analogWrite(pwm_a, 0);
  
  // print calibration results
  
  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println("Calibration Complete");
  delay(1000);
  
} // end setup

void loop()
{

  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int line_position = qtrrc.readLine(sensorValues);
  
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.print(line_position); // comment this line out if you are using raw values
  Serial.print("  ");
  Serial.print(sensorValues[0]);
  Serial.print("  ");
  Serial.println(sensorValues[7]);
  
  // begin maze solving
  MazeSolve(); // comment out and run serial monitor to test sensors while manually sweeping across line
  
  Serial.print("::");
  Serial.print(error);
  Serial.print('\t');
  Serial.print(PV);
  Serial.print('\t');
  Serial.print(m1Speed);
  Serial.print('\t');
  Serial.print(m2Speed);
  Serial.print('\t');
  Serial.print(lastError);
  Serial.print('\t');

}  // end main loop



//line following subroutine
// PD Control
void follow_line()  //follow the line
{

  lastError = 0;
  
  while(1)
  {

    line_position = qtrrc.readLine(sensorValues);
    switch(line_position)
    {
   
      // case 0 and case 7000 are used in the instructable for PD line following code.
      // kept here as reference.  Otherwise switch function could be removed.
      
      //Line has moved off the left edge of sensor
      //case 0:
      //       digitalWrite(dir_a, LOW); 
      //       analogWrite(pwm_a, 200);
      //       digitalWrite(dir_b, HIGH);  
      //       analogWrite(pwm_b, 200);
      //       Serial.println("Rotate Left\n");
      //break;
  
      // Line had moved off the right edge of sensor
      //case 7000:
      //       digitalWrite(dir_a, HIGH); 
      //       analogWrite(pwm_a, 200);
      //       digitalWrite(dir_b, LOW);  
      //       analogWrite(pwm_b, 200);
      //       Serial.println("Rotate Right\n");
      //break;
   
      default:
        error = (float)line_position - 3500;
   
        // set the motor speed based on proportional and derivative PID terms
        // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
        // kd is the floating-point derivative constant (maybe start with a value around 1)
        // note that when doing PID, it's very important you get your signs right, or else the
        // control loop will be unstable
        kp=.5;
        kd=1;
     
        PV = kp * error + kd * (error - lastError);
        lastError = error;
    
        //this codes limits the PV (motor speed pwm value)  
        // limit PV to 55
        if (PV > 55)
        {
        
          PV = 55;
        }
    
        if (PV < -55)
        {
          PV = -55;
        }
        
        m1Speed = 200 + PV;
        m2Speed = 200 - PV;
       
        //set motor speeds
        digitalWrite(dir_a, LOW);  
        analogWrite(pwm_a, m2Speed);
        digitalWrite(dir_b, LOW);  
        analogWrite(pwm_b, m1Speed);
        break;
    }
  
    // We use the inner six sensors (1 thru 6) to
    // determine if there is a line straight ahead, and the
    // sensors 0 and 7 if the path turns.
    if(sensorValues[1] < 100 && sensorValues[2] < 100 && sensorValues[3] < 100 && sensorValues[4] < 100 && sensorValues[5] < 100 && sensorValues[6] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }

    else if(sensorValues[0] > 200 || sensorValues[7] > 200)
    {
      // Found an intersection.
      return;
    }

  } 

} // end follow_line  

  
     
// Turns to the sent variable of
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back)
// Tune 'turnSpeed' at declaration
void turn(char dir)
{
  switch(dir)
  {
    // Turn left 90deg
    case 'L':    
      digitalWrite(dir_a, HIGH); 
      analogWrite(pwm_a, turnSpeed);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, turnSpeed);
      
      line_position = qtrrc.readLine(sensorValues);
      
      while (sensorValues[6] <200)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
  
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position > 4350)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_b, 0);  // stop right motor first to better avoid over run
      analogWrite(pwm_a, 0);  
      break;
      
    // Turn right 90deg
    case 'R':        
      digitalWrite(dir_a, LOW); 
      analogWrite(pwm_a, turnSpeed);
      digitalWrite(dir_b, HIGH);  
      analogWrite(pwm_b, turnSpeed);
           
      line_position = qtrrc.readLine(sensorValues);
      
      while (sensorValues[1] <200)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
    
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position < 3250)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);      
      break;
    
    // Turn right 180deg to go back
    case 'B':		
      digitalWrite(dir_a, LOW); 
      analogWrite(pwm_a, turnSpeed);
      digitalWrite(dir_b, HIGH);  
      analogWrite(pwm_b, turnSpeed);
      
      line_position = qtrrc.readLine(sensorValues);
  
      while (sensorValues[1] <200)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
       
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow); 
      
      // find center
      while (line_position < 3250)  // tune - wait for line position to find near center
      {
        line_position = qtrrc.readLine(sensorValues);
      }
     
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);           
      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
  }
} // end turn


// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
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
} // end select_turn


// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[path_length-i])
    {
      case 'R':
        total_angle += 90;
	break;
      case 'L':
	total_angle += 270;
	break;
      case 'B':
	total_angle += 180;
	break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch(total_angle)
  {
    case 0:
	path[path_length - 3] = 'S';
	break;
    case 90:
	path[path_length - 3] = 'R';
	break;
    case 180:
	path[path_length - 3] = 'B';
	break;
    case 270:
	path[path_length - 3] = 'L';
	break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
  
} // end simplify_path


// This function is called once, from the main loop
void MazeSolve()
{
  // Loop until we have solved the maze.
  while(1)
  {
    // FIRST MAIN LOOP BODY  
    follow_line();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    digitalWrite(dir_a, LOW);  
    analogWrite(pwm_a, 200);
    digitalWrite(dir_b, LOW);  
    analogWrite(pwm_b, 200);   
    delay(25); 

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;
		
    // Now read the sensors and check the intersection type.
    line_position = qtrrc.readLine(sensorValues); // error ----------------------------------------------

    // Check for left and right exits.
    if(error == 4)
    found_right = 1;
    if(error == -4)
    found_left = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    digitalWrite(dir_a, LOW);  
    analogWrite(pwm_a, 200);
    digitalWrite(dir_b, LOW);  
    analogWrite(pwm_b, 200);
    delay(drivePastDelay); 
  
    line_position = qtrrc.readLine(sensorValues);
    if(error == 1||error ==0||error ==-1||error ==2||error ==-2)
    found_straight = 1;

    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
    //if(sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] > 600)
	  //break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    //path[path_length] = dir;
    //path_length ++;

    // Simplify the learned path.
    //simplify_path();
  }

  // Solved the maze!

  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  while(1)
  {
    //  maybe you would like to add a blinking led or a beeper.
    //  we just have it waiting patiently to be placed back on the starting line.
    analogWrite(pwm_a, 0);  // stop both motors
    analogWrite(pwm_b, 0);
  
    // while(1){}; // uncomment this line to cause infinite loop to test if end was found if your robot never seems to stop

    // hold motors while robot is sitting on end point
    line_position = qtrrc.readLine(sensorValues);
    while(sensorValues[1] > 200 && sensorValues[2] > 200 && sensorValues[3] > 200 && sensorValues[4] > 200 && sensorValues[5] > 200 && sensorValues[6] > 200)
    {
      line_position = qtrrc.readLine(sensorValues);
      delay(50);
    }
 
    // hold until the start line is seen again when the robot has been placed there again
    while(1)
    {
      line_position = qtrrc.readLine(sensorValues);
      if(sensorValues[2] > 200 || sensorValues[3] > 200 || sensorValues[4] > 200 || sensorValues[5] > 200 && sensorValues[0] < 200 && sensorValues[1] < 200 && sensorValues[6] < 200 && sensorValues[7] < 200)
      break;
      delay(50);
    }
 
    // delay to give you time to let go of the robot
    delay(2000);

    // Re-run the now solved maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    int i;
    for(i=0;i<path_length;i++)
    {
      // SECOND MAIN LOOP BODY  
      follow_line();

      // drive past intersection slightly slower and timed delay to align wheels on line
      digitalWrite(dir_a, LOW);  
      analogWrite(pwm_a, 200);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, 200);
      delay(drivePastDelay); // tune time to allow wheels to position for correct turning

      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }

    // Follow the last segment up to the finish.
    follow_line();

      // drive past intersection slightly slower and timed delay to align wheels on line
      digitalWrite(dir_a, LOW);  
      analogWrite(pwm_a, 200);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, 200);
      delay(drivePastDelay); // tune time to allow wheels to position for correct turning
        
      // Now we should be at the finish!  Now move the robot again and it will re-run this loop with the solution again.  
 
  } // end running solved
  
} // end MazeSolve
