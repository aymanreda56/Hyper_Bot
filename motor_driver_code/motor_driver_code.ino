#define enA 9
#define in1 6
#define in2 7
#define button 4

//int rotDirection = 0;
//int pressed = false;
int speedP;

void setup() {
    Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  Serial.println("hello1");

//  pinMode(button, INPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Serial.println("hello");
}

void loop() {
  speedP = millis() % 1023;
  int potValue = speedP; // Read potentiometer value
  int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  Serial.println(pwmOutput);

  // Read button - Debounce
  //if (digitalRead(button) == true) {
    //pressed = !pressed;
  //}
  //while (digitalRead(button) == true);
  delay(20);

  
}
