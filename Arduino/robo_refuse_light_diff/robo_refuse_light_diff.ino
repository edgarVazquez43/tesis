int lightSensor_left;
int lightSensor_right;
int setPoint;
int error;

float kp;
float kd;
float ki;

void setup() {
  // put your setup code here, to run once:
  // Output motor right
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  // Output motor left
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);

}


void outputController(int output)
{
   if(abs(output) < 9)
  {
    Serial.println(" ---   output_map*: 25  ");
    //Direcction motor right (forward)
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    //PWM-for motor right
    analogWrite(10, 50);

    //Direcction motor left (backward)
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    //PWM-for motor left
    analogWrite(9, 50);
    delay(50);
  }
  else if(output < 0 )
  {
    output = abs(output);
    output = map(output, 10, 100, 40, 60);
  
    Serial.print(" ---   output_map:   ");
    Serial.println(output);
    
    //Direcction motor right (forward)
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    //PWM-for motor right
    analogWrite(10, output);

    //Direcction motor left (backward)
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    //PWM-for motor left
    analogWrite(9, output);
     
  }
  else
  {
    output = abs(output);
    output = map(output, 10, 100, 40, 60);
    
    Serial.print(" ---   output_map:   ");
    Serial.println(output);
    
    //Direcction motor right (backward)
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    //PWM-for motor right
    analogWrite(10, output);

    //Direcction motor left (forward)
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    //PWM-for motor left
    analogWrite(9, output);
    
  }
  
}


void loop() {
  lightSensor_right = 0;
  lightSensor_left = 0;
  setPoint = 0;
  error = 0;

  //Constant for controller PID
  kp = 0.5;
  kd = 0.4;
  ki = 0.05;

  //MEASUREMENT PHASE
  lightSensor_right = analogRead(A0);
  lightSensor_left = analogRead(A1);
  
  //Re-mapping sensor reading for error offset 
  lightSensor_left += int(lightSensor_left/2.6);
  Serial.print("L_s: ");
  Serial.print(lightSensor_left);
  Serial.print(" ---  L_r: ");
  Serial.println(lightSensor_right);

  //ERROR
  //Error positive means turn in positive sense
  error = lightSensor_left -lightSensor_right;

  //CONTROL INPUT (Actuator phase)
  error = error * kp;
  //Actuator pahse
  Serial.print("error: ");
  Serial.print(error);
  outputController(error);
  
  
  delay(150);
  
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  //PWM-for motor right
  analogWrite(10, 50);
  
  //Direcction motor left (forward)
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  //PWM-for motor left
  analogWrite(9, 50);

  delay(200);

}
