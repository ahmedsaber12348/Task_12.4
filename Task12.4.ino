#define ENCA 3
#define ENCB 2 
#define PWM_Signal 5
#define IN2 8
#define IN1 9

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float smoothedPwr = 0;  // New variable for the smoothed power
float alpha = 0.1;      

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  pinMode(PWM_Signal, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.println("target pos");
}

void loop() {
  int target = 2000;
  float kp = 1;
  float kd = 0.25;
  float ki = 0.05;
  
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e8);
  prevT = currT;
  
  int e = target - pos;
  float dedt = (e - eprev) / deltaT;
  eintegral = eintegral + e * deltaT;
  
  float u = kp * e + kd * dedt + ki * eintegral;
  float pwr = fabs(u);
  
  if (pwr > 255) {
    pwr = 255;
  }
  
  // Apply Exponential Smoothing Filter
  smoothedPwr = alpha * pwr + (1 - alpha) * smoothedPwr;
  
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  
  setMotor(dir, smoothedPwr, PWM_Signal, IN1, IN2);

  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }  
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}
