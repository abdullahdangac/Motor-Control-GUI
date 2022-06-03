#define motorCW 11
#define motorCCW 10
#define EncA 2
#define EncB 4

#define potPin A0
float potValue = 0;

// ===== MOTOR =====
float motorValue = 0;
float setValue = 0;
float speedValue = 0;

// ===== ENCODER =====
volatile long int encoder_pos = 0;
long int prev_encoder_pos = 0;
unsigned long current_time;
unsigned long prev_time = 0;
float velocity = 0;

// ===== PID =====
float Kp = 10;
float Ki = 20;
float Kd = 5;
float P = 0;
float I = 0;
float D = 0;
float error = 0; 
float error_prev = 0;
float error_sum = 0;

// ===== DELTA TIME (dt) =====
unsigned long current_t;
unsigned long prev_t = 0;


float pid_control(float Kp, float Ki, float Kd, float error, float error_prev){
  current_t = millis();
  float dt = current_t - prev_t;
  prev_t = current_t;
   
  P = Kp * error;
  I = Ki * error_sum * dt;
  D = Kd * (error - error_prev) / dt;
  speedValue = P + I + D;

  error_prev = error;
  error_sum += error;

  return speedValue;
}


void readEncoder()
{
  //if (digitalRead(EncA) != digitalRead(EncB)){
  if(digitalRead(EncB) == LOW){
    encoder_pos++;
  } 
  else {
    encoder_pos--;
  }
  
  current_time = millis();
  velocity = (((float)encoder_pos - (float)prev_encoder_pos) * 60.0 * 1000000.0 / (24 * (current_time - prev_time))) / 1000;
  //velocity = ((float)encoder_pos - (float)prev_encoder_pos) * 10000 / (24 * (current_time - prev_time));
  //velocity = (((float)encoder_pos - (float)prev_encoder_pos) / 374.0) * 60.0 * (10000 / (current_time - prev_time));

  prev_encoder_pos = encoder_pos;
  prev_time = current_time;
  
  Serial.print("speed = ");
  Serial.println(velocity);
  //Serial.print("error = ");
  //Serial.println(error); 
}


void setup() {
  pinMode(motorCW, OUTPUT);
  pinMode(motorCCW, OUTPUT);
  pinMode(EncA, INPUT);            
  pinMode(EncB, INPUT);            
  attachInterrupt(digitalPinToInterrupt(EncA), readEncoder, RISING);
    
  Serial.begin (9600);
  Serial.println("start");           
}


void loop() {
  /*if(Serial.available()){  
     data = Serial.read();
  }*/
  
  setValue = 120.0;

  error = setValue - velocity;

  speedValue = pid_control(Kp, Ki, Kd, error, error_prev);

  if(speedValue > 166){ speedValue=165; }
  if(speedValue < 0){ speedValue=0; }

  motorValue = map(speedValue, 0, 166, 0, 255);

  analogWrite(motorCW, motorValue);
  digitalWrite(motorCCW, LOW);
}
