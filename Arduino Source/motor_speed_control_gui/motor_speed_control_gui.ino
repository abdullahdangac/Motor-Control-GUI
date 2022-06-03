#define motorCW 11
#define motorCCW 10
#define EncA 2
#define EncB 4

// ===== MOTOR =====
bool isStart = false;
float setValue = 0;
float speedValue = 0;
float motorValue = 0;

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


/*union converter{
  int32_t integerValue;
  uint8_t byteArray[4];
}convert;*/


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
  
  //Serial.print("speed = ");
  //Serial.println(velocity);
}


/*void sendToPC(double* data1, double* data2, double* data3){
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}*/


void getSerialData(){
  while ((Serial.available()>1)){
    char input = Serial.read();
    String tmp = "";
    switch(input){
      case 'P':
        tmp = getVal();
        if (tmp != "X")
        {
          Kp = tmp.toFloat();
        }
        break;
      case 'I':
        tmp = getVal();
        if (tmp != "X")
        {
          Ki = tmp.toFloat();
        }
        break;
      case 'D':
        tmp = getVal();
        if (tmp != "X")
        {
          Kd = tmp.toFloat();
        }
        break;
      case 'S':
        tmp = getVal();
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        if (tmp != "X"){
          setValue = tmp.toFloat();
        }
        break;
      case 'R':
        startStopMotor();
        break;
      default:
        break;
    }
  }
}


String getVal(){
  String recieveString = "";
  while (Serial.available()){
    char input = Serial.read();
    if (input == '%'){   
      return recieveString;
    }
    recieveString += input;
  }
  return "X";   // failed to receive
}


void startStopMotor(){
  isStart = !isStart;
}


void setup() {
  pinMode(motorCW, OUTPUT);
  pinMode(motorCCW, OUTPUT);
  pinMode(EncA, INPUT);             
  pinMode(EncB, INPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EncA), readEncoder, RISING);
    
  Serial.begin (9600);
  //Serial.println("start");           
}


void loop() {
  if(Serial.available()){
    getSerialData();
  }

  setValue = 130.0;

  error = setValue - velocity;

  speedValue = pid_control(Kp, Ki, Kd, error, error_prev);

  if(speedValue > 166){ speedValue=165; }
  if(speedValue < 0){ speedValue=0; }

  if(isStart){   
    motorValue = map(speedValue, 0, 166, 0, 255);
    digitalWrite(12, HIGH);
  }
  else{
    motorValue = 0;
    digitalWrite(12, LOW);
  }

  //double data1 = setValue;
  //double data2 = velocity;
  //double data3 = error;
  //sendToPC(&data1, &data2, &data3);

  analogWrite(motorCW, motorValue);
  digitalWrite(motorCCW, LOW);
}
