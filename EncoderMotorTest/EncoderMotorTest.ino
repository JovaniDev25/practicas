#include <ESP32Encoder.h>

#define CLK 35  // CLK ENCODER
#define DT 34   // DT ENCODER

ESP32Encoder encoder;

int motor1Pin1 = 33;
int motor1Pin2 = 25;
int enable1Pin = 32;
long newPosition;
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;
unsigned long previousMillis = 0;
const long interval = 100;

double Input, Output, Setpoint;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double Kp = 2, Ki = 0.001, Kd = 0;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);
  Serial.begin(115200);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.print("Testing Encoder/Motor...");
}

void loop() {

  ReadStr();
  newPosition = encoder.getCount() / 2;
  //
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Ref:   ");
    Serial.print(Setpoint);
    Serial.print("    Retro:   ");
    Serial.print(newPosition);
    Serial.print("    Out:   ");
    Serial.println(Output);
    Serial.println("----------");
  }
  ledcWrite(pwmChannel, Output);
  //dutyCycle = 200;
  Input = newPosition;
  Output = sat(PID(), -255, 255);
  dutyCycle = Output;
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    //Serial.println(str);
    //if (str == "0") paso = 0;
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 's' || str[0] == 'f' || str[0] == 'b'|| str[0] == 'c') {
      int ln = str.length();
      String nm = str.substring(1);
      //Serial.println(nm);
      if (str[0] == 'p') Kp = nm.toFloat();
      if (str[0] == 'i') Ki = nm.toFloat();
      if (str[0] == 'd') Kd = nm.toFloat();
      if (str[0] == 's') Setpoint = nm.toFloat();
      if (str[0] == 'c') {
        encoder.setCount(nm.toInt());
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
      }
      if (str[0] == 'b') {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
      }
      if (str[0] == 'f') {
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
      }
    }
  }
}


double PID() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error

  double output = Kp * error+ Ki * cumError + Kd * rateError;  

  lastError = error;           // almacenar error anterior
  previousTime = currentTime;  // almacenar el tiempo anterior

  if (output < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  if (output == 0) {
    //digitalWrite(motor1Pin1, LOW);
    //digitalWrite(motor1Pin2, LOW);
  }
  if (output > 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  return output;
}

double sat(double x, int ymin, int ymax) {
  double yout;
  if (x > ymax) yout = ymax;
  else if (x < ymin) yout = ymin;
  else yout = x;
  return yout;
}
