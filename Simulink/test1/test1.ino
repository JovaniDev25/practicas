#include <ESP32Encoder.h>

#define CLK 34  // CLK ENCODER
#define DT 35   // DT ENCODER

ESP32Encoder encoder;


unsigned long previousMillis = 0;
const long interval = 50;

int motor1Pin1 = 33;
int motor1Pin2 = 25;
int enable1Pin = 32;

int dutyCycle = 0;


const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

double Input, Output, Setpoint;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double Kp = 10, Ki = 0, Kd = 0;



typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;



FLOATUNION_t myValue;

long newPosition;


void setup() {
  Serial.begin(115200);

  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);


  //Serial.println("Hola Mundo!");
}

void loop() {


  newPosition = encoder.getCount() / 2;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    //myValue.number = sin(((float)millis() / 1000));  // Give your float a value
    myValue.number = newPosition;
    Setpoint = getFloat() * 30;
    // Print header: Important to avoid sync errors!
    Serial.write('A');

    // Print float data
    for (int i = 0; i < 4; i++) {
      Serial.write(myValue.bytes[i]);
    }
    // Print terminator
    Serial.print('\n');
  }

  Input = newPosition;
  Output = sat(PID(), -255, 255);
  dutyCycle = abs(Output);
  ledcWrite(pwmChannel, dutyCycle);
}

float getFloat() {
  int cont = 0;
  FLOATUNION_t f;
  while (cont < 4) {
    f.bytes[cont] = Serial.read();
    cont = cont + 1;
  }
  return f.number;
}

double PID() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error

  double output = Kp * error;  //+ Ki * cumError + Kd * rateError;

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
