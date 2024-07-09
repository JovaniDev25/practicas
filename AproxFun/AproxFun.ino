#ifdef ESP8266
#include <ESP8266WiFi.h>
#else  //ESP32
#include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>
#include <ESP32Encoder.h>

#define CLK 34  // CLK ENCODER
#define DT 35   // DT ENCODER

ESP32Encoder encoder;

int motor1Pin1 = 33;
int motor1Pin2 = 25;
int enable1Pin = 32;
double position;


const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;


const int y1_HREG = 1, y2_HREG = 2, y1d_HREG = 3, y2d_HREG = 4, y1dd_HREG = 5, y2dd_HREG = 6;
ModbusIP mb;

unsigned long previousMillis = 0;
const long interval = 10;
float x, y, ym, yt;


double ut, yout, ymout;
double alfa = 0.05, xite = 0.5, b = 5, b2 = 2 * b * b;



double w[5], w1[5], w2[5], h[5], ha[5], dw[5], xi[2];
double ci[2][5] = {
  { -5, -2.5, 0, 2.5, 5 },
  { -10, -5, 0, 10, 5 }
};

double Input, Output, Setpoint;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double Kp = 10, Ki = 0, Kd = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Test.. Aproximador de Funciones");
  WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
  //WiFi.begin("ControlerAuto", "15022489");
  //WiFi.begin("CyberSphere2.4", "KD6JVB8kmCRe");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  mb.server();
  mb.addHreg(y1_HREG, 0);
  mb.addHreg(y2_HREG, 0);
  mb.addHreg(y1d_HREG, 0);
  mb.addHreg(y2d_HREG, 0);
  mb.addHreg(y1dd_HREG, 0);
  mb.addHreg(y2dd_HREG, 0);

  for (int i = 0; i <= 4; i++) {
    w[i] = random(15);
    w1[i] = w[i];
    w2[i] = w1[i];
  }
  for (int i = 0; i <= 4; i++) {
    Serial.println(ci[0][i]);
    Serial.println(ci[1][i]);
    Serial.println("-------");
  }

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);
  ut = -50;
}

void loop() {
  ReadStr();
  position = encoder.getCount() / 2;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    x++;
    /* if (x == 100) x = 0;
    ut =  sin( PI / x); */
    //+30
    if (x == 1000) {
      ut = 50;
    }
    if (x == 2000) {
      ut = -50;
      x = 0;
    }

    xi[1] = ut / 50;
    xi[2] = position / 50;

    for (int j = 0; j <= 4; j++) {
      double an1 = xi[1] - ci[0][j];
      double an2 = xi[2] - ci[1][j];
      double ex = pow(sqrt(pow(an1, 2) + pow(an2, 2)), 2) / b2;
      h[j + 1] = pow(2.7182, -ex);
    }

    double yi;
    for (int i = 1; i <= 5; i++) {
      yi = w[i] * h[i] + yi;
    }
    yout = position / 50;
    ymout = yi;
    //Serial.println(ymout);
    for (int i = 1; i <= 5; i++) {
      dw[i] = xite * h[i] * (yout - ymout);
    }

    for (int i = 1; i <= 5; i++) {
      w[i] = w1[i] + dw[i] + alfa * (w1[i] - w2[i]);
      w2[i] = w1[i];
      w1[i] = w[i];
    }

    /*     mb.Hreg(y1_HREG, ut);
    mb.Hreg(y2_HREG, ExDec(ut));
    mb.Hreg(y1d_HREG, position);
    mb.Hreg(y2d_HREG, ExDec(position));
    mb.Hreg(y1dd_HREG, ym);
    mb.Hreg(y2dd_HREG, ExDec(ymout)); */
    mb.Hreg(y1_HREG, xi[1]);
    mb.Hreg(y2_HREG, ExDec(xi[1]));
    mb.Hreg(y1d_HREG, xi[2]);
    mb.Hreg(y2d_HREG, ExDec(xi[2]));
    mb.Hreg(y1dd_HREG, ymout);
    mb.Hreg(y2dd_HREG, ExDec(ymout));
    mb.task();
  }
  Setpoint = ut;
  Input = position;
  Output = sat(PID(), -255, 255);
  dutyCycle = abs(int(Output));
  ledcWrite(pwmChannel, dutyCycle);
}

int ExDec(float Value) {
  int IntegerPart = (int)(Value);
  int DecimalPart = 10000 * (Value - IntegerPart);  //10000 b/c my float values always have exactly 4 decimal places
  return DecimalPart;
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
  /*if (output == 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
  }*/
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

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 'a' || str[0] == 'x' || str[0] == 'b' || str[0] == 's') {
      int ln = str.length();
      String nm = str.substring(1);
      //Serial.println(nm);
      if (str[0] == 'p') {
        Kp = nm.toFloat();
        Serial.print("Kp: ");
        Serial.println(Kp);
      }

      if (str[0] == 'i') {
        Ki = nm.toFloat();
        Serial.print("Ki: ");
        Serial.println(Ki);
      }
      if (str[0] == 'd') {
        Kd = nm.toFloat();
        Serial.print("Kd: ");
        Serial.println(Kd);
      }
      if (str[0] == 'a') {
        alfa = nm.toFloat();
        Serial.print("alfa: ");
        Serial.println(alfa);
      }
      if (str[0] == 'x') {
        xite = nm.toFloat();
        Serial.print("xite: ");
        Serial.println(xite);
      }
      if (str[0] == 'b') {
        b = nm.toFloat();
        Serial.print("b: ");
        Serial.println(b);
      }
    }
  }
}

/* double rad=deg2rad(deg)
  rad= deg*pi/180;
end */