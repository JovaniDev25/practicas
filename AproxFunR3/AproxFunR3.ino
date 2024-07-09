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


const int y1_HREG = 1, y2_HREG = 2, y3_HREG = 3;  //y1d_HREG = 3, y2d_HREG = 4, y1dd_HREG = 5, y2dd_HREG = 6;
const int yd1_HREG = 4, yd2_HREG = 5, yd3_HREG = 6;
const int ydd1_HREG = 7, ydd2_HREG = 8, ydd3_HREG = 9;

ModbusIP mb;

unsigned long previousMillis = 0;
const long interval = 20;
float x, y, ym, yt;


double ut, yout, ymout;
double alfa = 0.05, xite = 0.1, b = 5, b2 = 2 * b * b;



double w[5], w1[5], w2[5], h[5], ha[5], dw[5], xi[2];
double ci[2][5] = {
  { -5, -2.5, 0, 2.5, 5 },
  { -10, -5, 0, 10, 5 }
};

double Input, Output, Setpoint;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double Kp = 10, Ki = 2, Kd = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("Test.. Aproximador de Funciones");
  //WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
  WiFi.begin("ControlerAuto", "15022489");
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
  mb.addHreg(y3_HREG, 0);
  mb.addHreg(yd1_HREG, 0);
  mb.addHreg(yd2_HREG, 0);
  mb.addHreg(yd3_HREG, 0);
  mb.addHreg(ydd1_HREG, 0);
  mb.addHreg(ydd2_HREG, 0);
  mb.addHreg(ydd3_HREG, 0);


  for (int i = 0; i <= 4; i++) {
    w[i] = random();
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
  //ut = 1;
}

void loop() {
  ReadStr();
  position = encoder.getCount() / 2;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    x++;
    if (x == 360) x = 0;
    ut = sin(deg2rad(x));
    //ydd = -sin(deg2rad(x));
    /*
    if (x == 1000) {
      ut = 1;
    }
    if (x == 2000) {
      ut = 0;
      x = 0;
    }
*/
    xi[1] = ut;
    xi[2] = position/30;

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
    yout = position / 30;
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



    /* mb.Hreg(y1_HREG, ExSign(xi[1]));
    mb.Hreg(y2_HREG, ExInt(xi[1]));
    mb.Hreg(y3_HREG, ExDec(xi[1]));
    mb.Hreg(yd1_HREG, ExSign(xi[2]));
    mb.Hreg(yd2_HREG, ExInt(xi[2]));
    mb.Hreg(yd3_HREG, ExDec(xi[2]));
    mb.Hreg(ydd1_HREG, ExSign(ymout));
    mb.Hreg(ydd2_HREG, ExInt(ymout));
    mb.Hreg(ydd3_HREG, ExDec(ymout));

    mb.task(); */
  }
  Setpoint = ut * 30;
  Input = position;
  Output = sat(PID(), -255, 255);
  dutyCycle = abs(int(Output));
  ledcWrite(pwmChannel, dutyCycle);
}

int ExDec(float Value) {
  int DecimalPart;
  if (Value != 0) {
    int IntegerPart = (int)(Value);
    DecimalPart = 100 * (Value - IntegerPart);
    DecimalPart = abs(DecimalPart);
  } else DecimalPart = 0;
  return DecimalPart;
}


int ExInt(float Value) {
  int IntegerPart = (int)(Value);
  int DecimalPart = 10000 * (Value - IntegerPart);
  IntegerPart = abs(IntegerPart);
  return IntegerPart;
}

int ExSign(float Value) {
  int Sign;
  if (Value > 0) Sign = 1;
  else Sign = -1;
  return Sign;
}

double PID() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  //(error );// elapsedTime;  // calcular la derivada del error
   if (isnan(rateError)) {
    rateError = 0;
  } else {
    rateError = rateError;
  } 
  double output = Kp * error; //+ Kd * rateError;
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

double deg2rad(int deg) {
  double rad = deg * (PI / 180);
  return rad;
}
