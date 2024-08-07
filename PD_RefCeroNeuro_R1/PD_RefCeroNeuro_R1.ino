#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else  //ESP32
#include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MR_PIN 17
#define ML_PIN 16
int DELAY = 1000;

const int y1_HREG = 1, y2_HREG = 2, y3_HREG = 3, y1d_HREG = 4, y2d_HREG = 5, y3d_HREG = 6, y1dd_HREG = 7, y2dd_HREG = 8, y3dd_HREG = 9;  // motorR_HREG = 5, motorL_HREG = 6, ref_HREG = 7, kp_HREG = 8, ki_HREG = 9, kd_HREG = 10;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo mr;
Servo ml;

int ln, x;
bool fi;
String nm;

double InputR, InputL, OutputR, OutputL, Setpoint, sant;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
unsigned long currentTimeL, previousTimeL;
double elapsedTimeL;
double errorL, lastErrorL, cumErrorL, rateErrorL;
unsigned long previousMillis = 0;
const long interval = 100;

//double Kp = 0.4, Ki = 0.0001, Kd = 12;
double Kp = 2.3, Ki = 0.0002, Kd = 12;
double aPos, aVel;
//double Kp = 2, Ki = 0, Kd = 0.1;
int cte;
double position;
double ut, yout, ymout;
double alfa = 0.05, xite = 0.1, b = 5, b2 = 2 * b * b;
double w[5], w1[5], w2[5], h[5], ha[5], dw[5], xi[2];
double ci[2][5] = {
  { -5, -2.5, 0, 2.5, 5 },
  { -10, -5, 0, 10, 5 }
};


void setup() {
  Serial.begin(115200);
  WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
  //WiFi.begin("ControlerAuto", "15022489");

  //WiFi.begin("CyberSphere2.4", "KD6JVB8kmCRe");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();
  mb.addHreg(y1_HREG, 0);
  mb.addHreg(y2_HREG, 0);
  mb.addHreg(y3_HREG, 0);
  mb.addHreg(y1d_HREG, 0);
  mb.addHreg(y2d_HREG, 0);
  mb.addHreg(y3d_HREG, 0);
  mb.addHreg(y1dd_HREG, 0);
  mb.addHreg(y2dd_HREG, 0);
  mb.addHreg(y3dd_HREG, 0);
  /*   mb.addHreg(motorR_HREG, 0);
  mb.addHreg(motorL_HREG, 0);
  mb.addHreg(ref_HREG, 0);
  mb.addHreg(kp_HREG, 0);
  mb.addHreg(ki_HREG, 0);
  mb.addHreg(kd_HREG, 0); */

  while (!Serial) delay(10);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

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

  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  /////////////////////////////////////////////
  mr.attach(MR_PIN);
  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  Serial.println("..wait 2 seconds and press any key.");
  mr.writeMicroseconds(MAX_SIGNAL);

  Setpoint = 0;
  delay(1000);

  while (!Serial.available())
    ;
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");
  Serial.print(MIN_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  mr.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(8000);

  /////////////////////////////////////////////
  ml.attach(ML_PIN);
  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  Serial.println("..wait 2 seconds and press any key.");
  ml.writeMicroseconds(MAX_SIGNAL);

  Setpoint = 0;
  delay(1000);

  while (!Serial.available())
    ;
  Serial.read();

  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");
  Serial.print(MIN_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  ml.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(8000);
  Serial.println("Iniciando Rutina...");
}

void loop() {
  ReadStr();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sensors_event_t orientationData, angVelocityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    printEvent(&orientationData);
    printEvent(&angVelocityData);

    ut = sin(deg2rad(x));
    x++;
    if (x == 360) x = 0;
    Setpoint = ut * 30;
    InputR = aPos;
    InputL = aPos;
    position = aPos;
    //xi = ramp(0, 200, xi);
    cte = 1200;
    OutputR = -sat(PID_R(), -200, 200) + cte;
    OutputL = sat(PID_L(), -200, 200) + cte;

    WriteStr();

    xi[1] = ut;
    xi[2] = position / 30;

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



    mr.writeMicroseconds(OutputR);
    ml.writeMicroseconds(OutputL);
    /* mb.Hreg(y1_HREG, ExInt(aPos));
    mb.Hreg(y2_HREG, ExDec(aPos));
    mb.Hreg(y1d_HREG, Setpoint);
    mb.Hreg(y2d_HREG, ExDec(Setpoint));
    mb.Hreg(ydd1_HREG, ExSign(ymout));
    mb.Hreg(ydd2_HREG, ExInt(ymout));
    mb.Hreg(ref_HREG, Setpoint);
    mb.Hreg(kp_HREG, Kp);
    mb.Hreg(ki_HREG, Ki);
    mb.Hreg(kd_HREG, Kd); */

    mb.Hreg(y1_HREG, ExSign(xi[1]));
    mb.Hreg(y2_HREG, ExInt(xi[1]));
    mb.Hreg(y3_HREG, ExDec(xi[1]));
    mb.Hreg(y1d_HREG, ExSign(xi[2]));
    mb.Hreg(y2d_HREG, ExInt(xi[2]));
    mb.Hreg(y3d_HREG, ExDec(xi[2]));
    mb.Hreg(y1dd_HREG, ExSign(ymout));
    mb.Hreg(y2dd_HREG, ExInt(ymout));
    mb.Hreg(y3dd_HREG, ExDec(ymout));

    mb.task();
  }
}


void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    aPos = y;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    aVel = y;
  } else {
    Serial.print("Unk:");
  }
}

int ramp(int li, int ls, int xa) {
  int y = xa;
  if (xa < ls && fi == false) y++;
  if (xa == ls) fi = true;
  //if (xa > li && fi == true) y--;
  //if (xa == li) fi = false;
  return y;
}

double PID_R() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - InputR;                      // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error

  double output = Kp * error + Kd * rateError;  //+ Ki * cumError;  // calcular la salida del PID

  lastError = error;           // almacenar error anterior
  previousTime = currentTime;  // almacenar el tiempo anterior
  return output;
}

double PID_L() {
  currentTimeL = millis();                               // obtener el tiempo actual
  elapsedTimeL = (double)(currentTime - previousTimeL);  // calcular el tiempo transcurrido

  errorL = Setpoint - InputL;                        // determinar el error entre la consigna y la medición
  cumErrorL += errorL * elapsedTimeL;                // calcular la integral del error
  rateErrorL = (errorL - lastErrorL) / elapsedTime;  // calcular la derivada del error

  double output = Kp * errorL + Kd * rateErrorL;  //+ Ki * cumErrorL;  // calcular la salida del PID

  lastErrorL = errorL;           // almacenar error anterior
  previousTimeL = currentTimeL;  // almacenar el tiempo anterior

  return output;
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
void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    //if (str == "0") paso = 0;
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 's') {
      ln = str.length();
      nm = str.substring(1);
      if (str[0] == 'p') Kp = nm.toFloat();
      if (str[0] == 'i') {
        Ki = 0.0001 * nm.toFloat();
        cumErrorL = 0;
        cumError = 0;
      }
      if (str[0] == 'd') Kd = nm.toFloat();
      if (str[0] == 's') {
        Setpoint = nm.toInt();
      }
    }
  }
}


void WriteStr() {
  Serial.print("aPos:  ");
  Serial.print(aPos);
  Serial.print("   aVel:  ");
  Serial.println(aVel);
  Serial.print("Ref:  ");
  Serial.print(Setpoint);
  Serial.print("   ePos:  ");
  Serial.println(error);
  Serial.print("escR:  ");
  Serial.print(OutputR);
  Serial.print("   escL:  ");
  Serial.println(OutputL);

  Serial.print("Kp:  ");
  Serial.print(Kp);
  Serial.print("   Kd:  ");
  Serial.print(Kd);
  Serial.print("   Ki:  ");
  Serial.println(Ki * 10000);
  Serial.println("---------------");
}

double sat(double x, int ymin, int ymax) {
  double yout;
  if (x > ymax) yout = ymax;
  else if (x < ymin) yout = ymin;
  else yout = x;
  return yout;
}

double deg2rad(int deg) {
  double rad = deg * (PI / 180);
  return rad;
}