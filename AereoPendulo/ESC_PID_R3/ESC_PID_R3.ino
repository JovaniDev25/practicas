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
#define M1_PIN 16
#define M2_PIN 17

int DELAY = 1000;

const int y_HREG = 1, motor_HREG = 2;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo motorCW;
Servo motorCCW;

/*
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int m1Pin = 16;
int m2Pin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int m1Pin = 16;
int m2Pin = 17;
#else
int m1Pin = 16;
int m2Pin = 17;
#endif */

// Constantes del controlador
double Kp = 0.4, Ki = 0.0001, Kd = 12;
int xi, ln;
String nm;
// variables externas del controlador
double Input, Output1,Output2, Setpoint;

// variables internas del controlador
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
unsigned long previousMillis = 0;
const long interval = 10;

int paso;
int cteCW,cteCCW, angulo = 90;

void setup() {
  Setpoint = 45;
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
  mb.addHreg(y_HREG, 0);
  mb.addHreg(motor_HREG, 0);



  while (!Serial) delay(10);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  motorCW.attach(M1_PIN);
  motorCCW.attach(M2_PIN);

  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  Serial.println("..wait 2 seconds and press any key.");
  motorCW.writeMicroseconds(MAX_SIGNAL);
  motorCCW.writeMicroseconds(MAX_SIGNAL);
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
  motorCW.writeMicroseconds(MIN_SIGNAL);
  motorCCW.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(8000);
  paso = 1;
}

void loop() {

  ReadStr();
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    Input = orientationData.orientation.y + 87;

    if (paso == 0) {
      Output1 = 0;
      Output2 = 0;
    }
    if (paso == 1) {
      InitialPos(angulo);
      Output1 = 1000 + xi;
      Output2 = 1000 - xi;
    }
    if (paso == 2) {
      Output1 = 0;
      Output2 = 0;
    }
    //Output =computePID(Input) + cte;

    motorCW.writeMicroseconds(Output1);
    motorCCW.writeMicroseconds(Output2);
    mb.Hreg(y_HREG, Input);
    mb.Hreg(motor_HREG, Output1);
    mb.task();
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\sp= ");
  Serial.print(Setpoint);
  Serial.print(" |y= ");
  Serial.print(y);
  Serial.print(" |er= ");
  Serial.print(error);
  /*   Serial.print(" |\tz= ");
  Serial.println(z); */
  Serial.print(" |ESC1= ");
  Serial.print(Output1);
  Serial.print(" |ESC2= ");
  Serial.println(Output2);
  /* Serial.print(" |\t Kp= ");
  Serial.print(Kp);
  Serial.print(" |\t Ki= ");
  Serial.print(Ki);
  Serial.print(" |\t Kd= ");
  Serial.println(Kd); */
}

double computePID(double inp) {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error

  double output = Kp * error + Ki * cumError + Kd * rateError;  // calcular la salida del PID

  lastError = error;           // almacenar error anterior
  previousTime = currentTime;  // almacenar el tiempo anterior

  return output;
}

void InitialPos(int th) {
  if (Input < th) xi++;
  if (Input > th) {
    //cte = xi + 1000;
    xi--;
    //paso = 2;
  }
  cteCW = xi + 1000;
  cteCCW = -xi + 1000;
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    if (str == "0") paso = 0;
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 't') {
      ln = str.length();
      nm = str.substring(1);
      if (str[0] == 'p') Kp = nm.toFloat();
      if (str[0] == 'i') Ki = nm.toFloat();
      if (str[0] == 'd') Kd = nm.toFloat();
      if (str[0] == 't') Setpoint = nm.toInt();
    };
    Serial.println(paso);
  }
}