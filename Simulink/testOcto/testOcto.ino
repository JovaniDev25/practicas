#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else  //ESP32
#include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

#include <ESP32Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define M1_PIN 4
#define M2_PIN 32
#define M3_PIN 25
#define M4_PIN 27
#define M5_PIN 23
#define M6_PIN 19
#define M7_PIN 18
#define M8_PIN 26
int DELAY = 1000;


const int y1_HREG = 1, y2_HREG = 2, y3_HREG = 3;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo m1;
Servo m2;
Servo m3;
Servo m4;
Servo m5;
Servo m6;
Servo m7;
Servo m8;

unsigned long previousMillis = 0;
const long interval = 100;
double aPos, aVel;
int Offset;

double Input, Output, Setpoint;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double Kp = 2, Ki = 0, Kd = 0.1;

void setup() {

  Serial.begin(115200);
  //WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
  //WiFi.begin("ControlerAuto", "15022489");
  //WiFi.begin("CyberSphere2.4", "KD6JVB8kmCRe");
  WiFi.begin("JovaniWifi", "Jov250397");
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

  while (!Serial) delay(10);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1500);
  Serial.println("Iniciando Programa...");
  delay(1000);
  Serial.println("Calibrando ESC.....");

  /////////////////////////////////////////////
  /*m1.attach(M1_PIN);
  m2.attach(M2_PIN);
  m3.attach(M3_PIN);
  m4.attach(M4_PIN);

  m5.attach(M5_PIN);
  m6.attach(M6_PIN);
  m7.attach(M7_PIN);
  m8.attach(M8_PIN); */

  /* const int freq = 5000;
  const int ledChannel = 0;
  const int resolution = 8;
  ledcSetup(ledChannel, freq, resolution);
 */

  m1.attach(M1_PIN);
  m2.attach(M2_PIN);
  m3.attach(M3_PIN);
  m4.attach(M4_PIN);

  m5.attach(M5_PIN);
  m6.attach(M6_PIN);
  m7.attach(M7_PIN);
  m8.attach(M8_PIN);

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("  Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m1.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m1.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m1.writeMicroseconds(1200);

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m2.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m2.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m2.writeMicroseconds(1200);

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m3.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m3.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m3.writeMicroseconds(1200);


  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m4.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m4.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m4.writeMicroseconds(1200);

  ///////////////////////////

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m5.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m5.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m5.writeMicroseconds(1200);

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m6.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m6.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m6.writeMicroseconds(1200);

  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m7.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m7.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m7.writeMicroseconds(1200);


  Serial.print("Valor: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("   Maximo... )");
  Serial.print("\n");
  Serial.println("..En espera.");
  m8.writeMicroseconds(MAX_SIGNAL);
  delay(1000);
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Enviando Salida: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("   Minima)");
  Serial.print("\n");
  m8.writeMicroseconds(MIN_SIGNAL);
  Serial.println("ESC Calibrado");
  Serial.println("----");
  delay(8000);
  m8.writeMicroseconds(1200);
  //delay(1000);

  ////////////////
  Offset = 1400;
  Setpoint = 150;
  Serial.println("Iniciando Secuencia... Escribir ox");
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
    WriteStr();
    m1.writeMicroseconds(Offset - Output);
    m2.writeMicroseconds(Offset - Output);
    m3.writeMicroseconds(Offset + Output);
    m4.writeMicroseconds(Offset + Output);

    m5.writeMicroseconds(Offset - Output);
    m6.writeMicroseconds(Offset - Output);
    m7.writeMicroseconds(Offset + Output);
    m8.writeMicroseconds(Offset + Output);

    if (aPos < 180) Input = 180 - aPos;
    else Input = 180 + (360 - aPos);

    //Input = 180 - aPos;
    Output = sat(PID(), -250, 250);

    mb.Hreg(y1_HREG, ExDec(aPos));
    mb.Hreg(y2_HREG, ExDec(aVel));
    mb.Hreg(y3_HREG, ExDec(Output));
    mb.task();
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ORIENTATION) {

    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    aPos = x;

  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    aVel = x;
    //Serial.println(aVel);
  } else {
    Serial.print("Unk:");
  }
}

int ExInt(float Value) {
  int IntegerPart = (int)(Value);
  return IntegerPart;
}


int ExDec(float Value) {
  int IntegerPart = (int)(Value);
  int DecimalPart = 10000 * (Value - IntegerPart);  //10000 b/c my float values always have exactly 4 decimal places
  return DecimalPart;
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    //if (str == "0") paso = 0;
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 'o' || str[0] == 's') {
      int ln = str.length();
      String nm = str.substring(1);
      if (str[0] == 'o') {
        Offset = nm.toInt();
        Serial.print("Out:   ");
        Serial.println(Offset);
      };
      if (str[0] == 's') {
        Setpoint = nm.toInt();
        Serial.print("Setpoint:   ");
        Serial.println(Setpoint);
      };
      if (str[0] == 'p') {
        Kp = nm.toFloat();
        Serial.print("Kp:   ");
        Serial.println(Kp);
      }
      if (str[0] == 'i') {
        Ki = 0.00001 * nm.toFloat();
        cumError = 0;
        Serial.print("Ki:   ");
        Serial.println(Ki);
      }
      if (str[0] == 'd') {
        Kd = nm.toFloat();
        Serial.print("Kd:   ");
        Serial.println(Kd);
      }
    }
  }
}

void WriteStr() {
  Serial.print("Ref:  ");
  Serial.println(Setpoint);
  Serial.print("Retro:  ");
  Serial.println(Input);
  Serial.print("error:  ");
  Serial.println(error);
  Serial.print("Out:  ");
  Serial.println(Output);
  Serial.println("-------");
}


double PID() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error
  if (isnan(rateError)) {
    rateError = 0;
  } else {
    rateError = rateError;
  }
  double output = Kp * error + Kd * rateError + Ki * cumError;

  lastError = error;           // almacenar error anterior
  previousTime = currentTime;  // almacenar el tiempo anterior

  return output;
}

double sat(double x, int ymin, int ymax) {
  double yout;
  if (x > ymax) yout = ymax;
  else if (x < ymin) yout = ymin;
  else yout = x;
  return yout;
}



unsigned long previousMillis = 0;
const long interval = 10;

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t myValue1;
FLOATUNION_t myValue2;

void setup() {
  Serial.begin(115200);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    myValue1.number = getFloat();
    myValue2.number = getFloat();
    // Print header: Important to avoid sync errors!
    Serial.write('A');

    // Print float data
    for (int i = 0; i < 4; i++) {
      Serial.write(myValue1.bytes[i]);
    }
    for (int i = 0; i < 4; i++) {
      Serial.write(myValue2.bytes[i]);
    }
    // Print terminator
    Serial.print('\n');
  }
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