#include <Fuzzy.h>
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

Fuzzy *fuzzy = new Fuzzy();

const int y1_HREG = 1, y2_HREG = 2, y3_HREG = 3;
const int y1d_HREG = 4, y2d_HREG = 5, y3d_HREG = 6;
const int y1dd_HREG = 7, y2dd_HREG = 8, y3dd_HREG = 9;
const int y1v_HREG = 10, y2v_HREG = 11, y3v_HREG = 12;
const int y1m_HREG = 13, y2m_HREG = 14, y3m_HREG = 15;
const int y1o_HREG = 16, y2o_HREG = 17, y3o_HREG = 18;
const int y1kp_HREG = 19, y2kp_HREG = 20, y3kp_HREG = 21;
const int y1ki_HREG = 22, y2ki_HREG = 23, y3ki_HREG = 24;
const int y1kd_HREG = 25, y2kd_HREG = 26, y3kd_HREG = 27;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo mr;
Servo ml;

int xi, ln;
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
double Kp = 0.5, Ki = 0.0020, Kd = 0.1;
double aPos, aVel;
//double Kp = 2, Ki = 0, Kd = 0.1;
int cte,x;

void setup() {
  Serial.begin(115200);

  FuzzyInput *error = new FuzzyInput(1);

  FuzzySet *baja = new FuzzySet(0, 1, 2, 3);
  error->addFuzzySet(baja);

  FuzzySet *mediabaja = new FuzzySet(2, 3, 3, 6);
  error->addFuzzySet(mediabaja);

  FuzzySet *media = new FuzzySet(3, 6, 54, 63);
  error->addFuzzySet(media);

  FuzzySet *mediaalta = new FuzzySet(54, 63, 63, 72);
  error->addFuzzySet(mediaalta);

  FuzzySet *alta = new FuzzySet(63, 72, 81, 90);
  error->addFuzzySet(alta);

  fuzzy->addFuzzyInput(error);

  FuzzyOutput *gain = new FuzzyOutput(1);

  FuzzySet *minima = new FuzzySet(0, 0.5, 1, 1.5);
  gain->addFuzzySet(minima);

  FuzzySet *mediominima = new FuzzySet(1, 1.5, 1.5, 2);
  gain->addFuzzySet(mediominima);

  FuzzySet *medio = new FuzzySet(1.5, 2, 2.5, 3);
  gain->addFuzzySet(medio);

  FuzzySet *mediomaxima = new FuzzySet(2.5, 3, 3, 3.5);
  gain->addFuzzySet(mediomaxima);

  FuzzySet *maxima = new FuzzySet(3.5, 4, 4.5, 5);
  gain->addFuzzySet(maxima);

  fuzzy->addFuzzyOutput(gain);

  FuzzyRuleAntecedent *ifErrorBaja = new FuzzyRuleAntecedent();
  ifErrorBaja->joinSingle(baja);
  FuzzyRuleConsequent *thenGainMinima = new FuzzyRuleConsequent();
  thenGainMinima->addOutput(minima);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifErrorBaja, thenGainMinima);

  fuzzy->addFuzzyRule(fuzzyRule1);


  FuzzyRuleAntecedent *ifErrorMediaBaja = new FuzzyRuleAntecedent();
  ifErrorMediaBaja->joinSingle(mediabaja);
  FuzzyRuleConsequent *thenGainMedioMinima = new FuzzyRuleConsequent();
  thenGainMedioMinima->addOutput(mediominima);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifErrorMediaBaja, thenGainMedioMinima);

  fuzzy->addFuzzyRule(fuzzyRule2);


  FuzzyRuleAntecedent *ifErrorMedia = new FuzzyRuleAntecedent();
  ifErrorMedia->joinSingle(media);
  FuzzyRuleConsequent *thenGainMedio = new FuzzyRuleConsequent();
  thenGainMedio->addOutput(medio);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifErrorMedia, thenGainMedio);

  fuzzy->addFuzzyRule(fuzzyRule3);


  FuzzyRuleAntecedent *ifErrorMediaAlta = new FuzzyRuleAntecedent();
  ifErrorMediaAlta->joinSingle(mediaalta);
  FuzzyRuleConsequent *thenGainMedioMaxima = new FuzzyRuleConsequent();
  thenGainMedioMaxima->addOutput(mediomaxima);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifErrorMediaAlta, thenGainMedioMaxima);

  fuzzy->addFuzzyRule(fuzzyRule4);


  FuzzyRuleAntecedent *ifErrorAlta = new FuzzyRuleAntecedent();
  ifErrorAlta->joinSingle(alta);
  FuzzyRuleConsequent *thenGainMaxima = new FuzzyRuleConsequent();
  thenGainMaxima->addOutput(maxima);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifErrorAlta, thenGainMaxima);

  fuzzy->addFuzzyRule(fuzzyRule5);

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

  initializeMB();

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
    delay(9000);
  mr.writeMicroseconds(1200);

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
  delay(9000);
  ml.writeMicroseconds(1200);
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

    InputR = aPos;
    InputL = aPos;
    
    x++;
    if (x == 360) x = 0;
    Setpoint = sin(deg2rad(x))*15;


    cte = 1200;
    OutputR = -sat(PID_R(), -200, 200) + cte;
    OutputL = sat(PID_L(), -200, 200) + cte;
    error = Setpoint - aPos;
    fuzzy->setInput(1, abs(error));
    fuzzy->fuzzify();
    double gain = fuzzy->defuzzify(1);
    Kp = gain*0.3;

    WriteStr();

    mr.writeMicroseconds(OutputR);
    ml.writeMicroseconds(OutputL);

    modbusW();
  }
}


void printEvent(sensors_event_t *event) {
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

  double output = Kp * error + Ki * cumError + Kd * rateError;  // calcular la salida del PID

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

  double output = Kp * errorL + Ki * cumErrorL + Kd * rateErrorL;  // calcular la salida del PID

  lastErrorL = errorL;           // almacenar error anterior
  previousTimeL = currentTimeL;  // almacenar el tiempo anterior

  return output;
}

double sat(double x, int ymin, int ymax) {
  double yout;
  if (x > ymax) yout = ymax;
  else if (x < ymin) yout = ymin;
  else yout = x;
  return yout;
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

double deg2rad(int deg) {
  double rad = deg * (PI / 180);
  return rad;
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



void initializeMB() {
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
  mb.addHreg(y1v_HREG, 0);
  mb.addHreg(y2v_HREG, 0);
  mb.addHreg(y3v_HREG, 0);
  mb.addHreg(y1m_HREG, 0);
  mb.addHreg(y2m_HREG, 0);
  mb.addHreg(y3m_HREG, 0);
  mb.addHreg(y1o_HREG, 0);
  mb.addHreg(y2o_HREG, 0);
  mb.addHreg(y3o_HREG, 0);
  mb.addHreg(y1kp_HREG, 0);
  mb.addHreg(y2kp_HREG, 0);
  mb.addHreg(y3kp_HREG, 0);
  mb.addHreg(y1ki_HREG, 0);
  mb.addHreg(y2ki_HREG, 0);
  mb.addHreg(y3ki_HREG, 0);
  mb.addHreg(y1kd_HREG, 0);
  mb.addHreg(y2kd_HREG, 0);
  mb.addHreg(y3kd_HREG, 0);
}


void modbusW() {
  mb.Hreg(y1_HREG, ExSign(Setpoint));
  mb.Hreg(y2_HREG, ExInt(Setpoint));
  mb.Hreg(y3_HREG, ExDec(Setpoint));
  mb.Hreg(y1d_HREG, ExSign(aPos));
  mb.Hreg(y2d_HREG, ExInt(aPos));
  mb.Hreg(y3d_HREG, ExDec(aPos));
  mb.Hreg(y1dd_HREG, ExSign(error));
  mb.Hreg(y2dd_HREG, ExInt(error));
  mb.Hreg(y3dd_HREG, ExDec(error));
  mb.Hreg(y1v_HREG, ExSign(aVel));
  mb.Hreg(y2v_HREG, ExInt(aVel));
  mb.Hreg(y3v_HREG, ExDec(aVel));
  mb.Hreg(y1m_HREG, ExSign(OutputL));
  mb.Hreg(y2m_HREG, ExInt(OutputL));
  mb.Hreg(y3m_HREG, ExDec(OutputL));
  mb.Hreg(y1o_HREG, ExSign(cte));
  mb.Hreg(y2o_HREG, ExInt(cte));
  mb.Hreg(y3o_HREG, ExDec(cte));
  mb.Hreg(y1kp_HREG, ExSign(Kp));
  mb.Hreg(y2kp_HREG, ExInt(Kp));
  mb.Hreg(y3kp_HREG, ExDec(Kp));
  mb.Hreg(y1ki_HREG, ExSign(Ki * 1000));
  mb.Hreg(y2ki_HREG, ExInt(Ki * 1000));
  mb.Hreg(y3ki_HREG, ExDec(Ki * 1000));
  mb.Hreg(y1kd_HREG, ExSign(Kd));
  mb.Hreg(y2kd_HREG, ExInt(Kd));
  mb.Hreg(y3kd_HREG, ExDec(Kd));
  mb.task();
}

