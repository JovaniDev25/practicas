#include <Fuzzy.h>
#include <ESP32Encoder.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else  //ESP32
#include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

#define CLK 34
#define DT 35

ModbusIP mb;
Fuzzy *fuzzy = new Fuzzy();
ESP32Encoder encoder;


const int pos_HREG = 1, ref_HREG = 2, kp_HREG = 3, out_HREG = 4;
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
double Kp = 0, Ki = 0.001, Kd = 5;

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

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);
  Serial.begin(115200);



  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  error = Setpoint - newPosition;

  FuzzyInput *error = new FuzzyInput(1);

  FuzzySet *baja = new FuzzySet(0, 36, 72, 108);
  error->addFuzzySet(baja);

  FuzzySet *mediabaja = new FuzzySet(72, 108, 108, 144);
  error->addFuzzySet(mediabaja);

  FuzzySet *media = new FuzzySet(108, 144, 180, 216);
  error->addFuzzySet(media);

  FuzzySet *mediaalta = new FuzzySet(180, 216, 252, 288);
  error->addFuzzySet(mediaalta);

  FuzzySet *alta = new FuzzySet(252, 288, 324,360);
  error->addFuzzySet(alta);

  fuzzy->addFuzzyInput(error);


  FuzzyOutput *gain = new FuzzyOutput(1);

  FuzzySet *minima = new FuzzySet(0, 2, 6, 8);
  gain->addFuzzySet(minima);

  FuzzySet *mediominima = new FuzzySet(6, 8, 8, 10);
  gain->addFuzzySet(mediominima);

  FuzzySet *medio = new FuzzySet(8, 10, 14, 16);
  gain->addFuzzySet(medio);

  FuzzySet *mediomaxima = new FuzzySet(14, 16, 16, 18);
  gain->addFuzzySet(mediomaxima);

  FuzzySet *maxima = new FuzzySet(16, 18, 24, 25);
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

  mb.server();
  mb.addHreg(pos_HREG, 0);
  mb.addHreg(ref_HREG, 0);
  mb.addHreg(kp_HREG, 0);
  mb.addHreg(out_HREG, 0);

  Serial.print("Testing Encoder/Motor...");
}

void loop() {

  ReadStr();
  newPosition = encoder.getCount() / 2;
  Input = newPosition;

  error = Setpoint - newPosition;
  fuzzy->setInput(1, abs(error));
  fuzzy->fuzzify();
  double gain = fuzzy->defuzzify(1);
  Kp = gain;

  //Output = sat(PID(), -255, 255);
  Output = PID();
  dutyCycle = abs(Output);
  ledcWrite(pwmChannel, dutyCycle);

  mb.Hreg(pos_HREG, newPosition);
  mb.Hreg(ref_HREG, Setpoint);
  mb.Hreg(kp_HREG, Kp);
  mb.Hreg(out_HREG, dutyCycle);
  mb.task();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    WriteStr();
  }
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    //Serial.println(str);
    //if (str == "0") paso = 0;
    if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 's' || str[0] == 'f' || str[0] == 'b' || str[0] == 'c') {
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

void WriteStr() {
  Serial.print("Ref:   ");
  Serial.print(Setpoint);
  Serial.print("    Retro:   ");
  Serial.print(newPosition);
  Serial.print("    err:   ");
  Serial.print(error);
  Serial.print("    Kp:   ");
  Serial.print(Kp);
  Serial.print("    Out:   ");
  Serial.println(dutyCycle);
  Serial.println("----------");
}

double PID() {
  currentTime = millis();                              // obtener el tiempo actual
  elapsedTime = (double)(currentTime - previousTime);  // calcular el tiempo transcurrido

  error = Setpoint - Input;                       // determinar el error entre la consigna y la medición
  cumError += error * elapsedTime;                // calcular la integral del error
  rateError = (error - lastError) / elapsedTime;  // calcular la derivada del error

  double output = Kp * error + Ki * cumError;  //+ Ki * cumError + Kd * rateError;

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