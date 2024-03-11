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

const int y1_HREG = 1, y2_HREG = 2, y1d_HREG = 3, y2d_HREG = 4, motorR_HREG = 5, motorL_HREG = 6;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Servo mr;
Servo ml;

int xi, ln;
bool fi;
String nm;

double Input, OutputR, OutputL, Setpoint;
unsigned long currentTime, previousTime;
unsigned long previousMillis = 0;
const long interval = 100;
double aPos, aVel;

void setup() {
  Serial.begin(115200);
  //WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
  WiFi.begin("ControlerAuto", "15022489");

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
  mb.addHreg(y1d_HREG, 0);
  mb.addHreg(y2d_HREG, 0);
  mb.addHreg(motorR_HREG, 0);
  mb.addHreg(motorL_HREG, 0);



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
}

void loop() {
  ReadStr();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    printEvent(&orientationData);
    printEvent(&angVelocityData);

    xi = ramp(0, 200, xi);
    OutputR = 1000 + xi;
    OutputL = OutputR;
    mr.writeMicroseconds(OutputR);
    ml.writeMicroseconds(OutputL);
    mb.Hreg(y1_HREG, aPos);
    mb.Hreg(y2_HREG, ExDec(aPos));
    mb.Hreg(y1d_HREG, aVel);
    mb.Hreg(y2d_HREG, ExDec(aVel));
    mb.Hreg(motorR_HREG, OutputR);
    mb.Hreg(motorL_HREG, OutputL);
    mb.task();
  }
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    //if (str == "0") paso = 0;
    if (str[0] == 'r' || str[0] == 'l') {
      ln = str.length();
      nm = str.substring(1);
      if (str[0] == 'r') OutputR = nm.toInt() + 1000;
      if (str[0] == 'l') OutputL = nm.toInt() + 1000;
    };
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    aPos = y;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    aVel = y;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.print(z);
  Serial.print(" |\t ESCR= ");
  Serial.print(OutputR);
  Serial.print(" |\t ESCL= ");
  Serial.println(OutputL);
}

int ExDec(float Value) {
  int IntegerPart = (int)(Value);
  int DecimalPart = 10000 * (Value - IntegerPart);  //10000 b/c my float values always have exactly 4 decimal places
  return DecimalPart;
}

int ramp(int li, int ls, int xa) {
  int y = xa;
  if (xa < ls && fi == false) y++;
  if (xa == ls) fi = true;
  if (xa > li && fi == true) y--;
  if (xa == li) fi = false;
  return y;
}
