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

const int y1_HREG = 1, y2_HREG = 2, y1d_HREG = 3, y2d_HREG = 4, motorR_HREG = 5, motorL_HREG = 6;
ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

unsigned long previousMillis = 0;
const long interval = 100;

double aPos, aVel;
double InputR, InputL, OutputR, OutputL, Setpoint; 
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

    //Serial.println(aPos);
    // Serial.println(aVel);
    //Serial.println(ExDec(orientationData.orientation.y));
    mb.Hreg(y1_HREG, aPos);
    mb.Hreg(y2_HREG, ExDec(aPos));
    mb.Hreg(y1d_HREG, aVel);
    mb.Hreg(y2d_HREG, ExDec(aVel));
    mb.Hreg(motorR_HREG, OutputR);
    mb.Hreg(motorL_HREG, OutputL);
    mb.task();
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem

  if (event->type == SENSOR_TYPE_ORIENTATION) {
    //Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    aPos = y;
  } /* else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    
  }  */
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    aVel = y;
  } else {
    Serial.print("Unk:");
  }
  /* 
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z); */
  /*
  Serial.print(" |\t ESCR= ");
  Serial.print(OutputR);
  Serial.print(" |\t ESCL= ");
  Serial.println(OutputL); */
}

int ExDec(float Value) {
  int IntegerPart = (int)(Value);
  int DecimalPart = 10000 * (Value - IntegerPart);  //10000 b/c my float values always have exactly 4 decimal places
  return DecimalPart;
  //Serial.println (DecimalPart);
}

void ReadStr() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    Serial.println(str);
    //if (str == "0") paso = 0;
    /* if (str[0] == 'p' || str[0] == 'i' || str[0] == 'd' || str[0] == 't') {
      ln = str.length();
      nm = str.substring(1);
      if (str[0] == 'p') Kp = nm.toFloat();
      //if (str[0] == 'i') Ki = nm.toFloat();
      if (str[0] == 'd') Kd = nm.toFloat();
      if (str[0] == 't') Setpoint = nm.toInt();
    };
    //Serial.println(paso);
  } */
  }
}