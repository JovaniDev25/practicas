#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <ESP32Servo.h>
//#include <Servo.h>


const int x1_HREG = 1, x2_HREG = 2, y1_HREG = 3, y2_HREG = 4, z1_HREG = 5, z2_HREG = 6;
const int sm1fb_HREG = 7, sm2fb_HREG = 8, sm3fb_HREG = 9;
const int refMD_HREG = 10, refMM_HREG = 11, refMU_HREG = 12;

ModbusIP mb;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

static const int smDownPin = 25;
static const int smMiddlePin = 33;
static const int smUpPin = 32;
const int potMiddlePin = 36;
const int potDownPin = 34;
const int potUpPin = 39;

int potDownValue = 0, potMiddleValue = 0, potUpValue = 0;

Servo servo_mD;
Servo servo_mM;
Servo servo_mU;

unsigned long previousMillis = 0;
const long interval = 10;
String txt;
int posDegreesSMD = 0, posDegreesSMM = 0, posDegreesSMU = 0;

int pEx, pDx, pEy, pDy, pEz, pDz;
float anDw, xnDw, KnDw, xaDw = 300, paDw = 225, pnDw, rnDw = 10000;
float anMd, xnMd, KnMd, xaMd = 800, paMd = 225, pnMd, rnMd = 10000;
float anUp, xnUp, KnUp, xaUp = 750, paUp = 760, pnUp, rnUp = 10;

float jumpDown, jumpMiddle, jumpUp;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");
  //WiFi.begin("ControlerAuto", "15022489");
  WiFi.begin("Totalplay-ADAA", "ADAA3E6E4kmg2Yy6");
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
  mb.addHreg(x1_HREG, 0);
  mb.addHreg(x2_HREG, 0);
  mb.addHreg(y1_HREG, 0);
  mb.addHreg(y2_HREG, 0);
  mb.addHreg(z1_HREG, 0);
  mb.addHreg(z2_HREG, 0);


  mb.addHreg(sm1fb_HREG, 0);
  mb.addHreg(sm2fb_HREG, 0);
  mb.addHreg(sm3fb_HREG, 0);

  mb.addHreg(refMD_HREG, 0);
  mb.addHreg(refMM_HREG, 80);
  mb.addHreg(refMU_HREG, 0);

  //while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  servo_mD.attach(smDownPin);
  servo_mM.attach(smMiddlePin);
  servo_mU.attach(smUpPin);
  delay(1000);
  Serial.println("Secuencia...");
  //pn = pa;
}

void loop() {

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  printEvent(&orientationData);


  if (Serial.available()) {
    txt = Serial.readStringUntil('\n');
    //Serial.println(txt);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    //posDegrees = txt.toInt();
    posDegreesSMD = mb.Hreg(refMD_HREG);
    posDegreesSMM = mb.Hreg(refMM_HREG);
    posDegreesSMU = mb.Hreg(refMU_HREG);
    servo_mD.write(posDegreesSMD);
    servo_mM.write(posDegreesSMM);
    servo_mU.write(posDegreesSMU);
    //servo_mU.write(posDegreesSMU);
    potDownValue = analogRead(potDownPin);
    potMiddleValue = analogRead(potMiddlePin);
    potUpValue = analogRead(potUpPin);
    Serial.println("-------------------------");
    delay(BNO055_SAMPLERATE_DELAY_MS);

    pEx = int(orientationData.orientation.x);
    pDx = int((orientationData.orientation.x - pEx) * 100);
    pEy = int(orientationData.orientation.y);
    pDy = int((orientationData.orientation.y - pEy) * 100);
    pEz = int(orientationData.orientation.z);
    pDz = int((orientationData.orientation.z - pEz) * 100);


    /*     jumpDown = abs(potDownValue - xaDw);
    if (jumpDown > 10) xaDw = potDownValue;

    jumpMiddle = abs(potMiddleValue - xaMd);
    if (jumpMiddle > 100) xaMd = potMiddleValue; */

/*     jumpUp = abs(potUpValue - xaUp);
    if (jumpUp > 10) xaUp = potUpValue; */

    /*     KnDw = pnDw / (pnDw + rnDw);
    xnDw = xaDw + KnDw * (potDownValue - xaDw);
    pnDw = (1 - KnDw) * pnDw;
    xaDw = xnDw;

    KnMd = pnMd / (pnMd + rnMd);
    xnMd = xaMd + KnMd * (potMiddleValue - xaMd);
    pnMd = (1 - KnMd) * pnMd;
    xaMd = xnMd; */

    KnUp = pnUp / (pnUp + rnUp);
    xnUp = xaUp + KnUp * (potUpValue - xaUp);
    pnUp = (1 - KnUp) * pnUp;
    xaUp = xnUp;


    mb.Hreg(x1_HREG, pEx);
    mb.Hreg(x2_HREG, pDx);
    mb.Hreg(y1_HREG, pEy);
    mb.Hreg(y2_HREG, pDy);
    mb.Hreg(z1_HREG, pEz);
    mb.Hreg(z2_HREG, pDz);
  /*   
     mb.Hreg(sm1fb_HREG, potDownValue);
    mb.Hreg(sm2fb_HREG, potMiddleValue);
    mb.Hreg(sm3fb_HREG, potUpValue); */

     mb.Hreg(sm1fb_HREG, xnDw);
    mb.Hreg(sm2fb_HREG, xnMd);
    mb.Hreg(sm3fb_HREG, xnUp); 


    Serial.print("FbD : ");
    Serial.print(potDownValue);
    //Serial.print(xnDw);
    Serial.print("   FbM : ");
    Serial.print(potMiddleValue);
    //Serial.print(xnMd);
    Serial.print("   FbU : ");
    Serial.println(potUpValue);
    // Serial.print(xnUp);
    mb.task();
    Serial.println("-------------------------");
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
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
