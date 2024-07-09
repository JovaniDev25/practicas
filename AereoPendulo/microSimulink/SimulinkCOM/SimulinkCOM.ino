#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

float outSin, inRad;

unsigned long previousMillis = 0;
const long interval = 10;

// Create a union to easily convert float to byte
typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t myIMUx;
FLOATUNION_t myIMUy;
FLOATUNION_t myIMUz;
FLOATUNION_t mySignal;

void setup() {
  // initialize serial, use the same boudrate in the Simulink Config block
  Serial.begin(115200);
  //Serial.println("Hola Mundo!!!");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
}
void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
   if (inRad > 6.2831) inRad = 0;
    else inRad = inRad + 0.01;
    outSin = sin(inRad*5);
    mySignal.number = outSin;
    //Serial.println(outSin);
    Serial.write('z');
    for (int i = 0; i < 4; i++) {
      Serial.write(mySignal.bytes[i]);
    }
    Serial.print('\n');
  } 


  //Serial.println("Abc");

   /* sensors_event_t orientationData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    printEvent(&orientationData);
    printEvent(&linearAccelData);
    //Serial.println("AQC");
    //Serial.write('z');
    myIMUx.number = orientationData.orientation.x;
    myIMUy.number = orientationData.orientation.y;
    myIMUz.number = orientationData.orientation.z;
    Serial.write('z');
    for (int i = 0; i < 4; i++) {
      Serial.write(myIMUx.bytes[i]);
    }
    //Serial.print('\n');
    for (int i = 0; i < 4; i++) {
      Serial.write(myIMUy.bytes[i]);
    }
    for (int i = 0; i < 4; i++) {
      Serial.write(myIMUz.bytes[i]);
    }
    Serial.print('\n');
  }  */
}

void printEvent(sensors_event_t* event) {
  float x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
  float xa = -1000000, ya = -1000000, za = -1000000;

  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    xa = event->acceleration.x;
    ya = event->acceleration.y;
    za = event->acceleration.z;
  } else {
    Serial.print("Unk:");
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