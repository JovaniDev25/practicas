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