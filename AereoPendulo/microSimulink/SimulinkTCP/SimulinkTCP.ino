#include <WiFi.h>
#include <ESP32Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define M1_PIN 23

//const char *ssid = "Totalplay-ADAA";
//const char *password = "ADAA3E6E4kmg2Yy6";

const char *ssid = "ControlerAuto";
const char *password = "15022489";
byte command;

/* const char *ssid = "CyberSphere2.4";
const char *password = "KD6JVB8kmCRe"; */

WiFiServer server;  // Declara el objeto del servidor
Servo motorCW;
int cteCW, paso, Input, Output, ti, angulo = 35;
unsigned long previousMillis = 0;
const long interval = 10;

int xi, yi, zi, txtln, i;
String xs, ys, zs, txt, txtn;
float x, y, z;

unsigned long StartTime;
unsigned long CurrentTime;
unsigned long ElapsedTime;

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Desactiva la suspensión de wifi en modo STA para mejorar la velocidad de respuesta
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());

  server.begin(22333);  // El servidor comienza a escuchar el puerto número 22333

  /* 
  delay(1500);
  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  motorCW.attach(M1_PIN);

  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.print(" us in this case)");
  Serial.print("\n");
  Serial.println("..wait 2 seconds and press any key.");
  motorCW.writeMicroseconds(MAX_SIGNAL);
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
  Serial.println("The ESC is calibrated");
  Serial.println("----");
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");
  delay(8000);
  paso = 1; */
  //unsigned long StartTime = millis();
}

void loop() {
  
  WiFiClient client = server.available();  // Intenta crear un objeto de cliente
  if (client)                              // Si el cliente actual está disponible
  {
    Serial.println("[Client connected]");
    String readBuff;
    while (client.connected())  // Si el cliente está conectado
    {
      if (client.available())  // Si hay datos legibles
      {
        CurrentTime = millis();
        txt = client.readStringUntil('r');  // Leer un byte
        txtln = txt.length();
        for (int j = 0; j <= txtln; j++) {
          char c = txt[j];
          if (isPrintable(c)) {
            txtn = txtn + c;
          }
        }
        xi = txtn.indexOf('x');
        yi = txtn.indexOf('y');
        zi = txtn.indexOf('z');
        xs = txtn.substring(xi + 1, yi);
        ys = txtn.substring(yi + 1, zi);
        zs = txtn.substring(zi + 1);
        x = xs.toFloat();
        y = ys.toFloat();
        z = zs.toFloat();
        txtn = "";
        /* Input = x;
        if (paso == 0) {
          Output = 0;
        }
        if (paso == 1) {
          Serial.println(ti);
          if (Input < 90) ti++;
          if (Input > 90) {
            
            //xi--;
            paso = 2;
          }
          cteCW = ti + 1000;
          Output = cteCW;
        }
        if (paso == 2) {
          Output = y + cteCW;
        }
        Serial.print("paso: ");
        Serial.println(paso);
        Serial.print("Input: ");
        Serial.println(Input);
        Serial.print("Output: ");
        Serial.println(Output);
        motorCW.writeMicroseconds(Output); */

        //Serial.println(txt);
        //Serial.print("x  ");
        //motorCW.writeMicroseconds(x);
        Serial.print(x);
        Serial.print(',');
        //Serial.print("y  ");
        Serial.print(y);
        Serial.print(',');
        Serial.print(z);
        Serial.print(',');
        Serial.println(ElapsedTime);
        ElapsedTime = 0;
        CurrentTime = millis();
        ElapsedTime = CurrentTime - StartTime;
        
      }
    }
    client.stop();  // Finalizar la conexión actual:
    Serial.println("[Client disconnected]");
  }
  /*   unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


  } */
}

void InitialPos(int th) {
  Serial.print("Ax");
  if (Input < th) ti++;
  if (Input > th) {
    cteCW = ti + 1000;
    //xi--;
    paso = 2;
  }
}