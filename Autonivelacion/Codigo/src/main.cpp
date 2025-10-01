#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <ArduinoJson.h>  // Para JSON

  //Declaraciones para uso de Wifi/Comunicaciones

const char* ssid = "Nombre de red";              //Cambiar al definirlas
const char* password = "Contraseña de red";      //Cambiar al definirlas
WiFiServer server(80);

  //Declaraciones Giroscopio MPU6050

MPU6050 mpu;

  //Declaraciones Servos

Servo AleronIzquierdo; //Declara el aleron izquierdo como servo
const int PinAleronIzquierdo = 18; //Cambiar pines a los que toquen

Servo AleronDerecho; //Declara9 el aleron derecho como servo
const int PinAleronDerecho = 1; //Cambiar pines a los que toquen

  //Valores para autonivelador (PID)

float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float setpoint = 0.0;
float error, prevError = 0, integral = 0;
unsigned long lastTime;

  //Variables para filtro complementario

float anguloGiro = 0.0;
const float alpha = 0.98; // ponderación giroscopio

  //Funcion PID

float computePID(float input) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;  // segundos
    lastTime = now;

    //Formula general para PID

    error = setpoint - input;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
    return output;
}


void setup() {
 
    //Encender monitor serial para comunicaciones

  Serial.begin(115200);

    //Comunicacion I2C para el giroscopio

  Wire.begin(21, 22);   // SDA = 21, SCL = 22 (Pines Giroscopio)

    //Conectar servos a sus pines

  AleronIzquierdo.attach(PinAleronIzquierdo); 

  AleronDerecho.attach(PinAleronDerecho); 

    // Inicializar Giroscopio

  mpu.initialize();
  if (mpu.testConnection()) Serial.println("Giroscopio conectado"); //Probar conexion

   // Conectar WiFi

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");

  while (WiFi.status() != WL_CONNECTED) {     // No inicia el programa si 
    delay(500);                               // no se conecta a wifi
    Serial.print(".");                        // remover para pruebas sin conexion
   
  }

  Serial.println("\nWiFi conectado");
  server.begin();

    //Empieza timer para el PID

  lastTime = millis();

}


void loop() {

    // Leer Giroscopio
  int16_t ax, ay, az, gx, gy, gz; //Declara variables para leer el giroscopio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Lee aceleraciones y rotaciones hacia todas direcciones (x,y,z)

    // Convertir giroscopio a grados/s

  float anguloAvion = gx / 131.0;  // ajuste al 131 según sensibilidad del sensor y 
                                // al eje de rotacion segun como se ponga el sensor

  // Ángulo estimado usando filtro complementario para mejorar estabilidad

    float rollAccel = atan2((float)ay, (float)az) * 180.0 / PI;
    anguloGiro = alpha * (anguloGiro + anguloAvion * ((millis() - lastTime)/1000.0)) + (1.0 - alpha) * rollAccel;

   //PID para nivelar

  float pidOutput = computePID(anguloGiro);

    // Ajustar servos para nivelar

  int PosAleronIzq = constrain(90 + pidOutput, 0, 180);   // Sumamos y restamos por que queremos que giren en contra para
  int PosAleronDer = constrain(90 - pidOutput, 0, 180);   // girar el avion

  AleronIzquierdo.write(PosAleronIzq);
  AleronDerecho.write(PosAleronDer);

      // Telemetría WiFi en JSON (multi-cliente no bloqueante)

  WiFiClient client = server.available();
    while(client) {
        StaticJsonDocument<256> doc;
        doc["AX"] = ax;
        doc["AY"] = ay;
        doc["AZ"] = az;
        doc["GX"] = gx;
        doc["GY"] = gy;
        doc["GZ"] = gz;
        doc["AnguloGiro"] = anguloGiro;
        doc["ServoLeft"] = PosAleronIzq;
        doc["ServoRight"] = PosAleronDer;

        String output;
        serializeJson(doc, output);
        client.println(output);

        client = server.available(); // permitir siguiente cliente
    }


    // Imprimimos en serial para realizar pruebas
  Serial.print("Angulo del avion: "); Serial.print(anguloGiro);
  Serial.print(" ServoL: "); Serial.print(PosAleronIzq);
  Serial.print(" ServoR: "); Serial.println(PosAleronDer);

}
