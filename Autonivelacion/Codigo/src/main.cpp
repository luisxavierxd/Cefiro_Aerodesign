#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#include <WiFi.h>

  //Declaraciones para uso de Wifi/Comunicaciones

const char* ssid = "Nombre de red";              //Cambiar al definirlas
const char* password = "Contraseña de red";      //Cambiar al definirlas

  //Declaraciones Giroscopio MPU6050

MPU6050 mpu;

  //Declaraciones Servos

Servo AleronIzquierdo; //Declara el aleron izquierdo como servo

int PinAleronIzquierdo = 0; //Cambiar pines a los que toquen

Servo AleronDerecho; //Declara el aleron derecho como servo

int PinAleronDerecho = 1; //Cambiar pines a los que toquen

  //Valores para autonivelador (PID)

float Kp = 1.0, Ki = 0.0, Kd = 0.0;  //Ajustar para modificar la sensibilidad de la nivelacion
float error, prevError = 0, integral = 0;

  //Servidor WIFI

WiFiServer server(80);

void setup() {
 
   //Encender monitor serial para comunicaciones

  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA = 21, SCL = 22

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
}


void loop() {

    // Leer Giroscopio
  int16_t ax, ay, az, gx, gy, gz; //Declara variables para leer el giroscopio
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Lee aceleraciones y rotaciones hacia todas direcciones (x,y,z)

    // Convertir giroscopio a grados/s

  float AnguloAvion = gx / 131.0;  // ajuste al 131 según sensibilidad del sensor y 
                                // al eje de rotacion segun como se ponga el sensor

   //PID para nivelar

  float setpoint = 0;  // declara valor que queremos conseguir (horizontal)
  error = setpoint - AnguloAvion;
  integral += error * 0.01;
  float derivative = (error - prevError) / 0.01;
  float RESULTADO = Kp*error + Ki*integral + Kd*derivative;
  prevError = error;

    // Ajustar servos para nivelar

  int PosAleronIzq = constrain(90 + RESULTADO, 0, 180);   // Sumamos y restamos por que queremos que giren en contra para
  int PosAleronDer = constrain(90 - RESULTADO, 0, 180);   // girar el avion

  AleronIzquierdo.write(PosAleronIzq);
  AleronDerecho.write(PosAleronDer);

   // Telemetria por WIFI
  WiFiClient client = server.available();
  if (client) {
    String response = "AX:" + String(ax) + " AY:" + String(ay) + " AZ:" + String(az)
                      + " GX:" + String(gx) + " GY:" + String(gy) + " GZ:" + String(gz)
                      + "\n";
    client.println(response);
    client.stop();
  }

  // Imprimimos en serial para probar
  Serial.print("Angulo del avion: "); Serial.print(AnguloAvion);
  Serial.print(" ServoL: "); Serial.print(PosAleronIzq);
  Serial.print(" ServoR: "); Serial.println(PosAleronDer);

  delay(10); // delay necesario para funcionamiento del giroscopio
}
