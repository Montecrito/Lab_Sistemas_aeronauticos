#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3]; //Angle[0]=Roll, Angle[1]=pitch,Angle[2]=yaw

// Pines de entrada de PWM
const int rollPin = 16;  // GPIO 16 (D0 en el NodeMCU)
const int pitchPin = 5;  // GPIO 5 (D1 en el NodeMCU)
const int yawPin = 4;    // GPIO 4 (D2 en el NodeMCU)

// Variables de los setpoints
volatile float rollSetpoint = 0;
volatile float pitchSetpoint = 0;
volatile float yawSetpoint = 0;

// Variables internas para calcular PWM
volatile unsigned long rollPulseStart = 0, pitchPulseStart = 0, yawPulseStart = 0;

// Constantes PID
float Kp_Roll = 20, Ki_Roll = 0, Kd_Roll = 0.57;
float Kp_Pitch = 7.8, Ki_Pitch = 0, Kd_Pitch = 0.27;
float Kp_Yaw = 7.8, Ki_Yaw = 0, Kd_Yaw = 0.27;

float prevRollError = 0, prevPitchError = 0, prevYawError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;

// Tiempo
unsigned long prevTime = 0;

// Pines de salida para los motores
int motor1Pin = 9;
int motor2Pin = 10;
int motor3Pin = 11;
int motor4Pin = 12;

char status;
//No Library? Click this -> https://github.com/ekstrand/ESP8266wifi

const char *ssid = "banco"; //ESP Access Point
const char *password = "12345678"; //ESP Password
//Please connect to LANZ
//Please take note that some ESP Wifi are hidden
//Please add/connect the ESP Wifi Manually through your phone
//Please make sure your password are 8 or more characters long

WiFiClient client;
IPAddress ip (192, 168, 0, 1);
IPAddress netmask (255,255,255,0);
const int port = 5210;
WiFiServer server(port);

void setup() {
  Wire.begin(4,5);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Calibración y configuración de los ESCs
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

  // Inicialización de los motores
 // calibrateESCs();
  Serial.setTimeout(10);
 // Configurar pines como entrada
  pinMode(rollPin, INPUT);
  pinMode(pitchPin, INPUT);
  pinMode(yawPin, INPUT);

  // Adjuntar interrupciones para detectar cambios
  attachInterrupt(digitalPinToInterrupt(rollPin), handleRollPWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pitchPin), handlePitchPWM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(yawPin), handleYawPWM, CHANGE);

  // Configurar Serial para depuración
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, netmask);
  WiFi.softAP(ssid, password);

  Serial.println("Wifi Details:");
  Serial.println(ssid);
  Serial.println(password);
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  if (!client.connected()) {
    Serial.println("Error! Please make sure to connect to LANZ");
    Serial.println("Please manually add LANZ to your phone");
    Serial.println();
    //Please take note that this message will repeatedly show until
    //you add your device to the app (Wifi Serial Terminal)
    //After the device is connected then this message will stop
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);

    delay(2000);
    client = server.available();
    return;
  }
  if (client.available() >= 0) {
    // Mapear valores PWM a rangos deseados (por ejemplo, -45° a 45°)
  float rollAngle = map(rollSetpoint, 1000, 2000, -45, 45);  // Ejemplo de mapeo
  float pitchAngle = map(pitchSetpoint, 1000, 2000, -45, 45);
  float yawAngle = map(yawSetpoint, 1000, 2000, -180, 180);
  // Leer los ángulos del MPU6050
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

  // Calcular el tiempo desde el último ciclo
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*deltaTime) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*deltaTime) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*deltaTime;
 



  // Control PID para Roll
  float rollError = rollSetpoint - Angle[0];
  rollIntegral += rollError * deltaTime;
  float rollDerivative = (rollError - prevRollError) / deltaTime;
  float rollOutput = Kp_Roll * rollError + Ki_Roll * rollIntegral + Kd_Roll * rollDerivative;
  prevRollError = rollError;

  // Control PID para Pitch
  float pitchError = pitchSetpoint - Angle[1];
  pitchIntegral += pitchError * deltaTime;
  float pitchDerivative = (pitchError - prevPitchError) / deltaTime;
  float pitchOutput = Kp_Pitch * pitchError + Ki_Pitch * pitchIntegral + Kd_Pitch * pitchDerivative;
  prevPitchError = pitchError;

  
  float yawError = yawSetpoint - Angle[2];
  yawIntegral += yawError * deltaTime;
  float yawDerivative = (yawError - prevYawError) / deltaTime;
  float yawOutput = Kp_Yaw * yawError + Ki_Yaw * yawIntegral + Kd_Yaw * yawDerivative;
  prevYawError = yawError;


  int motor1Speed = constrain(1000 + rollOutput + pitchOutput - yawOutput, 1000, 2000);
  int motor2Speed = constrain(1000 - rollOutput + pitchOutput + yawOutput, 1000, 2000);
  int motor3Speed = constrain(1000 - rollOutput - pitchOutput - yawOutput, 1000, 2000);
  int motor4Speed = constrain(1000 + rollOutput - pitchOutput + yawOutput, 1000, 2000);

  // Enviar señales a los motores
  analogWrite(motor1Pin, motor1Speed);
  analogWrite(motor2Pin, motor2Speed);
  analogWrite(motor3Pin, motor3Speed);
  analogWrite(motor4Pin, motor4Speed);

  // Espera un poco para la siguiente lectura
  delay(10);
  }
}
void calibrateESCs() {
  // Calibrar los ESCs con señales de PWM
  analogWrite(motor1Pin, 2000);
  analogWrite(motor2Pin, 2000);
  analogWrite(motor3Pin, 2000);
  analogWrite(motor4Pin, 2000);
  delay(2000);
  analogWrite(motor1Pin, 1000);
  analogWrite(motor2Pin, 1000);
  analogWrite(motor3Pin, 1000);
  analogWrite(motor4Pin, 1000);
  delay(2000);
}
// Función para calcular el ancho de pulso
void IRAM_ATTR handleRollPWM() {
  if (digitalRead(rollPin)) {
    rollPulseStart = micros();  // Inicio del pulso
  } else {
    rollSetpoint = (micros() - rollPulseStart);  // Duración del pulso en microsegundos
  }
}

void IRAM_ATTR handlePitchPWM() {
  if (digitalRead(pitchPin)) {
    pitchPulseStart = micros();  // Inicio del pulso
  } else {
    pitchSetpoint = (micros() - pitchPulseStart);  // Duración del pulso en microsegundos
  }
}

void IRAM_ATTR handleYawPWM() {
  if (digitalRead(yawPin)) {
    yawPulseStart = micros();  // Inicio del pulso
  } else {
    yawSetpoint = (micros() - yawPulseStart);  // Duración del pulso en microsegundos
  }
}
