#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <RF24.h>
#include <Servo.h>

// Define los pines de I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Define los pines del módulo radio nRF24L01
#define CE 9
#define CSN 10

// Define el pin del servo
#define SERVO_PIN 16

// Define el pin del servo
#define ESC_PIN 17

// Crea un objeto MPU6050
MPU6050 mpu;

// Crea un objeto RF24
RF24 radio(CE, CSN);

// Crea un objeto Servo
Servo servo;

// Inicializa el bus I2C
void setup() {
  // Inicializa el bus I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicializa el sensor MPU6050
  mpu.initialize();

  // Verifica que el sensor esté funcionando correctamente
  if (!mpu.testConnection()) {
    while (true) {
      Serial.println("Error al inicializar el sensor MPU6050");
      delay(1000);
    }
  }

  // Inicializa el módulo radio nRF24L01
  radio.begin();

  // Establece la dirección del módulo radio
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.setRetries(15, 15);
  radio.openReadingPipe(0, 0x1234567890);

  // Inicializa el servo
  servo.attach(SERVO_PIN);
}

// Bucle principal
void loop() {
  // Lee las velocidades angulares
  float gyroX, gyroY, gyroZ;
  mpu.getRotationVector(&gyroX, &gyroY, &gyroZ);

  // Lee los ángulos de Euler
  float roll, pitch, yaw;
  mpu.getYawPitchRoll(roll, pitch, yaw);

  // Lee las aceleraciones lineales
  float accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);

  // Calcula las velocidades lineales
  float velX = accelX * (millis() - prevMillis) / 1000.0f;
  float velY = accelY * (millis() - prevMillis) / 1000.0f;
  float velZ = accelZ * (millis() - prevMillis) / 1000.0f;

  // Actualiza el tiempo anterior
  prevMillis = millis();

  // Crea un paquete de datos
  byte datos[12];
  datos[0] = gyroX;
  datos[1] = gyroY;
  datos[2] = gyroZ;
  datos[3] = velX;
  datos[4] = velY;
  datos[5] = velZ;
  datos[6] = roll;
  datos[7] = pitch;
  datos[8] = yaw;

  // Recibe los datos del nRF24L01
  byte datos[2];
  radio.read(datos, 2);

  // Convierte la velocidad radial en señal PWM para el ESC
  float velocidad_radial = datos[0];
  int pwm_esc = map(velocidad_radial, -1, 1, 0, 255);

  // Escribe la señal PWM en el ESC
  analogWrite(ESC_PIN, pwm_esc);

  // Convierte el ángulo en señal para el servo
  float angulo = datos[1];
  int pwm_servo = map(angulo, -1, 1, 0, 180);

  // Escribe la señal PWM en el ESC
  servo.write(pwm_servo);

  // Envía el paquete de datos
  radio.write(datos, 12);
  // Recibe los datos del nRF24L01
  byte datos[2];
  radio.read(datos, 2);

  // Espera 100 milisegundos
  delay(100);
}
