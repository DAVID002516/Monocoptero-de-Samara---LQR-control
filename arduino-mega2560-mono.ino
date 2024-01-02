#include <RF24.h>

// Define los pines de I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Define los pines del módulo radio nRF24L01
#define CE 9
#define CSN 10


// Crea un objeto RF24
RF24 radio(CE, CSN);

  // Inicializa el módulo radio nRF24L01
  radio.begin();

  // Establece la dirección del módulo radio
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.setRetries(15, 15);
  radio.openReadingPipe(0, 0x1234567890);
}

// Bucle principal
void loop() {
  // Recibe los datos del nRF24L01
  byte datos[6];
  radio.read(datos, 6);

  // Almacena los datos recibidos en variables flotantes
  float gyroX = datos[0];
  float gyroY = datos[1];
  float gyroZ = datos[2];
  float accelX = datos[3];
  float accelY = datos[4];
  float accelZ = datos[5];

  // Imprime los datos recibidos en el monitor serie
  Serial.print("Gyro X: ");
  Serial.print(gyroX);
  Serial.print(", Gyro Y: ");
  Serial.print(gyroY);
  Serial.print(", Gyro Z: ");
  Serial.println(gyroZ);

  Serial.print("Accel X: ");
  Serial.print(accelX);
  Serial.print(", Accel Y: ");
  Serial.print(accelY);
  Serial.print(", Accel Z: ");
  Serial.println(accelZ);

  // Recibe los datos del serial
  float velocidad_radial;
  float angulo;
  while (Serial.available() < 2) {
    delay(10);
  }
  velocidad_radial = Serial.parseFloat();
  angulo = Serial.parseFloat();
  // Crea un paquete de datos
  byte datos[2];
  datos[0] = velocidad_radial;
  datos[1] = angulo;

  // Envia el paquete de datos
  radio.write(datos, 2);

  // Espera 100 milisegundos
  delay(100);

}
