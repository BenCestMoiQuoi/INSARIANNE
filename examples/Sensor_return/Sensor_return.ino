#include <Sensors_INSARIANNE.h>

BMP085 bmp;
MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if(!bmp.begin() || !mpu.begin()) {
    Serial.println("Initialisation impossible !");
    while(1); 
  }
  Serial.println("Initialisation terminée !");
}

void loop() {
  bmp.read_sensor();
  mpu.read_sensor();

  Serial.print("Altitude : "); Serial.print(bmp.altitude); Serial.println(" m,");
  Serial.print("Pression : "); Serial.print(bmp.pressure); Serial.println(" Bar,");
  Serial.print("Pression à la mer : "); Serial.print(bmp.sealevelpressure); Serial.println(" Bar,");
  Serial.print("Température : "); Serial.print(bmp.temperature); Serial.println(" °C.");
  Serial.println();

  Serial.print("Accéleration X : "); Serial.print(mpu.accX); Serial.println(" m/s^2,");
  Serial.print("Accéleration Y : "); Serial.print(mpu.accY); Serial.println(" m/s^2,");
  Serial.print("Accéleration Z : "); Serial.print(mpu.accZ); Serial.println(" m/s^2,");
  Serial.print("Rotation X : "); Serial.print(mpu.gyroX); Serial.println(" rad/s,");
  Serial.print("Rotation Y : "); Serial.print(mpu.gyroY); Serial.println(" rad/s,");
  Serial.print("Rotation Z : "); Serial.print(mpu.gyroZ); Serial.println(" rad/s,");
  Serial.print("Température : "); Serial.print(mpu.temperature); Serial.println(" °C.");
  Serial.println();

  delay(1000);
}
