#include <INSARIANNE.h>

MPU6050 mpu;

void Transfert_Info(){
    mpu.read_sensor();

    Serial.print("Accéleration X : "); myFile.print("Accéleration X : ");
    Serial.print(mpu.accX); myFile.print(mpu.accX);
    Serial.println(" m/s^2,"); myFile.println(" m/s^2,");
    Serial.print("Accéleration Y : "); myFile.print("Accéleration Y : ");
    Serial.print(mpu.accY); myFile.print(mpu.accY);
    Serial.println(" m/s^2,"); myFile.println(" m/s^2,");
    Serial.print("Accéleration Z : "); myFile.print("Accéleration Z : ");
    Serial.print(mpu.accZ); myFile.print(mpu.accZ);
    Serial.println(" m/s^2,"); myFile.println(" m/s^2,");
    Serial.print("Rotation X : "); myFile.print("Rotation X : ");
    Serial.print(mpu.gyroX); myFile.print(mpu.gyroX);
    Serial.println(" rad/s,"); myFile.println(" rad/s,");
    Serial.print("Rotation Y : "); myFile.print("Rotation Y : ");
    Serial.print(mpu.gyroY); myFile.print(mpu.gyroY);
    Serial.println(" rad/s,"); myFile.println(" rad/s,");
    Serial.print("Rotation Z : "); myFile.print("Rotation Z : ");
    Serial.print(mpu.gyroZ); myFile.print(mpu.gyroZ);
    Serial.println(" rad/s,"); myFile.println(" rad/s,");
    Serial.print("Température : "); myFile.print("Température : ");
    Serial.print(mpu.temperature); myFile.print(mpu.temperature);
    Serial.println(" °C."); myFile.println(" °C.");
    Serial.println(); myFile.println();
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    mpu.begin();
}

void loop() {
    Transfert_Info();
    delay(500);
}