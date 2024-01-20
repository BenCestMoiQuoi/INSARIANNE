#include <INSARIANNE.h>

#define ALTITUDE_BASE 85

BMP085 bmp;

void Transfert_Info(){
    bmp.read_sensor();

    Serial.print("Altitude : "); myFile.print("Altitude : ");
    Serial.print(bmp.altitude); myFile.print(bmp.altitude);
    Serial.println(" m,"); myFile.println(" m,");
    Serial.print("Pression : "); myFile.print("Pression : ");
    Serial.print(bmp.pressure); myFile.print(bmp.pressure);
    Serial.println(" Bar,"); myFile.println(" Bar,");
    Serial.print("Pression à la mer : "); myFile.print("Pression à la mer : ");
    Serial.print(bmp.sealevelpressure); myFile.print(bmp.sealevelpressure);
    Serial.println(" Bar,"); myFile.println(" Bar,");
    Serial.print("Température : "); myFile.print("Température : "); 
    Serial.print(bmp.temperature); myFile.print(bmp.temperature);
    Serial.println(" °C."); myFile.println(" °C.");
    Serial.println(); myFile.println();
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    bmp.begin();
    bmp.Set_SLP(ALTITUDE_BASE);
}

void loop() {
    Transfert_Info();
    delay(500);
}