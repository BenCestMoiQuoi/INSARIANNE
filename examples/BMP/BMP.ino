#include <INSARIANNE_BMP085.h>


#define ALTITUDE_BASE 85

BMP085 bmp;

void Transfert_Info(){
    bmp.read_sensor();

    Serial.print("Altitude : ");
    Serial.print(bmp.altitude);
    Serial.println(" m,");
    Serial.print("Pression : ");
    Serial.print(bmp.pressure);
    Serial.println(" Bar,");
    Serial.print("Pression à la mer : ");
    Serial.print(bmp.sealevelpressure);
    Serial.println(" Bar,");
    Serial.print("Température : ");
    Serial.print(bmp.temperature);
    Serial.println(" °C.");
    Serial.println();
}

void setup() {
    Serial.begin(9600);
    
    Serial.print("Version Librairie : "); Serial.println(VERSION_LIB);
    Wire.begin();

    bmp.begin();
    bmp.Set_SLP(ALTITUDE_BASE);
}

void loop() {
    Transfert_Info();
    delay(500);
}