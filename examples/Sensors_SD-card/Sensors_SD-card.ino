#include <INSARIANNE.h>
#include <SPI.h>
#include <SD.h>

# define _BV(n) (1<<n)

BMP085 bmp;
MPU6050 mpu;

File myFile;
# define SD_pin 5
# define NAME_FILE "text.txt"

unsigned long timer_ms;
unsigned long timer_info;
unsigned long count_ms;
unsigned long count_s;

void Init_Timer(){
  /*
  Initialisation du Timer, 
  le timmer est fait pour une intervalle de 1 seconde,
  une interruption pour l'update se fait toutes les 1ms.

  Pour comprendre, faut se référer à la 
    Datasheet ATMEGA48A
  l'init du timer se fait par les registres de ce micro-processeur
  (Celui de l'Arduino Nano) Clk = 16 MHz
  
  Nous allons utilisez le Timmer 0 pour nous faire notre horloge.
  Le Timmer 1 est utilisé pour les deux servo en pin PWM 9 et 10 par la librairie Servo.
  */
  cli();//stop interrupts

  //set timer0 interrupt at 1kHz
  TCCR0A = 0 | _BV(WGM01); // turn on CTC mode
  TCCR0B = 0 | _BV(CS01) | _BV(CS00); //Set the prescale 1/64 clock
  TIMSK0 |= _BV(OCIE0A); //Set the interrupt request
  // set compare match register for 1khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  

  sei(); //Enable interrupt
  // Init variable timer
  timer_ms = 0;
  timer_info = 0;
  count_ms = 0;
  count_s = 0;
}

bool Init_Sensor() {
  Wire.begin();
  Serial.begin(9600);

  if(!bmp.begin() || !mpu.begin()) {
    Serial.println("Initialisation capteurs impossible !");
    return false; 
  }

  if(!SD.begin(SD_pin)){
    Serial.println("Initialisation SD impossible !");
    return false;
  }
  myFile = SD.open(NAME_FILE, FILE_WRITE);
  
  
  return true;
}

void Transfert_Info(unsigned long nb_ms){
  /*
  Fonction qui envoie l'ensemble des données des capteurs sur la Carte SD
  Permet également de gérer l'horloge personnalisé (secondes:milliseconde).

  L'écriture sur la carte se fait toutes les nb_ms

  Input : 
    unsigned long nb_ms -> Nombre de millis pour l'écriture
  */
  if (count_ms >= 1000){
    count_ms-=1000;
    count_s++;
  }

  if (timer_info >= nb_ms){
    timer_info = 0;
    
    bmp.read_sensor();
    mpu.read_sensor();

    myFile.print(count_s);
    myFile.print(":");
    myFile.println(count_ms);

    myFile.print("Altitude : "); myFile.print(bmp.altitude); myFile.println(" m,");
    myFile.print("Pression : "); myFile.print(bmp.pressure); myFile.println(" Bar,");
    myFile.print("Pression à la mer : "); myFile.print(bmp.sealevelpressure); myFile.println(" Bar,");
    myFile.print("Température : "); myFile.print(bmp.temperature); myFile.println(" °C.");
    myFile.println();

    myFile.print("Accéleration X : "); myFile.print(mpu.accX); myFile.println(" m/s^2,");
    myFile.print("Accéleration Y : "); myFile.print(mpu.accY); myFile.println(" m/s^2,");
    myFile.print("Accéleration Z : "); myFile.print(mpu.accZ); myFile.println(" m/s^2,");
    myFile.print("Rotation X : "); myFile.print(mpu.gyroX); myFile.println(" rad/s,");
    myFile.print("Rotation Y : "); myFile.print(mpu.gyroY); myFile.println(" rad/s,");
    myFile.print("Rotation Z : "); myFile.print(mpu.gyroZ); myFile.println(" rad/s,");
    myFile.print("Température : "); myFile.print(mpu.temperature); myFile.println(" °C.");
    myFile.println();
  }
}


void setup() {
  Init_Timer();
  if (!Init_Sensor()) while(1);
  
  Serial.println("Initialisation terminée !");
  myFile.println("Initialisation terminée !");
  count_ms = 0;
  count_s = 0;
}

void loop() {
    Transfert_Info(1000);
}

ISR(TIMER0_COMPA_vect){    //This  is the interrupt request
  timer_ms++;
  count_ms++;
  timer_info++;
}

