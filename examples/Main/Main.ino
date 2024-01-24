/*
Programme Arduino
Nom : Main.ino
Auteur : Roche Corentin
Version : 4
Date : 20/01/2023

Programme principale du project
INSARIANNE 1

*/

#include <Servo.h>
#include <INSARIANNE.h>
#include <SD.h>

#define _BV(n) (1<<n)

#define ALTITUDE_BASE 85


/*  Define pins  */

#define Jack_pin 4

#define Led_pin_sol_1 6
#define Led_pin_sol_2 7
#define Switch_pin_sol_1 2
#define Switch_pin_sol_2 3

#define Val_Timer 3000 // 3s ici

Servo servo_haut;
Servo servo_bas;
#define Pin_Servo_Bas 9
#define Pin_Servo_Haut 10

File myFile;
#define SD_pin 5

MPU6050 mpu;
BMP085 bmp;

/*  Declare the variables  */

int Etat_sol;
bool Etat_vol;

unsigned long timer_ms;
unsigned long timer_info;
unsigned long count_ms;
unsigned long count_s;

/*  Initialisation  */

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

void Init_Sol(){
  /* 
  Initialisation de la phase au sol
  
  Avec 2 LEDs, 2 Switchs et la prise Jack
  Configure les LEDs en état "avec initialisation" 
  (Etat_sol = 0)
  */
  pinMode(Led_pin_sol_1, OUTPUT);
  pinMode(Led_pin_sol_2, OUTPUT);
  pinMode(Switch_pin_sol_1, INPUT_PULLUP);
  pinMode(Switch_pin_sol_2, INPUT_PULLUP);
  
  pinMode(Jack_pin, INPUT_PULLUP);

  Etat_sol = 0;
  write_LED_Sol();
}

void Init_Vol(){
  /* 
  Initialisation de la phase de vol
  
  Avec 2 Servomoteur
  (Etat_vol = 0)
  */
  servo_bas.attach(Pin_Servo_Bas);
  servo_bas.write(0);

  servo_haut.attach(Pin_Servo_Haut);
  servo_haut.write(180);

  Etat_vol = false;
}

void Init_Sensor(){
  /* 
  Initialisation de la partie capteur
  
  Avec 1 carte SD
       1 accelerometre 3 axes
       1 gyroscope 3 axes
       1 capteur de température
  */
  SD.begin(SD_pin);
  myFile = SD.open("test.txt", FILE_WRITE);

  mpu.begin();
  bmp.begin();
  bmp.Set_SLP(ALTITUDE_BASE);
}


void Verif_Sol(){
  /*
  Fonction qui permet de gérer entièrement l'état d'avancement de la fusée
  lorsquel est au sol (Etat_sol de 1 à 3).
  
  Etat_sol = 1 
      LED 1 --> Alumé
      LED 2 --> Eteinte
      Servo_haut --> 0
  Etat_sol = 2 (si prise Jack branché)
      LED 1 --> Eteinte
      LED 2 --> Alumé
      Servo_haut --> 180
  Etat_sol = 3 (dernière validation)
      LED 1 --> Alumé
      LED 2 --> AlumémyFil
      Servo_haut --> 180
  Lorsque le Jack et débranché, Etat_vol = True
  */
  if (!digitalRead(Switch_pin_sol_1) && (Etat_sol == 2 || Etat_sol == 3)){
    Etat_sol = 1;
    write_LED_Sol();
    servo_haut.write(0);
  }
  if (digitalRead(Switch_pin_sol_1) && Etat_sol == 1) servo_haut.write(180);
  if (Etat_sol == 1 && digitalRead(Switch_pin_sol_1) && !digitalRead(Switch_pin_sol_2) 
        && !digitalRead(Jack_pin) && servo_haut.read()==180){
    Etat_sol = 2;
    write_LED_Sol();
  }
  if (Etat_sol == 2 && digitalRead(Switch_pin_sol_2)){
    Etat_sol = 3;
    write_LED_Sol();
  }
  if (Etat_sol == 3 && !digitalRead(Switch_pin_sol_2)){
    Etat_sol = 2;
    write_LED_Sol();
  }
  if (Etat_sol == 3 && digitalRead(Jack_pin)) Etat_vol = true;
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

    Serial.print(count_s); myFile.print(count_s);
    Serial.print(":"); myFile.print(":");
    Serial.println(count_ms); myFile.println(count_ms);

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
}

void Ouverture_porte(){
  /*
  Fonction qui permet d'ouvrir la trappe tout en transmettant 
  les données des capteurs

  Ouverture du servo haut (locket)
  Puis 
  */
  timer_ms = 0;
  while (timer_ms < 500) {
    Transfert_Info(200);
    servo_haut.write(0);
  }
  timer_ms = 0;
  while (timer_ms < 500) {
    Transfert_Info(200);
    servo_haut.write(0);
    servo_bas.write(180);
  }
}

void Sol(){
  /*
  Fonction de la fusée lorsqu'elle est au sol.
  Et lorsque que l'initialisation est faite.
  S'arrete lorsque la prise jack (ici le 3e switch) est arraché
    Etat_vol devient True

    Plusieurs phase sont considéré : 
      - Phase 1 : 
        LED 1 -> Allumée
        En attante de pose de la porte
        Une fois la porte monté, appuyer sur le switch 1
      Entre P1 et P2, Servo_Haut se ferme
      - Phase 2 :
        LED 2 -> Allumée
        En attente d'une dernière validation (porte bien fermé)
        Appuyer sur le switch 2
      - Phase 3 :
        LED 1 et LED 2 -> Allumée
        En attante de lancement (arrachement de la prise jack)
  */
  Etat_vol = false;
  timer_ms = 0;
  while (!Etat_vol){
    Verif_Sol();
    Transfert_Info(1000); // Toutes les 1s (1000 ms)
  }
  Serial.println("Décolage");
}

void Vol(){
  /*
  Fonction de la fusée lorsqu'elle est en vol.
  

  Toutes les 200ms on envoi des infos des capteurs
  */
  timer_ms = 0;
  while (timer_ms < Val_Timer) Transfert_Info(200);
  Ouverture_porte();
  Serial.println("Ouverture porte");
// Fin du vol
  while(1) Transfert_Info(200);
}

void write_LED_Sol(){
  /*
  Fonction qui gère l'état des LED en fonction de la variable Etat_sol
  */
  switch (Etat_sol) {
  case 1:
    digitalWrite(Led_pin_sol_1, HIGH);
    digitalWrite(Led_pin_sol_2, LOW);
    break;
  case 2:
    digitalWrite(Led_pin_sol_1, LOW);
    digitalWrite(Led_pin_sol_2, HIGH);
    break;
  case 3:
    digitalWrite(Led_pin_sol_1, HIGH);
    digitalWrite(Led_pin_sol_2, HIGH);
    break;
  default:
    digitalWrite(Led_pin_sol_1, LOW);
    digitalWrite(Led_pin_sol_2, LOW);
    break;
  }
}

void setup() {
  Serial.begin(9600);

  Init_Sol();
  Init_Timer();
  Init_Vol();
  Init_Sensor();

  while(digitalRead(Switch_pin_sol_1) || digitalRead(Switch_pin_sol_2));
  
  Serial.println("Initialisation Finis");
  Etat_sol = 1;
  count_ms = 0;
  count_s = 0;
  write_LED_Sol();
  
  /*  Main program  */
  Sol();
  Vol();
}
void loop() {}

ISR(TIMER0_COMPA_vect){    //This  is the interrupt request
  timer_ms++;
  count_ms++;
  timer_info++;
}
