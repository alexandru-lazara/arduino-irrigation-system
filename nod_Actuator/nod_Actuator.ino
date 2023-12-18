
#include <SPI.h>
#include <LoRa.h>

#define FS300A 6

float factor_Calibrare = 5.5;
double cantitate_Apa;
double cantitate_Apa_mL;
double cantitate_Apa_total;

volatile int frecventa_Puls;
const int RELAY_PIN = 1;

void setup() {
  Serial.begin(19200);

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTxPower(12);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(500E3);

  attachInterrupt(FS300A, Flow, RISING);
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {

  frecventa_Puls = 0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {

      String numString = LoRa.readString();
      float dVal = numString.toFloat();
      int interval = (int)(dVal * 30000);

      if (dVal >= 0.55) {
        Serial.print("Timpul cat electrovalva e pornită = ");
        Serial.print(interval);
        Serial.println(" ms");

        interrupts();
        digitalWrite(RELAY_PIN, HIGH);
        delay(interval);
        digitalWrite(RELAY_PIN, LOW);
        noInterrupts();

        cantitate_Apa = frecventa_Puls / factor_Calibrare;
        cantitate_Apa_mL = (cantitate_Apa / 60) * 1000;
        cantitate_Apa_total += cantitate_Apa_mL;

        Serial.print("Cantitate de apa folosită la irigare: \t");
        Serial.print(cantitate_Apa_mL);
        Serial.println(" mL");
        Serial.print("Cantitate de apa folosita in total: \t");
        Serial.print(cantitate_Apa_total);
        Serial.println(" mL");
        Serial.println();
      }

      else if (dVal < 0.55) {
        Serial.println("Irigarea nu este necesară. Se așteaptă o nouă decizie de irigare.");
      }
      
    }
  }
}

void Flow()
{
  frecventa_Puls++;
}
