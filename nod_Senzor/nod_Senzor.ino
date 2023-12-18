
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "ArduinoLowPower.h"

#define SN1 A1
#define SN2 A2
#define EC1 A3
#define EC2 A4
#define WM2 A5
#define WM1 A6
#define chipSelect 4
#define comutator 5
#define DS18PIN 6
#define DHTPIN 7
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(DS18PIN);
DallasTemperature sensors(&oneWire);
File myFile;

int counter = 1;
const int nr_Citiri = 10;
const int Rx = 6800;
const float tensiune_Alimentare_MKR = 3.3;
const float tensiune_Alimentare_DECAGON = 2.5;
const float eps = 0.01;

float trimf(float x, float a, float b, float c);
float irigation(float T, float H, float M, float WM, float EC);

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(19200);
  Serial.print("Initializing SD card... ");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  } Serial.println("card initialized.");


  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
  } Serial.println("LoRa Sender");

  LoRa.setTxPower(12);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(500E3);
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------

void loop() {
  
  pinMode(comutator, OUTPUT);
  digitalWrite(comutator, HIGH);

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // SEN0193
  float m_SEN01903_1 = 0;
  float m_SEN01903_2 = 0;

  delay(100);
  for (int i = 0; i < nr_Citiri; i++)
  {
    m_SEN01903_1 += analogRead(SN1);
    m_SEN01903_2 += analogRead(SN2);
  }
  float m_SEN01903 = (m_SEN01903_1 + m_SEN01903_2) / (2 * nr_Citiri);

  if (m_SEN01903 < 280)
    m_SEN01903 = 280 + eps;
  else if (m_SEN01903 > 680)
    m_SEN01903 = 680 - eps;

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // DS18B20
  sensors.begin();
  sensors.requestTemperatures();
  float t_DS18B20 = sensors.getTempCByIndex(0);
  digitalWrite(comutator, LOW);

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // DHT22
  dht.begin();
  delay(1000);
  float h_DHT22 = dht.readHumidity();
  float t_DHT22 = dht.readTemperature();

  if (t_DHT22 < 10)
    t_DHT22 = 10 + eps;
  else if (t_DHT22 > 40)
    t_DHT22 = 40 - eps;

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // WATERMARK 200SS
  float WM1_Direct_Read = 0;
  float WM1_Invers_Read = 0;
  float WM2_Direct_Read = 0;
  float WM2_Invers_Read = 0;

  pinMode(0, OUTPUT); digitalWrite(0, LOW);
  pinMode(1, OUTPUT); digitalWrite(1, LOW);
  pinMode(2, OUTPUT); digitalWrite(2, LOW);
  pinMode(3, OUTPUT); digitalWrite(3, LOW);

  for (int i = 0; i < nr_Citiri; i++)
  {

    // Citire polaritate directa
    digitalWrite(2, HIGH);
    digitalWrite(0, HIGH);
    delay(0.1);
    WM1_Direct_Read += analogRead(WM1);
    WM2_Direct_Read += analogRead(WM2);
    digitalWrite(2, LOW);
    digitalWrite(0, LOW);

    // Asteapta 100 ms
    delay(100);

    // Citire polaritate inversa
    digitalWrite(3, HIGH);
    digitalWrite(1, HIGH);
    delay(0.1);
    WM1_Invers_Read += analogRead(WM1);
    WM2_Invers_Read += analogRead(WM2);
    digitalWrite(3, LOW);
    digitalWrite(1, LOW);
  }

  // Conversia tensiunii in gama de valori 0-3.3V
  float WM1_Direct_Voltage = ((WM1_Direct_Read / 1024) * tensiune_Alimentare_MKR) / nr_Citiri;
  float WM2_Direct_Voltage = ((WM2_Direct_Read / 1024) * tensiune_Alimentare_MKR) / nr_Citiri;
  float WM1_Invers_Voltage = ((WM1_Invers_Read / 1024) * tensiune_Alimentare_MKR) / nr_Citiri;
  float WM2_Invers_Voltage = ((WM2_Invers_Read / 1024) * tensiune_Alimentare_MKR) / nr_Citiri;

  // Divizor de tensiune
  float WM1_Direct_Resistance = (Rx * (tensiune_Alimentare_MKR - WM1_Direct_Voltage) / WM1_Direct_Voltage);
  float WM2_Direct_Resistance = (Rx * (tensiune_Alimentare_MKR - WM2_Direct_Voltage) / WM2_Direct_Voltage);
  float WM1_Invers_Resistance = (Rx * WM1_Invers_Voltage) / (tensiune_Alimentare_MKR - WM1_Invers_Voltage);
  float WM2_Invers_Resistance = (Rx * WM2_Invers_Voltage) / (tensiune_Alimentare_MKR - WM2_Invers_Voltage);

  // Valoarea rezistentei
  float WM1_Resistance = (WM1_Direct_Resistance + WM1_Invers_Resistance) / 2;
  float WM2_Resistance = (WM2_Direct_Resistance + WM2_Invers_Resistance) / 2;
  float WM_Resistance = (WM1_Resistance + WM2_Resistance) / 2;

  // Conversia rezistentei in CentiBari
  float case_1 = -2.246 - 5.239 * (WM_Resistance / 1000.00) * (1 + 0.018 * (t_DS18B20 - 24.00)) - 0.06756 * (WM_Resistance / 1000.00) * (WM_Resistance / 1000.00) * ((1.00 + 0.018 * (t_DS18B20 - 24.00)) * (1.00 + 0.018 * (t_DS18B20 - 24.00)));
  float case_2 = (-3.213 * (WM_Resistance / 1000.00) - 4.093) / (1 - 0.009733 * (WM_Resistance / 1000.00) - 0.01205 * (t_DS18B20));
  float case_3 = ((WM_Resistance / 1000.00) * 23.156 - 12.736) * (1.00 + 0.018 * (t_DS18B20 - 24.00));

  float WM_CB = 0;
  if (WM_Resistance > 8000.00)
    WM_CB = abs(case_1);
  else if (WM_Resistance > 1000.00)
    WM_CB = abs(case_2);
  else if (WM_Resistance > 550.00)
    WM_CB = abs(case_3);
  else
    WM_CB = 0 + eps;

  if (WM_CB > 200)
    WM_CB = 200 - eps;

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // DECAGON EC-5
  float EC1_ADC = 0;
  float EC2_ADC = 0;

  for (int i = 0; i < nr_Citiri; i++)
  {
    digitalWrite(comutator, HIGH);
    delay(10);
    EC1_ADC += analogRead(EC1);
    EC2_ADC += analogRead(EC2);
    digitalWrite(comutator, LOW);
    delay(90);
  }

  float EC1_mV = ((EC1_ADC / 1024) * tensiune_Alimentare_DECAGON * 1000) / nr_Citiri;
  float EC2_mV = ((EC2_ADC / 1024) * tensiune_Alimentare_DECAGON * 1000) / nr_Citiri;
  float EC1_VWC = ((2.11 / 1000) * EC1_mV) - 0.675;
  float EC2_VWC = ((2.11 / 1000) * EC2_mV) - 0.675;
  float EC_VWC = ((EC1_VWC + EC2_VWC) * 100) / 2;

  if (EC_VWC < 0)
    EC_VWC = 0 + eps;
  else if (EC_VWC > 60)
    EC_VWC = 60 - eps;

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Valori Default pentru senzori
  if (isnan(t_DHT22))
    t_DHT22 = 25;
  if (isnan(h_DHT22))
    h_DHT22 = 50;
  if (isnan(m_SEN01903))
    m_SEN01903 = 480;
  if (isnan(WM_CB))
    WM_CB = 100;
  if (isnan(EC_VWC))
    EC_VWC = 30;
  delay (100);

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Afisare pentru comparatia valorilor cu cele obtinute in Matlab
  Serial.println();
  Serial.print("Citire nr: ");
  Serial.println(counter);
  Serial.print("DS18B20: ");
  Serial.print("\tTemperatura sol: ");
  Serial.print(t_DS18B20);
  Serial.println("°C ");
  Serial.print("DHT22: ");
  Serial.print("\t\tTemperatura aer: ");
  Serial.print(t_DHT22);
  Serial.println("°C");
  Serial.print("\t\tUmiditate aer: ");
  Serial.print(h_DHT22);
  Serial.println("% ");
  Serial.print("SEN0193: ");
  Serial.print("\tUmiditate sol senzor 1: ");
  Serial.println(m_SEN01903_1);
  Serial.print("\t\tUmiditate sol senzor 2: ");
  Serial.println(m_SEN01903_2);
  Serial.print("\t\tUmiditate sol: ");
  Serial.println(m_SEN01903);
  Serial.print("WM 200SS: ");
  Serial.print("\tWM1(Resistance)= ");
  Serial.println(WM1_Resistance);
  Serial.print("\t\tWM2(Resistance)= ");
  Serial.println(WM2_Resistance);
  Serial.print("\t\tWM(Resistance)= ");
  Serial.println(WM_Resistance);
  Serial.print("\t\tWM(CB)= ");
  Serial.println(WM_CB);
  Serial.print("Decagon 5-EC:");
  Serial.print("\tVWC1: ");
  Serial.println(EC1_VWC);
  Serial.print("\t\tVWC2: ");
  Serial.println(EC2_VWC);
  Serial.print("\t\tVWC: ");
  Serial.println(EC_VWC);
  Serial.println();
  Serial.print(EC_VWC);
  Serial.print(" ");
  Serial.print(WM_CB);
  Serial.print(" ");
  Serial.print(m_SEN01903);
  Serial.print(" ");
  Serial.print(h_DHT22);
  Serial.print(" ");
  Serial.println(t_DHT22);

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Calcularea nivelului necesar de irigare
  float irig = irigation(t_DHT22, h_DHT22, m_SEN01903, WM_CB, EC_VWC);
  Serial.print("Nivel irigare: ");
  Serial.print(irig * 100);
  Serial.println(" %");
  Serial.println();

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Transmitere nivelului necesar de irigare prin LoRa
  LoRa.beginPacket();
  LoRa.println(irig, 4);
  LoRa.endPacket();

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Scrierea pe cardul SD
  myFile = SD.open("HAIDEE.txt", FILE_WRITE);
  if (myFile) {

    // Setup
    myFile.println();
    myFile.print("Citire nr: ");
    myFile.println(counter);

    // DS18B20
    myFile.print("DS18B20: ");
    myFile.print("\tTemperatura sol: ");
    myFile.print(t_DS18B20);
    myFile.println("°C ");

    // DHT22
    myFile.print("DHT22: ");
    myFile.print("\t\tTemperatura aer: ");
    myFile.print(t_DHT22);
    myFile.println("°C");
    myFile.print("\t\tUmiditate aer: ");
    myFile.print(h_DHT22);
    myFile.println("% ");

    // SEN0193
    myFile.print("SEN0193: ");
    myFile.print("\tUmiditate sol senzor 1: ");
    myFile.println(m_SEN01903_1);
    myFile.print("\t\tUmiditate sol senzor 2: ");
    myFile.println(m_SEN01903_2);
    myFile.print("\t\tUmiditate sol: ");
    myFile.println(m_SEN01903);

    // WATERMARK 200SS
    myFile.print("WM 200SS: ");
    myFile.print("\tWM1(Resistance)= ");
    myFile.println(WM1_Resistance);
    myFile.print("\t\tWM2(Resistance)= ");
    myFile.println(WM2_Resistance);
    myFile.print("\t\tWM(Resistance)= ");
    myFile.println(WM_Resistance);
    myFile.print("\t\tWM(CB)= ");
    myFile.println(WM_CB);

    // DECAGON EC-5
    myFile.print("Decagon 5-EC: ");
    myFile.println(EC_VWC);

    // Teardown
    myFile.println();
    myFile.close();
  }

  else {
    Serial.println("Nu s-a putut deschide fisierul .txt");
  }

  // -------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // Pregatire pentru urmatorul loop
  counter++;
  delay(30000);
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// Functie de apartenenta triunghiulara
// x = variabila de temperatura, umiditate, etc, masurata de senzori
// a = punctul de inceput al triungiului
// b = varful triunghiului
// c = punctul de final al triungiului
float trimf(float x, float a, float b, float c) {
  float f;
  float eps = 0.0001;
  if (x <= a)
    f = 0;
  else if ((a <= x) && (x <= b))
    f = (x - a) / (b - a);
  else if ((b <= x) && (x <= c))
    f = (c - x) / (c - b);
  else if (c <= x)
    f = 0;
  return f;
}

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// Functia de prezicere a nivelului necesar de irigatii
float irigation(float T, float H, float M, float WM, float EC) {

  // Grade de necesitate a irigării
  // Funcțiile de apartenență de ieșire de tip singleton din sistem
  float c1 = 0;
  float c2 = 0.5;
  float c3 = 1;

  // Reguli Fuzzy
  // Model Takagi-Sugeno
  float w1   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w2   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w3   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w4   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w5   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w6   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w7   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w8   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w9   = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w10  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w11  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w12  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w13  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w14  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w15  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w16  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w17  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w18  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w19  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w20  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w21  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w22  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w23  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w24  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w25  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w26  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w27  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w28  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w29  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w30  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w31  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w32  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w33  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w34  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w35  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w36  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w37  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w38  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w39  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w40  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w41  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w42  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w43  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w44  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w45  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w46  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w47  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w48  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w49  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w50  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w51  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w52  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w53  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w54  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w55  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w56  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w57  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w58  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w59  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w60  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w61  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w62  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w63  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w64  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w65  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w66  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w67  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w68  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w69  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w70  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w71  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w72  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w73  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w74  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w75  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w76  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w77  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w78  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w79  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w80  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w81  = min (trimf(EC, 0, 0, 30),     min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w82  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w83  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w84  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w85  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w86  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w87  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w88  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w89  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w90  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w91  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w92  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w93  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w94  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w95  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w96  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w97  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w98  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w99  = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w100 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w101 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w102 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w103 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w104 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w105 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w106 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w107 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w108 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w109 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w110 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w111 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w112 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w113 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w114 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w115 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w116 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w117 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w118 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w119 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w120 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w121 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w122 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w123 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w124 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w125 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w126 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w127 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w128 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w129 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w130 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w131 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w132 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w133 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w134 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w135 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w136 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w137 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w138 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w139 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w140 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w141 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w142 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w143 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w144 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w145 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w146 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w147 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w148 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w149 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w150 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w151 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w152 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w153 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w154 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w155 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w156 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w157 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w158 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w159 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w160 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w161 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w162 = min (trimf(EC, 0, 30, 60),    min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w163 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w164 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w165 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w166 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w167 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w168 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w169 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w170 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w171 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w172 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w173 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w174 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w175 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w176 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w177 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w178 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w179 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w180 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w181 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w182 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w183 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w184 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w185 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w186 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w187 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w188 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w189 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 0, 100),       min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w190 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w191 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w192 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w193 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w194 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w195 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w196 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w197 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w198 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w199 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w200 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w201 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w202 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w203 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w204 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w205 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w206 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w207 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w208 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w209 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w210 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w211 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w212 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w213 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w214 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w215 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w216 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 0, 100, 200),     min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w217 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w218 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w219 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w220 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w221 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w222 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w223 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w224 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w225 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 280, 480),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w226 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w227 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w228 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w229 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w230 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w231 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w232 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w233 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w234 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 280, 480, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));
  float w235 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 10, 25)))));
  float w236 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 10, 25, 40)))));
  float w237 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 0, 50),       trimf(T, 25, 40, 40)))));
  float w238 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 10, 25)))));
  float w239 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 10, 25, 40)))));
  float w240 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 0, 50, 100),     trimf(T, 25, 40, 40)))));
  float w241 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 10, 25)))));
  float w242 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 10, 25, 40)))));
  float w243 = min (trimf(EC, 30, 60, 60),   min (trimf(WM, 100, 200, 200),   min (trimf(M, 480, 680, 680),   min (trimf(H, 50, 100, 100),   trimf(T, 25, 40, 40)))));

  // Media ponderata
  float c1_Rules = w1 + w4 + w6 + w7 + w8 + w9 + w13 + w16 + w18 + w25 + w31 + w34 + w36 + w43 + w61 + w82 + w84 + w85 + w86 + w87 + w88 + w89 + w90 + w91 + w94 + w96 + w97 +
                   w98 + w99 + w103 +  w106 + w108 + w109 + w112 + w114 + w115 + w116 + w117 + w121 + w124 + w126 + w133 + w139 + w142 + w144 + w151 + w163 + w164 + w165 + w166 +
                   w167 + w168 + w169 + w170 + w171 + w172 + w174 + w175 + w176 + w177 + w178 + w179 + w180 + w181 + w184 + w186 + w187 + w188 + w189 + w190 + w192 + w193 + w194 +
                   w195 + w196 + w197 + w198 + w199 + w202 + w204 + w205 + w206 + w207 + w211 + w214 + w216 + w217 + w220 + w222 + w223 + w224 + w225 + w229 + w232 + w234 + w241;
  float c2_Rules = w2 + w3 + w5 + w10 + w12 + w14 + w15 + w17 + w19 + w22 + w24 + w26 + w27 + w28 + w30 + w32 + w33 + w35 + w37 + w40 + w42 + w44 + w45 + w49 + w52 + w54 + w55 +
                   w58 + w60 + w62 + w63 + w67 + w70 + w72 + w79 + w83 + w92 + w93 + w95 + w100 + w102 + w104 + w105 + w107 + w110 + w111 + w113 + w118 + w120 + w122 + w123 + w125 +
                   w127 + w130 + w132 + w134 + w135 + w136 + w138 + w140 + w141 + w143 + w145 + w148 + w150 + w152 + w153 + w157 + w160 + w162 + w173 + w182 + w183 + w185 + w191 +
                   w200 + w201 + w203 + w208 + w210 + w212 + w213 + w215 + w218 + w219 + w221 + w226 + w228 + w230 + w231 + w233 + w235 + w238 + w240 + w242 + w243;
  float c3_Rules = w11 + w20 + w21 + w23 + w29 + w38 + w39 + w41 + w46 + w47 + w48 + w50 + w51 + w53 + w56 + w57 + w59 + w64 + w65 + w66 + w68 + w69 + w71 + w73 + w74 + w75 + w76 +
                   w77 + w78 + w80 + w81 + w101 + w119 + w128 + w129 + w131 + w137 + w146 + w147 + w149 + w154 + w155 + w156 + w158 + w159 + w161 + w209 + w227 + w236 + w237 + w239;

  // Defuzzyficare
  float z = (c1_Rules * c1 + c2_Rules * c2 + c3_Rules * c3) / (c1_Rules + c2_Rules + c3_Rules);
  return z;

}
