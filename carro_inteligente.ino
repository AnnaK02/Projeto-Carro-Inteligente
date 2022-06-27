// ***************************** BIBLIOTECAS *******************************

#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <stdbool.h>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include "DHT.h"


// ***************************** VARIÁVEIS *******************************

/* VARIÁVEIS E CLASSES BLUETOOTH */

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
const int button = 0;      // button on PIN G0
const int readPin = 4;    // analog pin G4
const int LEDpin = 2;      // LED on pin G2
bool convert = false;
String rxString = "";
std::string rxValue;       // rxValue gathers input data

// UART service UUID data
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)  {
      convert = true;      // flag to invoke convertControlpad routine
      Serial.println(" ");
      Serial.print("Received data: ");
      for (int i = 0; i < rxValue.length(); i++) { 
        Serial.print(rxValue[i]);
        rxString = rxString + rxValue[i]; // build string from received data 
      } 
    } 
  } 
}; 


/* OUTRAS VARIÁVEIS */

/* Pinos para controle dos motores (ponte H) */
int pin1 = 13;
int pin2 = 27;
int pin3 = 26;
int pin4 = 25;

/* Variáveis Start-Stop */
bool startStopFrente = false;
bool startStopTras = false;
bool startStopEsq = false;
bool startStopDir = false;

/* Sensor ultrassônico */
int ECHO = 35;
int TRIGGER = 33;
int tempo = 0;
int distancia = 0;
int lastCommand = -1;

/* Sensor de temperatura/umidade */
int pinDHT11 = 32;
DHT dht11;

int temperature = 0;
int humidity = 0;
int tempInterna = 0;
string aux1 = " ";
string aux2 = " ";
string aux3 = " ";

int rgb_red = 12;
int rgb_blue = 14;


// ***************************** SETUP *******************************

void setup() { 

  /* SETUP PARA CONEXÃO BLUETOOTH */
  
  BLEDevice::init("Wim's ESP32 UART"); // give the BLE device a name
  
  BLEServer *pServer = BLEDevice::createServer(); // create BLE server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY);                    
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  pService->start(); // start the service

  pServer->getAdvertising()->start(); // start advertising
  Serial.println("Waiting a client connection to notify...");
  Serial.println(" ");

  /* SETUP DOS DEMAIS COMPONENTES */

  /* Ponte H */
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  /* Sensor ultrassônico */
  pinMode(ECHO, INPUT);
  pinMode(TRIGGER, OUTPUT); 

  /* Sensor temperatura/umidade */

  dht11.setup(pinDHT11);
  pinMode(rgb_red, OUTPUT);
  pinMode(rgb_blue, OUTPUT);
   
/* Outros */
  Serial.begin(115200);
  
  pinMode(LEDpin, OUTPUT); 
  pinMode(button, INPUT);


  // ****** SETUP COMPLETO ******
  
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

// *************************** MAIN PROGRAM *********************************
void loop() {

  /* FAZENDO CONEXÃO BLUETOOTH */
  DistanceSafety();
  
  if (deviceConnected) {

    txValue = analogRead(readPin);        // pick up value on readPin
    char txString[8];                     // convert the value to a char array
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, decimal_digits, char_buffer   
//    pCharacteristic->setValue(&txValue, 1); // to send the integer value
//    pCharacteristic->setValue("Hello!");    // to send a test message
    pCharacteristic->setValue(txString);  // prepare to send array
    if (digitalRead(button) == LOW) {     // send when button is pressed
      pCharacteristic->notify();          // send the value to the app!
      pCharacteristic->setValue(" ");     // send a space
      pCharacteristic->notify();
      digitalWrite(LEDpin, HIGH);         // switch on the LED
      Serial.println(" ");
      Serial.print("*** Peripheral sends: "); 
      Serial.print(txString);             // report value on serial line
      Serial.println(" ***");
    }
    else digitalWrite(LEDpin, LOW);
    
    if (convert) convertControlpad();
  }

  else{
    //Serial.println("********** CONEXÃO PERDIDA **********");
    analogWrite(pin1, LOW);
    analogWrite(pin2, LOW);
    analogWrite(pin3, LOW);
    analogWrite(pin4, LOW);
  }
  
  Temp_Umid();
  
  delay(50);
}

// ************************* CONVERT CONTROLPAD CODE ************************

/* DETECAR DISTANCIA DE SEGURANÇA */

void DistanceSafety(){
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);

  tempo = pulseIn(ECHO, HIGH);

  distancia = (tempo/2.0)/29.1;

  if(distancia<=25 && (lastCommand==1 || lastCommand==3 || lastCommand==4)){
    Serial.println("Risco de colisao!");
    analogWrite(pin1, LOW);
    analogWrite(pin2, LOW);
    analogWrite(pin3, LOW);
    analogWrite(pin4, LOW);
  }
}


/* DETECÇÃO DE TEMPERATURA E UMIDADE */

void Temp_Umid() {
  // start working...
  Serial.println("=================================");
  Serial.println("Sample DHT11...");

  temperature = dht11.getTemperature();
  humidity = dht11.getHumidity();
  
  Serial.print("Sample OK");

  if(tempInterna==0){
    analogWrite(rgb_red, LOW);
    analogWrite(rgb_blue, LOW);
  }
  else if(tempInterna > 25){
    analogWrite(rgb_red, 200);
    analogWrite(rgb_blue, LOW);
  }
  else if (tempInterna < 22){
    analogWrite(rgb_red, LOW);
    analogWrite(rgb_blue, 200);
  }
  else if(22 <= tempInterna <= 25){
    analogWrite(rgb_red, 50);
    analogWrite(rgb_blue, 150);
  }
  

  /* Enviando informações para o aplicativo */
  aux1 = to_string(temperature);
  aux2 = to_string(tempInterna);
  aux3 = to_string(humidity);

  pCharacteristic->setValue("Ext: ");
  pCharacteristic->notify();
  
  pCharacteristic->setValue(aux1);
  pCharacteristic->notify();
  pCharacteristic->setValue("*C, Int: ");
  pCharacteristic->notify();

  pCharacteristic->setValue(aux2);
  pCharacteristic->notify();
  pCharacteristic->setValue("*C, Umid: ");
  pCharacteristic->notify();

  pCharacteristic->setValue(aux3);
  pCharacteristic->notify();
  pCharacteristic->setValue("%\n");
  pCharacteristic->notify();

  delay(20);
}



/* CONTROLPAD */

void convertControlpad() {
  convert = false;
  Serial.print("      ");

  
  if (rxString == "!B11:"){
    // do action 1
    Serial.println("********** DIMINUIR TEMPERATURA INTERNA");
    tempInterna--;
  }


  else if (rxString == "!B219"){
    // do action 2
    Serial.println("********** AUMENTAR TEMPERATURA INTERNA");
    tempInterna++;
  }

  
  else if (rxString == "!B318"){
    // do action 3
    Serial.println("********** RECALCULAR RELACAO TEMP INTERNA/TEMP EXTERNA");
    tempInterna = temperature;
  }

  

  /* PARA FRENTE */
  else if (rxString == "!B516"){

    lastCommand = 1;
    
    Serial.println("********** Start Action FORWARD");
  
    digitalWrite(LED_BUILTIN, LOW);
    startStopFrente = !startStopFrente;
    startStopTras = false;
    startStopEsq = false;
    startStopDir = false;
  
    if(startStopFrente == true){ // Pressionado novamente
  
      analogWrite(pin1, LOW);
      analogWrite(pin2, 200);
      analogWrite(pin3, LOW);
      analogWrite(pin4, 200);
    }
    else{
      analogWrite(pin2, LOW);
      analogWrite(pin4, LOW);
    } 
  }

  /* PARA TRÁS */
  else if (rxString == "!B615"){

    lastCommand = 2;
    
    Serial.println("********** Start Action BACKWARD");

    startStopFrente = false;
    startStopTras = !startStopTras;
    startStopEsq = false;
    startStopDir = false;

    if(startStopTras == true){ // Pressionado novamente

      analogWrite(pin1, 200);
      analogWrite(pin2, LOW);
      analogWrite(pin3, 200);
      analogWrite(pin4, LOW);
    }
    else{
      analogWrite(pin1, LOW);
      analogWrite(pin3, LOW);
    }
  }

  /* PARA TRÁS ESQUERDA */
  else if (rxString == "!B714"){

    lastCommand = 3;
    
    Serial.println("********** Start Action LEFT FORWARD");

    startStopFrente = false;
    startStopTras = false;
    startStopEsq = !startStopEsq;
    startStopDir = false;

    if(startStopEsq == true){ // Pressionado novamente

      analogWrite(pin1, LOW);
      analogWrite(pin2, 250);
      analogWrite(pin3, LOW);
      analogWrite(pin4, LOW);
    }
    else{
      analogWrite(pin2, LOW);
    }
  }

  /* PARA TRÁS DIREITA */
  else if (rxString == "!B813"){

    lastCommand = 4;
    
    Serial.println("********** Start Action RIGHT FORWARD");

    startStopFrente = false;
    startStopTras = false;
    startStopEsq = false;
    startStopDir = !startStopDir;
    

    if(startStopDir == true){ // Pressionado novamente

      analogWrite(pin1, LOW);
      analogWrite(pin2, LOW);
      analogWrite(pin3, LOW);
      analogWrite(pin4, 250);
    }
    else{
      analogWrite(pin4, LOW);
    }
  }
   
  /*
  else if (rxString == "!B10;") Serial.println("********** Stop Action 1");
  else if (rxString == "!B20:") Serial.println("********** Stop Action 2");
  else if (rxString == "!B309") Serial.println("********** Stop Action 3");
  else if (rxString == "!B408") Serial.println("********** Stop Action 4");
  else if (rxString == "!B507") Serial.println("********** Stop Action UP");
  else if (rxString == "!B606") Serial.println("********** Stop Action DOWN");
  else if (rxString == "!B705") Serial.println("********** Stop Action LEFT");
  else if (rxString == "!B804") Serial.println("********** Stop Action RIGHT");
  */
   
  rxString = "";
}
