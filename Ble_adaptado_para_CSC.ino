
/*
 * Piece of code are based on:
 * This sketch emulates parts of a Polar H7 Heart Rate Sensor. 
 * It exposes the Heart rate and the Sensor position characteristics
 
   Copyright <2017> <Andreas Spiess>

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
   
   Pieces of code are Based on Neil Kolban's example file: https://github.com/nkolban/ESP32_BLE_Arduino
    and
   Multi BLE Sensor - Richard Hedderly 2019

   Pieces of code are Based on example file of Animal Land - Youtuber-2024

Log_ de acciones
   ESTA VERSION ES LA MAS CERCANA PARA CONTINUAR EL DESAROOLO - ADAPTADA POR Hernando Bolaños _ Marzo_2024 - Se usan partes de codigo de cuatro autores-
   Issues:
   Mejoras AGREGAR EL TEMA DE BPM - los senosres para arduino son un poco inestables e inexactos segun comparaciones con bandas de frecuecnia cardiaca deportivas
   RESOLVER EL TEMA DEL TIME CRANK- Status; Resuelto con mapping adecuado de cada byte en programa de Richard
   
EL PROGRAMA CORRE SIN PROBLEMAS EN ARDUINO 1.8.13 Y USANDO LA TARJETA DE DESARROLLO ESP32 -WROOM-32
   
 */
 
/*DECLARACION DE LIBRERIAS */
 
#include <BLE2902.h>//Esto debe ir en la  primera linea o sino no compila elprograma-un Bug de la libreria en c++
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

/* DECLARACION DE VARABLES*/

volatile unsigned long previousMillis = 0;//concept: volatile is a keyword known as a variable qualifier, it is usually used before the datatype of a variable, to modify the way in which the compiler and subsequent program treats the variable.
volatile unsigned long lastMillis = 0;
volatile unsigned long currentMillis = 0;

volatile unsigned long time_prev_wheel = 0, time_now_wheel;
volatile unsigned long time_prev_crank = 0, time_now_crank;
volatile unsigned long time_chat = 100;

//volatile unsigned int wheelRev = 0;
volatile unsigned int oldWheelRev = 0;
volatile unsigned long oldWheelMillis = 0;
//volatile unsigned long lastWheeltime = 0;
//volatile unsigned int crankRev = 0;
volatile unsigned int oldCrankRev = 0;
//volatile unsigned long lastCranktime = 0;
volatile unsigned long oldCrankMillis = 0;

uint16_t crankRev;  // Cadence RPM_ concept:  uint16_t is a datatype that's unsigned and is 16 bits wide. So, the maximum value is 2^16, or 65535.Each uint16_t is two bytes
uint16_t lastCranktime; // Last crank time

uint32_t wheelRev;      // Wheel revolutions
uint16_t lastWheeltime;     // Last crank time

uint16_t cadence;

byte speedkph;    // Speed in KPH
byte speedmph;    // Speed in MPH


byte flags = 0b00001110;
byte bpm;
byte crank[11] = { 0b00000011,10, 0, 0, 0 , 0, 0, 0,0,0,0};//la bandera indica presencia de crank y wheel
byte cscfeature[1] = { 0b0000000000000010 };
byte crankmPos[1] = {2}; //revisar

bool _BLEClientConnected = false;


/*DECLARACION DE SERVICIO Y CARACTERISTICAS BLE SEGUN DOUCMENTOS DE GATT DE LA ORGANIZACION BLUETOOTH*/

#define CSCService BLEUUID((uint16_t)0x1816) //Se define servicio para cadencia y velocidad segun GATT
BLECharacteristic CSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);//CSC Measurement 0x2A5B
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5C), BLECharacteristic::PROPERTY_READ);//CSC Feature 0x2A5C
BLECharacteristic CSCMeasurementCharacteristics2(BLEUUID((uint16_t)0x2A5C), BLECharacteristic::PROPERTY_NOTIFY);//
BLECharacteristic sensorPositionCharacteristic2(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);//0x2A5D Sensor Location
BLECharacteristic cscFeatureCharacteristics(     BLEUUID ((uint16_t) 0x2A5C), BLECharacteristic::PROPERTY_READ);    //CSC Feature Characteristic


BLEDescriptor CSCDescriptor(BLEUUID((uint16_t)0x2901));//Characteristic User Description
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));//Characteristic User Description
BLEDescriptor cscFeatureDescriptor(BLEUUID ((uint16_t) 0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};


/*RUTINA PARA ARRANCAR EL SERVIDOR BLE*/

void InitBLE() {
  BLEDevice::init("HBP");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pCSC = pServer->createService(CSCService);

  pCSC->addCharacteristic(&CSCMeasurementCharacteristics);
  CSCDescriptor.setValue("Rate from 0 to 200");
  CSCMeasurementCharacteristics.addDescriptor(&CSCDescriptor);
  CSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pCSC->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Biela pierna Izquierda");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);


  cscFeatureDescriptor.setValue ("Exercise Bike CSC Feature");
  cscFeatureCharacteristics.addDescriptor (&cscFeatureDescriptor);

  pServer->getAdvertising()->addServiceUUID(CSCService);

  pCSC->start();
  // Start advertising
  pServer->getAdvertising()->start();
}


void setup() {

  lastWheeltime = millis();//concept: Returns the number of milliseconds since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.
  lastCranktime = millis();

 
/*DECLARACION DE ENTRADAS DE LO SESNORES COMO INTERRUPCIONES*/


attachInterrupt(digitalPinToInterrupt(32), wheelAdd, FALLING); // RISING FALLING CHANGE LOW /concept:Interrupts are useful for making things happen automatically in microcontroller programs, and can help solve timing problems. Good tasks for using an interrupt may include reading a rotary encoder, or monitoring user input.
attachInterrupt(digitalPinToInterrupt(33), crankAdd, FALLING);//
  
  Serial.begin(115200);
  Serial.println("Start");
  InitBLE();
  crankRev = 1;

  lastCranktime = 0;
  wheelRev = 0;
  lastWheeltime = 0;
}

void loop() {
  // put your main code here, to run repeatedly:


/* DECLARACION DE CADA BYTE PARA ENVIAR INFORMACION DE LOS SENSORES DE CADENCIA Y VELOCIDAD Y DEL  TIMER DE CADA EVENTO O EL ESTAMPE DE TIEMPO*/
 
  crank[1] = wheelRev & 0xFF;
  crank[2] = (wheelRev >> 8) & 0xFF; 

  crank[3] = (wheelRev >> 16) & 0xFF; 
  crank[4] = (wheelRev >> 24) & 0xFF; 
  
  crank[5] = lastWheeltime& 0xFF;
  crank[6] = (lastWheeltime>> 8) & 0xFF; 

  crank[7] = crankRev & 0xFF;
  crank[8] = (crankRev >> 8) & 0xFF; 

  crank[9] = lastCranktime & 0xFF;
  crank[10] = (lastCranktime >> 8) & 0xFF; 
  

  
  Serial.println(crankRev);

  /*instrucciones para enviar la información de sensores y de estampe de tiempo a los clientes en esta caso la aplicación del telefono celular*/

  CSCMeasurementCharacteristics.setValue(crank, 11);
  CSCMeasurementCharacteristics.notify();



  sensorPositionCharacteristic.setValue(crankmPos, 1);

  cscFeatureCharacteristics.setValue(cscfeature, 1);
  


 currentMillis = millis();
      // if 200ms have passed, check the  mesurement:
      if (oldWheelRev < wheelRev && currentMillis - oldWheelMillis >= 200) {
        updateCSC("wheel");
      }
      else if (oldCrankRev < crankRev && currentMillis - oldCrankMillis >= 200) {
        updateCSC("crank");
      }
      else if (currentMillis - previousMillis >= 1000) {
        updateCSC("timer");
      }

Serial.print(crank[0]);  //Should be 2 to reflect binary - Yes
  Serial.print(" ");
  Serial.print(crank[1]);  // Should increment by 2 - Yes
  Serial.print(" ");
  Serial.print(crank[2]);
  Serial.print(" ");
  Serial.print(crank[3]);
  Serial.print(" ");
  Serial.print(crank[4]);
  Serial.print(" ");
  Serial.print(crank[5]);
  Serial.print(" ");
  Serial.print(crank[6]);
  Serial.print(" ");
  Serial.print(crank[7]);
  Serial.print(" ");
  Serial.print(crank[8]);
  Serial.print(" ");
  Serial.print(crank[9]);
  Serial.print(" ");
  Serial.print(crank[10]);
  Serial.print(" ");
  Serial.print(crank[11]);
  Serial.println(" ");

  delay(2000);
}


// SUBRUTINA PARA ADICIONAR EVENTO Y ESTAMPE DE TIEMPODEL SENSOR DE VELOCIDAD

void wheelAdd() {
  time_now_wheel = millis();
  if( time_now_wheel > time_prev_wheel + time_chat){
    wheelRev = wheelRev + 1;
    time_prev_wheel = time_now_wheel;
    lastWheeltime = millis();
  }
}

// SUBRUTINA PARA ADICIONAR EVENTO Y ESTAMPE DE TIEMPO DEL SENSOR DE CADENCIA

void crankAdd() {
  time_now_crank = millis();
  if( time_now_crank > time_prev_crank + time_chat){
    crankRev = crankRev + 1;
    time_prev_crank = time_now_crank;
    lastCranktime = millis();
  }
}


// SUBRUTINA PARA ACTUALIZAR INFORMACION DE REFERENCIA E IMPRIMIR POR PUERTO SERIAL

void updateCSC(String sType) {
  oldWheelRev = wheelRev;
  oldCrankRev = crankRev;
  oldWheelMillis = currentMillis;
  previousMillis = currentMillis;
  
  Serial.print("Wheel Rev.: ");
  Serial.print(wheelRev);
  Serial.print(" WheelTime : ");
  Serial.print(lastWheeltime);
  Serial.print(" Crank Rev.: ");
  Serial.print(crankRev);
  Serial.print(" CrankTime : ");
  Serial.print(lastCranktime);
  Serial.print("  ");
  Serial.println(sType);
}
