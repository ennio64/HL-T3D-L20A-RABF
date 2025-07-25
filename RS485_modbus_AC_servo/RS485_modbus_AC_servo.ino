/*
Controllo direzione e velocità AC Servo Motor 80AST-A1C04025 (Pn 1.0 kW - Tn 4.0 Nm - In 4.8 A) con Arduino Nano ATmega168 e RS485 TTL adapter
drive HL-T3D-L20A-RABF
Ennio Sesana 2024
*/

#include <SoftwareSerial.h>

const byte rxPin = 9;
const byte txPin = 10;

SoftwareSerial Serial1(rxPin, txPin);  // Canale seriale per comunicazione RS485

int potPin = A0;
int switchPin = 7;
int potValue = 0;
bool direction = true;        // true per avanti, false per indietro
//bool isServoEnabled = false;  // Variabile per tenere traccia dello stato del servo

// Funzione per calcolare il checksum CRC16-Modbus
uint16_t calculateCRC(byte* buf, int length) {
  uint16_t crc = 0xFFFF;  // Inizializza il CRC correttamente
  for (int pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

// Funzione per inviare una richiesta Modbus con CRC e stampare su seriale
void sendModbusRequestWithCRC(byte* request, int length) {
  // Calcola il checksum
  uint16_t crc = calculateCRC(request, length - 2);
  request[length - 2] = crc & 0xFF;         // LSB del CRC
  request[length - 1] = (crc >> 8) & 0xFF;  // MSB del CRC

  // Invia la richiesta e stampa su seriale
  for (int i = 0; i < length; i++) {
    Serial1.write(request[i]);                 // Invia il byte
    //Serial.print(request[i] < 16 ? "0" : "");  // Aggiunge uno zero iniziale se il byte è inferiore a 16 per la corretta formattazione esadecimale
    //Serial.print(request[i], HEX);             // Stampa il byte in esadecimale
    //Serial.print(" ");                         // Aggiungi uno spazio tra i byte per una migliore leggibilità
  }
  //Serial.println();  // Vai a capo alla fine della stampa
}

void setup() {

  Serial.begin(19200);
  Serial1.begin(19200);

  pinMode(potPin, INPUT);
  pinMode(switchPin, INPUT_PULLUP);

  // Lettura del registro della velocità massima
  readAndShowMaxSpeed();

  Serial.println("Ready to Start");
  delay(5000);

  // Azzeramento del registro della velocità
  byte resetSpeed[8] = { 0x01, 0x06, 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00 };
  sendModbusRequestWithCRC(resetSpeed, sizeof(resetSpeed));

  delay(100);
}

unsigned long previousMillis = 0;
const long interval = 1000;  // Intervallo di aggiornamento in millisecondi

void loop() {
  int potValue = map(analogRead(potPin), 0, 1023, 0, 3000);
  int direction = digitalRead(switchPin);

  if (potValue > 10) {
    /*
    if (!isServoEnabled) {
      byte enableServo[3] = { 0x01, 0x42, 0x55, 0x00, 0x00 };
      sendModbusRequestWithCRC(enableServo, sizeof(enableServo));
      isServoEnabled = true;
    }
    */
    byte jogCommand[5];
    jogCommand[0] = 0x01;
    jogCommand[1] = 0x44;
    jogCommand[3] = 0x00;
    jogCommand[4] = 0x00;

    if (direction == HIGH) {
      jogCommand[2] = 0x01;  // JOG in avanti
    } else {
      jogCommand[2] = 0x02;  // JOG in retromarcia
    }

    sendModbusRequestWithCRC(jogCommand, sizeof(jogCommand));

    delay(10);

    byte setSpeed[8] = { 0x01, 0x06, 0x00, 0x4C, highByte(potValue), lowByte(potValue), 0x00, 0x00 };

    sendModbusRequestWithCRC(setSpeed, sizeof(setSpeed));

  } else {
    byte disableJog[5] = { 0x01, 0x44, 0x00, 0x00, 0x00 };

    sendModbusRequestWithCRC(disableJog, sizeof(disableJog));

    /*
    byte disableServo[3] = { 0x01, 0x42, 0xAA, 0x00, 0x00 };
    sendModbusRequestWithCRC(disableServo, sizeof(disableServo));
    isServoEnabled = false;
    */
  }

  delay(100);  // Aggiorna ogni 100 millisecondi

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Salva il tempo attuale come il momento dell'ultimo aggiornamento
    previousMillis = currentMillis;
    readAndShowSpeed();
  }
}

void readAndShowSpeed() {

  // Tx: 01 04 10 00 00 01 35 0A
  // Comando Modbus per leggere la velocità (indirizzo 0x1000, 1 registro)
  byte readSpeedCommand[8] = { 0x01, 0x04, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00 };  // Indirizzo 0x1000, 1 registro
  sendModbusRequestWithCRC(readSpeedCommand, sizeof(readSpeedCommand));           // Invia il comando Modbus con CRC incluso

  // Attendi la risposta Modbus
  delay(1);  // Adatta il tempo in base alla velocità di risposta del dispositivo

  // Leggi la risposta Modbus
  byte response[7];  // Array per memorizzare la risposta Modbus
  int responseLength = 0;

  while (responseLength < 7) {
    if (Serial1.available() > 0) {
      byte currentByte = Serial1.read();

      // Aggiungi zero davanti se il valore esadecimale è a una cifra
      if (currentByte < 0x10) {
        Serial.print("0");
      }
      Serial.print(currentByte, HEX);
      Serial.print(" ");

      // Memorizza solo i byte che identificano la risposta e la velocità
      if (responseLength < 7) {
        response[responseLength] = currentByte;
        responseLength++;
      }
    }
  }

  // Calcola il CRC della risposta ricevuta
  uint16_t receivedCRC = word(response[5], response[6]);                         // I byte 5 e 6 rappresentano il CRC ricevuto
  uint16_t calculatedCRC = calculateCRC(response, 5);                            // Calcola il CRC sui primi 5 byte della risposta
  uint16_t reversedCalculatedCRC = (calculatedCRC << 8) | (calculatedCRC >> 8);  // Inverti l'ordine dei byte nel CRC calcolato

  /*
  // Stampa i CRC in formato esadecimale (HEX)
  Serial.println("");
  Serial.print("CRC Ricevuto: 0x");
  Serial.println(receivedCRC, HEX);
  Serial.println("");
  Serial.print("CRC Calcolato: 0x");
  Serial.println(reversedCalculatedCRC, HEX);
*/

  // Verifica se il CRC ricevuto corrisponde al CRC calcolato
  if (receivedCRC == reversedCalculatedCRC) {
    Serial.println("CRC corretto: la risposta Modbus è valida.");
    // Visualizza la velocità attuale estratta dagli altri byte della risposta
    int speed = word(response[3], response[4]);  // Calcola la velocità attuale dai byte 3 e 4
    Serial.print("Velocità attuale: ");
    Serial.print(speed);
    Serial.println(" r/min");
  } else {
    Serial.println("Errore CRC: la risposta Modbus potrebbe essere corrotta.");
  }
}

void readAndShowMaxSpeed() {

  //Tx:01 03 00 CF 00 01 B4 35 request max Speed
  // Comando Modbus per leggere la velocità massima (indirizzo 0x00CF, 1 registro)
  byte readMaxSpeed[8] = { 0x01, 0x03, 0x00, 0xCF, 0x00, 0x01, 0x00, 0x00 };
  sendModbusRequestWithCRC(readMaxSpeed, sizeof(readMaxSpeed));  // Invia il comando Modbus con CRC incluso

  // Attendi la risposta Modbus
  delay(1);  // Adatta il tempo in base alla velocità di risposta del dispositivo

  // Leggi la risposta Modbus
  byte response[7];  // Array per memorizzare la risposta Modbus
  int responseLength = 0;

  while (responseLength < 7) {
    if (Serial1.available() > 0) {
      byte currentByte = Serial1.read();

      // Stampa il byte in formato esadecimale (HEX)
      Serial.print(currentByte < 0x10 ? "0" : "");
      Serial.print(currentByte, HEX);
      Serial.print(" ");

      // Memorizza i byte della risposta
      if (responseLength < 7) {
        response[responseLength] = currentByte;
        responseLength++;
      }
    }
  }

  // Calcola il CRC della risposta ricevuta
  uint16_t receivedCRC = word(response[5], response[6]);                         // CRC ricevuto
  uint16_t calculatedCRC = calculateCRC(response, 5);                            // Calcola il CRC sui primi 6 byte della risposta
  uint16_t reversedCalculatedCRC = (calculatedCRC << 8) | (calculatedCRC >> 8);  // Inverti l'ordine dei byte nel CRC calcolato
                                                                                 /*
  // Stampa i CRC in formato esadecimale (HEX)
  Serial.println("");
  Serial.print("CRC Ricevuto: 0x");
  Serial.println(receivedCRC, HEX);
  Serial.println("");
  Serial.print("CRC Calcolato: 0x");
  Serial.println(reversedCalculatedCRC, HEX);
*/
  // Verifica se il CRC ricevuto corrisponde al CRC calcolato
  if (receivedCRC == reversedCalculatedCRC) {
    Serial.println("");
    Serial.println("CRC corretto: la risposta Modbus è valida.");
    // Estrai la massima velocità impostata dal registro
    int maxSpeed = word(response[3], response[4]);  // Velocità massima impostata dai byte 3 e 4
    Serial.print("Massima Velocità Impostata: ");
    Serial.print(maxSpeed);
    Serial.println(" r/min");
  } else {
    Serial.println("");
    Serial.println("Errore CRC: la risposta Modbus potrebbe essere corrotta.");
  }
}
