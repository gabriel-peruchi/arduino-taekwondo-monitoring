/* Biblioteca Serial */
#include "SoftwareSerial.h"

/* Comunicação serial Bluetooth RX TX */
SoftwareSerial bluetooth(10, 11);

void setup() {
    /* Inicializa a comuniação Serial */
    Serial.begin(9600);

    Serial.println(F("Digite comandos AT:"));

    /* 
     *  Inicializa a comunicção serial com o Bluetooth 
     *  Obs: Verifique a velocidade já configurada do seu módulo
    */
    bluetooth.begin(57600);
}

void loop() {
    /* Verifica se recebeu um byte do serial do hardware */
    if (Serial.available()) {
        /* Lê e salve o byte */
        char r = Serial.read(); 

        /* Envia o byte para o Bluetooth por serial de software */
        bluetooth.print(r);  

        /* Printa o comando enviado */
        Serial.print(r);
    }
  
    /* Verifica o byte recebido do bluetooth pelo serial do software */
    if (bluetooth.available()) {
        /* Lê e salva o byte */
        char r = bluetooth.read();

        /* Printa o byte */
        Serial.print(r); 
    }
}
