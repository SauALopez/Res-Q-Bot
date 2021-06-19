/*
 *  ARDUINO BASE FIRMWARE FOR 
 *  RES-Q BOT
 *  AUTHORS: SAUL, JORGE, PETER, HEBER.
 * 
*/
#include "constantes.h"

void setup(){
    Serial.begin(9600);
    /* CONFIG INPUT PULLUPS FOR INTERRUPTIONS */
    pinMode(ENC_RFW, INPUT_PULLUP);
    pinMode(ENC_LFW, INPUT_PULLUP);
    pinMode(ENC_RRW, INPUT_PULLUP);
    pinMode(ENC_LRW, INPUT_PULLUP);
    /* ATTACHING PINS TO INTERRUPTION SUB ROUTINE*/
    attachInterrupt(digitalPinToInterrupt(ENC_RFW),ISRENC_RFW, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_LFW),ISRENC_LFW, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_RRW),ISRENC_RRW, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC_LRW),ISRENC_LRW, FALLING);
}

void loop(){
    Serial.println("Hola Mundo con constantes");
    delay(10000);   
}
void ISRENC_RFW(){
    
    long t = millis()- last_RFW;
    RPM_RFW = 60000/t;
    last_RFW = millis();

}
void ISRENC_LFW(){

    long t = millis()- last_LFW;
    RPM_LFW = 60000/t;
    last_LFW = millis();

}
void ISRENC_RRW(){

    long t = millis()- last_RRW;
    RPM_RRW = 60000/t;
    last_RRW = millis();

}
void ISRENC_LRW(){

    long t = millis()- last_LRW;
    RPM_LRW = 60000/t;
    last_LRW = millis();

}