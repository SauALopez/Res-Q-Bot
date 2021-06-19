

/*
 *  ARDUINO BASE FIRMWARE FOR 
 *  RES-Q BOT
 *  AUTHORS: SAUL, JORGE, PETER, HEBER.
 * 
*/
#include <PID_v1.h>
#include "constantes.h"
void setup(){
    Serial.begin(115200);
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
    /* CONFIG OUTPUT PWM PINS */
    pinMode(RFW, OUTPUT);
    pinMode(LFW, OUTPUT);
    pinMode(RRW, OUTPUT);
    pinMode(LRW, OUTPUT);
    /* CONFIG OUTPUT DIRECTION CONTROL PINS */
    pinMode(FW_RFW, OUTPUT);
    pinMode(BW_RFW, OUTPUT);

    pinMode(FW_LFW, OUTPUT);
    pinMode(BW_LFW, OUTPUT);

    pinMode(FW_RRW, OUTPUT);
    pinMode(BW_RRW, OUTPUT);

    pinMode(FW_LRW, OUTPUT);
    pinMode(BW_LRW, OUTPUT);
}

void loop(){
    Serial.println("Hola Mundo, DEBUG");
    delay(10000);   
}




/* FUNCTION TO CONTROL THE DIRECTION OF THE WHEELS */

void FW_ALL(){
    //LOW BACKWARD
    digitalWrite(BW_LFW, LOW);
    digitalWrite(BW_LRW, LOW);
    digitalWrite(BW_RFW, LOW);
    digitalWrite(BW_RRW, LOW);
    //HIGH FORDWARD
    digitalWrite(FW_LFW, HIGH);
    digitalWrite(FW_LRW, HIGH);
    digitalWrite(FW_RFW, HIGH);
    digitalWrite(FW_RRW, HIGH);
}

void BW_ALL(){
    //LOW FORDWARD
    digitalWrite(FW_LFW, LOW);
    digitalWrite(FW_LRW, LOW);
    digitalWrite(FW_RFW, LOW);
    digitalWrite(FW_RRW, LOW);
    //HIGH BACKWARD
    digitalWrite(BW_LFW, HIGH);
    digitalWrite(BW_LRW, HIGH);
    digitalWrite(BW_RFW, HIGH);
    digitalWrite(BW_RRW, HIGH);
}


/* INTERRUPT SUB ROUTINES */

void ISRENC_RFW(){      // ISR FOR RIGHT FRONT WHEEL

    long t = millis()- last_RFW;
    RPM_RFW = 60000/t;      //CONVERT 1 REVOLUTION IN RPM
    last_RFW = millis();

}
void ISRENC_LFW(){      // ISR FOR LEFT FRONT WHEEL

    long t = millis()- last_LFW;
    RPM_LFW = 60000/t;
    last_LFW = millis();

}
void ISRENC_RRW(){      // ISR FOR RIGHT REAR WHEEL

    long t = millis()- last_RRW;
    RPM_RRW = 60000/t;
    last_RRW = millis();

}
void ISRENC_LRW(){      // ISR FOR LEFT REAR WHEEL

    long t = millis()- last_LRW;
    RPM_LRW = 60000/t;
    last_LRW = millis();

}