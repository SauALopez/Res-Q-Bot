/*
    ARDUINO BASE FIRMWARE FOR
    RES-Q BOT
    AUTHORS: SAUL, JORGE, PETER, HEBER.

*/
#include <PID_v1.h>
//0#inclue <Servo.h>
#include "constantes.h"


bool G_FLAG = false;
/* PID - OBJECTS FROM PID LIBRARY */
PID RFW_PID(&RPM_RFW, &PWM_RFW, &SetRPM_RFW, kp, ki, kd, DIRECT);
PID LFW_PID(&RPM_LFW, &PWM_LFW, &SetRPM_LFW, 0.95, 0.35, 0.11, DIRECT);
PID RRW_PID(&RPM_RRW, &PWM_RRW, &SetRPM_RRW, kp, ki, kd, DIRECT);
PID LRW_PID(&RPM_LRW, &PWM_LRW, &SetRPM_LRW, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(115200);
  /* CONFIG INPUT PULLUPS FOR INTERRUPTIONS */
  pinMode(ENC_RFW, INPUT_PULLUP);
  pinMode(ENC_LFW, INPUT_PULLUP);
  pinMode(ENC_RRW, INPUT_PULLUP);
  pinMode(ENC_LRW, INPUT_PULLUP);
  /* ATTACHING PINS TO INTERRUPTION SUB ROUTINE*/
  attachInterrupt(digitalPinToInterrupt(ENC_RFW), ISRENC_RFW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LFW), ISRENC_LFW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RRW), ISRENC_RRW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_LRW), ISRENC_LRW, RISING);
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


  // MOVE FORDWARD
  FW_ALL();
  /*CONFIG PID OBJECTS OF THE WHEELS */

  //FREQUENCY (ms) THAT THE PID CALCULATES
  //THE PWM FOR EACH WHEEL (#20MS)
  RFW_PID.SetSampleTime(SAMPLETIME);
  LFW_PID.SetSampleTime(SAMPLETIME);
  RRW_PID.SetSampleTime(SAMPLETIME);
  LRW_PID.SetSampleTime(SAMPLETIME);
  //TURNING ON THE PID'S
  RFW_PID.SetMode(AUTOMATIC);
  LFW_PID.SetMode(AUTOMATIC);
  RRW_PID.SetMode(AUTOMATIC);
  LRW_PID.SetMode(AUTOMATIC);

  /* CONFIG SENSORS PINS */
  pinMode(RF_SENSOR, INPUT_PULLUP);
  pinMode(LF_SENSOR, INPUT_PULLUP);
  pinMode(RR_SENSOR, INPUT_PULLUP);
  pinMode(LR_SENSOR, INPUT_PULLUP);

  //Serial.println('All good, lets go');
  analogWrite(LFW, 98);
  analogWrite(LRW, 64);
  analogWrite(RFW, 64);
  analogWrite(RRW, 64);
}

void loop() {

  //STOP();
  /* POLLING SENSOR PINS */
  if (G_FLAG){
    if (!digitalRead(RR_SENSOR)){
          G_FLAG = false;
          STOP();
          delay(2000);
          RFW_PID.Compute();
          LFW_PID.Compute();
          RRW_PID.Compute();
          LRW_PID.Compute();
      }
  }
  else {
    if (!digitalRead(RF_SENSOR)) {
        Serial.println("Sensor frontal derecho");
        LEFT_TURN();
        if (!digitalRead(LF_SENSOR)) {
          GIRO_RIGHT();
          G_FLAG = true;
        }
    } else if (!digitalRead(LF_SENSOR)) {
        Serial.println("Sensor frontal izquierdo");
        RIGHT_TURN();
        if (!digitalRead(RF_SENSOR)) {
          GIRO_RIGHT();
          G_FLAG = true;
        }
    } else {
        Serial.println("TODO PARA ADELANTE");
        FW_ALL();
      }
  }

  /* CALCULATE PWM WITH PID */
  //FW_ALL();
  RFW_PID.Compute();
  LFW_PID.Compute();
  RRW_PID.Compute();
  LRW_PID.Compute();




  /* CHANGING PWM VALUE OF WHELS */

  analogWrite(LFW, PWM_LFW);
  analogWrite(LRW, PWM_LRW);
  analogWrite(RFW, PWM_RFW);
  analogWrite(RRW, PWM_RRW);

  /*Serial DEBUG*/
  //Serial.println(RPM_LFW); //cambiar variabels
  //Serial.println(RPM_LRW);  No me acuerdo
  //Serial.println(RPM_RFW);  Si esta bueno
  //Serial.println(RPM_RRW);  cacho ajuste
}





/* FUNCTION TO CONTROL THE DIRECTION OF THE WHEELS */
void STOP() {
  SetRPM_RFW = 0;
  SetRPM_RRW = 0;
  SetRPM_LFW = 0;
  SetRPM_LRW = 0;
  //LOW BACKWARD
  digitalWrite(BW_LFW, LOW);
  digitalWrite(BW_LRW, LOW);
  digitalWrite(BW_RFW, LOW);
  digitalWrite(BW_RRW, LOW);
  //LOW FORDWARD
  digitalWrite(FW_LFW, LOW);
  digitalWrite(FW_LRW, LOW);
  digitalWrite(FW_RFW, LOW);
  digitalWrite(FW_RRW, LOW);
}

void FW_ALL() {
  SetRPM_RFW = 20;
  SetRPM_RRW = 20;
  SetRPM_LFW = 20;
  SetRPM_LRW = 20;
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

void BW_ALL() {
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
//FORDWARD TURNS
void LEFT_TURN() {
  //LOW BACKWARD
  digitalWrite(BW_LFW, LOW);
  digitalWrite(BW_LRW, LOW);
  digitalWrite(BW_RFW, LOW);
  digitalWrite(BW_RRW, LOW);
  //LOW FORDWARD LEFT WHEELS
  digitalWrite(FW_LFW, LOW);
  digitalWrite(FW_LRW, LOW);
  //HIGH FORDWARD RIGHT WHEELS
  digitalWrite(FW_RFW, HIGH);
  digitalWrite(FW_RRW, HIGH);
}

void RIGHT_TURN() {
  //LOW BACKWARD
  digitalWrite(BW_LFW, LOW);
  digitalWrite(BW_LRW, LOW);
  digitalWrite(BW_RFW, LOW);
  digitalWrite(BW_RRW, LOW);
  //LOW FORDWARD RIGHT WHEELS
  digitalWrite(FW_RFW, LOW);
  digitalWrite(FW_RRW, LOW);
  //HIGH FORDWARD LEFT WHEELS
  digitalWrite(FW_LFW, HIGH);
  digitalWrite(FW_LRW, HIGH);
}

void GIRO_RIGHT() {
  SetRPM_RFW = 35;
  SetRPM_RRW = 35;
  //LOW BACKWARD
  digitalWrite(BW_LFW, LOW);
  digitalWrite(BW_LRW, LOW);
  digitalWrite(BW_RFW, HIGH);
  digitalWrite(BW_RRW, HIGH);
  //LOW FORDWARD RIGHT WHEELS
  digitalWrite(FW_RFW, LOW);
  digitalWrite(FW_RRW, LOW);
  //HIGH FORDWARD LEFT WHEELS
  SetRPM_LFW = 30;
  SetRPM_LRW = 33;
  digitalWrite(FW_LFW, HIGH);
  digitalWrite(FW_LRW, HIGH);
}

/* INTERRUPT SUB ROUTINES */

void ISRENC_RFW() {     // ISR FOR RIGHT FRONT WHEEL

  long t = micros() - last_RFW;
  RPM_RFW = (60000000) / (t * 370);    //CONVERT 1 REVOLUTION IN RPM
  last_RFW = micros();

}
void ISRENC_LFW() {     // ISR FOR LEFT FRONT WHEEL

  long t = micros() - last_LFW;
  RPM_LFW = (60000000) / (t * 370);
  last_LFW = micros();

}
void ISRENC_RRW() {     // ISR FOR RIGHT REAR WHEEL

  long t = micros() - last_RRW;
  RPM_RRW = (60000000) / (t * 370);
  last_RRW = micros();

  //Serial.print(RPM_RRW);
  //Serial.print(" , ");
  //Serial.println(t);

}
void ISRENC_LRW() {     // ISR FOR LEFT REAR WHEEL

  long t = micros() - last_LRW;
  RPM_LRW = (60000000) / (t * 370);
  last_LRW = micros();


}
