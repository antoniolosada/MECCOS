#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2023-12-02 10:11:27

#include "Arduino.h"
#include "Arduino.h"
#include <MeccaBrain.h>
#include <Wire.h>
#include <Servo.h>

void setJoint(byte jointName, byte pos) ;
int getJoint(byte jointName) ;
void setEyesColor(byte red, byte green, byte blue, byte fadetime) ;
void setJointColor(byte jointName, byte color) ;
void setup() ;
void loop() ;
void receiveEvent(int howMany) ;
void EjecutarMovimientoProgramado() ;
void EjecutarComando(int comando, int disp, int pos_color) ;
void ControlMovimientoTemporizado() ;

#include "meccos.ino"


#endif
