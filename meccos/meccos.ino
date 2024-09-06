// MECCOS: MECano COntrol System

#include "Arduino.h"
#include <MeccaBrain.h>
#include <Wire.h>
#include <Servo.h>

const byte MAX_SERVOS_POS = 12;
const byte MAX_SERVOS_MECANOID = 8;
const byte GRADOS_SEG = 120;

Servo aServo[4]; //Manos con servos estandar

struct sPosServos
{
  int iValor;
  int iValorInicial;
  int iValorFinal;
  byte iValoIniTemporizado;
  byte iValorFinTemporizado;
  long ms_tiempo_mov = 0;
  long ms_inicial = 0;
  long ms_final = 0;
  bool MovTemporizadoActivo = false;
} aPosServos[MAX_SERVOS_POS];

//Pins to connect Meccanoids' servos, where chain 1 is left arm, chain 2 is head and chain 3 is right arm
//pins can be any digital pins, not necessary PWM
const int chainPin1 = 3; //Left
const int chainPin2 = 4; //Head
const int chainPin3 = 2; //Right

const int PinLeftWirst = 	6;
const int PinLeftHand = 	5;
const int PinRightWirst = 	8;
const int PinRightHand = 	7;

MeccaBrain chain1(chainPin1); //Left, each chain allows to plug up to 4 smart modules
MeccaBrain chain2(chainPin2); //Head, each chain allows to plug up to 4 smart modules
MeccaBrain chain3(chainPin3); //Right, each chain allows to plug up to 4 smart modules

const byte COMMAND_POS  	= 80;
const byte COMMAND_COLOR  	= 81;
const byte COMMAND_HEAD_LED	= 82;
const byte EXEC_PROG_MOV	= 83;

//Joints mapping:
//Chain 1 - Left Arm. 1.0 is Arm Pitch, 1.1 is Arm Roll, 1.2 is Elbow
//Chain 2 - Head. 2.0 is Head Yaw, 2.1 is Head Roll, 2.2 is LEDs
//Chain 3 - Right Arm. 3.0 is Arm Pitch, 3.1 is Arm Roll 3.2 is Elbow
const byte LEFT_ARM_PITCH 	= 0; //hombro
const byte LEFT_ARM_ROLL 	= 1; //brazo
const byte LEFT_ARM_ELBOW 	= 2;

const byte HEAD_YAW 		= 3; //Cabeza lateral
const byte HEAD_ROLL 		= 4; //Cabeza arriba-abajo

const byte RIGHT_ARM_PITCH 	= 5; //hombro
const byte RIGHT_ARM_ROLL 	= 6; //brazo
const byte RIGHT_ARM_ELBOW 	= 7;

const byte RIGHT_WRIST 		= 8; //Muñeca
const byte RIGHT_HAND 		= 9; //Mano

const byte LEFT_WRIST 		= 10;//Muñeca
const byte LEFT_HAND 		= 11;//MAno

//	Pos Servos				     {  0,   1, 2,  3,  4,  5,  6,  7,  8,  9, 10, 11};
byte valor_ini[MAX_SERVOS_POS] = {250,100,128,120,130, 30,170,128,  8, 80,175,120};
byte valor_min[MAX_SERVOS_POS] = { 50, 40,  0, 25, 85, 30,100,000,  5, 80,  5, 60};
byte valor_max[MAX_SERVOS_POS] = {220,140,255,220,180,200,200,255,180,120,180,120};
byte valor_dir[MAX_SERVOS_POS] = {  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1};

//jointName is LEFT_ARM_ROLL etc
//pos is 0...255
void setJoint(byte jointName, byte pos)
{

  switch (jointName)
  {
    case LEFT_ARM_PITCH:
      chain1.setServoPosition(0, pos);
      chain1.communicate();
      break;
    case LEFT_ARM_ROLL:
      chain1.setServoPosition(1, pos);
      chain1.communicate();
      break;
//    case LEFT_ARM_ELBOW:
//      chain1.setServoPosition(2, pos);
//      chain1.communicate();
//      break;
    case HEAD_YAW:
      chain2.setServoPosition(0, pos);
      chain2.communicate();
      break;
    case HEAD_ROLL:
      chain2.setServoPosition(1, pos);
      chain2.communicate();
      break;
    case RIGHT_ARM_PITCH:
      chain3.setServoPosition(0, pos);
      chain3.communicate();
      break;
    case RIGHT_ARM_ROLL:
      chain3.setServoPosition(1, pos);
      chain3.communicate();
      break;
//    case RIGHT_ARM_ELBOW:
//      chain3.setServoPosition(2, pos);
//      chain3.communicate();
//      break;

    case RIGHT_WRIST:
    	aServo[0].write(pos);
	  break;
    case RIGHT_HAND:
    	aServo[1].write(pos);
    	break;
    case LEFT_WRIST:
    	aServo[2].write(pos);
    	break;
    case LEFT_HAND:
    	aServo[3].write(pos);
    	break;
  }
}

int getJoint(byte jointName)
{

  switch (jointName)
  {
    case LEFT_ARM_PITCH:
		chain1.setServotoLIM(0);
		chain1.communicate();
		chain1.getServoPosition(0);
		break;
    case LEFT_ARM_ROLL:
        chain1.setServotoLIM(1);
        chain1.communicate();
        chain1.getServoPosition(1);
      break;
//    case LEFT_ARM_ELBOW:
//        chain1.setServotoLIM(2);
//        chain1.communicate();
//        chain1.getServoPosition(2);
//      break;
    case HEAD_YAW:
    	chain2.setServotoLIM(0);
	    chain2.communicate();
    	chain2.getServoPosition(0);
      break;
    case HEAD_ROLL:
    	chain2.setServotoLIM(1);
	    chain2.communicate();
    	chain2.getServoPosition(1);
      break;
    case RIGHT_ARM_PITCH:
    	chain3.setServotoLIM(0);
		chain3.communicate();
    	chain3.getServoPosition(0);
      break;
    case RIGHT_ARM_ROLL:
    	chain3.setServotoLIM(1);
		chain3.communicate();
    	chain3.getServoPosition(1);
      break;
//    case RIGHT_ARM_ELBOW:
//    	chain3.setServotoLIM(2);
//		chain3.communicate();
//    	chain3.getServoPosition(2);
//      break;
  }
}

void setEyesColor(byte red, byte green, byte blue, byte fadetime) {

  chain2.setLEDColor(red, green, blue, fadetime);
  chain2.communicate();
}

// Servo colors
const byte JOINT_BLACK = 0xF0;
const byte JOINT_RED = 0xF1;
const byte JOINT_GREEN = 0xF2;
const byte JOINT_BROWN = 0xF3;
const byte JOINT_BLUE = 0xF4;
const byte JOINT_VIOLET = 0xF5;
const byte JOINT_SEA = 0xF6;
const byte JOINT_WHITE = 0xF7;

// set the servo color
// for example, setJointColor(RIGHT_ARM_ELBOW, JOINT_VIOLET)
void setJointColor(byte jointName, byte color) {

  switch (jointName) {

    case LEFT_ARM_PITCH:
      chain1.setServoColor(0, color);
      chain1.communicate();
      break;
    case LEFT_ARM_ROLL:
      chain1.setServoColor(1, color);
      chain1.communicate();
      break;
    case LEFT_ARM_ELBOW:
      chain1.setServoColor(2, color);
      chain1.communicate();
      break;
    case HEAD_YAW:
      chain2.setServoColor(0, color);
      chain2.communicate();
      break;
    case HEAD_ROLL:
      chain2.setServoColor(1, color);
      chain2.communicate();
      break;
    case RIGHT_ARM_PITCH:
      chain3.setServoColor(0, color);
      chain3.communicate();
      break;
    case RIGHT_ARM_ROLL:
      chain3.setServoColor(1, color);
      chain3.communicate();
      break;
//    case RIGHT_ARM_ELBOW:
//      chain3.setServoColor(2, color);
//      chain3.communicate();
//      break;
  }
}

//The setup function is called once at startup of the sketch *************************************************************************************************
void setup()
{
	  pinMode(chainPin1, OUTPUT);
	  pinMode(chainPin2, OUTPUT);
	  pinMode(chainPin3, OUTPUT);


	  pinMode(PinLeftWirst, OUTPUT);
	  pinMode(PinLeftHand, OUTPUT);
	  pinMode(PinRightWirst, OUTPUT);
	  pinMode(PinRightHand, OUTPUT);


	  Serial.begin(9600);

	  Wire.begin(13);                // join i2c bus with address #8
	  Wire.onReceive(receiveEvent); // register event

	  //"Discover" all the modules (make them blue-colored instead of green-colored)
	  //for some unknown reason, I have to repeat it from time to time
	  for (int i = 0; i < 50; i++)
		  chain1.communicate();
	  for (int i = 0; i < 50; i++)
		  chain2.communicate();
	  for (int i = 0; i < 50; i++)
		  chain3.communicate();

	  //delay to be sure that all modules are ready
	  //if some module is "not discovered" than it will remain green and later this module will behave strangely
	  setJointColor(RIGHT_ARM_PITCH, JOINT_RED);
	  setJointColor(LEFT_ARM_PITCH, JOINT_RED);
	  setJointColor(RIGHT_ARM_ROLL, JOINT_RED);
	  setJointColor(LEFT_ARM_ROLL, JOINT_RED);
	  Serial.println("iniciando...");

	  //for (int i=0; i<MAX_SERVOS_MECANOID; i++)
		//  aPosServos[i].iValor = getJoint(i);

	  //Inicializamos posición de los servos

	  for (int j=0; j<10; j++)
		  for (int i=0; i<MAX_SERVOS_MECANOID; i++)
		  {
			  setJoint(i, valor_ini[i]);
			  aPosServos[i].iValor = valor_ini[i];
		  }

	  aServo[0].attach(PinRightWirst);
	  aServo[0].write(valor_ini[RIGHT_WRIST]);
	  aPosServos[RIGHT_WRIST].iValor = valor_ini[RIGHT_WRIST];

	  aServo[1].attach(PinRightHand);
	  aServo[1].write(valor_ini[RIGHT_HAND]);
	  aPosServos[RIGHT_HAND].iValor = valor_ini[RIGHT_HAND];

	  aServo[2].attach(PinLeftWirst);
	  aServo[2].write(valor_ini[LEFT_WRIST]);
	  aPosServos[LEFT_WRIST].iValor = valor_ini[LEFT_WRIST];

	  aServo[3].attach(PinLeftHand);
	  aServo[3].write(valor_ini[LEFT_HAND]);
	  aPosServos[LEFT_HAND].iValor = valor_ini[LEFT_HAND];

	  setJointColor(RIGHT_ARM_PITCH, JOINT_BLUE);
	  setJointColor(LEFT_ARM_PITCH, JOINT_BLUE);
	  setJointColor(RIGHT_ARM_ROLL, JOINT_BLUE);
	  setJointColor(LEFT_ARM_ROLL, JOINT_BLUE);

}

// The loop function is called in an endless loop *************************************************************************************************
int movimiento = 0;
int espera = 0;
void loop()
{


	if (espera == 0)
	{
		switch(movimiento)
		{
		case 0: //RIGHT_ARM_PITCH
			Serial.println(movimiento);
			setEyesColor(255,1,1,50);
			  chain1.communicate();
			EjecutarComando(COMMAND_POS, HEAD_YAW,120);
			EjecutarComando(COMMAND_POS, HEAD_ROLL,130);

			EjecutarComando(COMMAND_POS, RIGHT_ARM_ROLL,100);
			EjecutarComando(COMMAND_POS, LEFT_ARM_ROLL, 140);
			EjecutarComando(EXEC_PROG_MOV, 0, 0);
			movimiento += 1;
			espera = 400;
			break;
		case 1:
			Serial.println(movimiento);
			  EjecutarComando(COMMAND_POS, RIGHT_ARM_ROLL, 150);
				EjecutarComando(COMMAND_POS, LEFT_ARM_ROLL, 80);
			  EjecutarComando(EXEC_PROG_MOV, 0, 0);
			  movimiento += 1;
			  espera = 400;
			  break;
		case 2:
			Serial.println(movimiento);
			  EjecutarComando(COMMAND_POS, RIGHT_ARM_ROLL, 200);
				EjecutarComando(COMMAND_POS, LEFT_ARM_ROLL, 40);
			  EjecutarComando(EXEC_PROG_MOV, 0, 0);
			  movimiento = 99;
			  espera = 400;
			  break;
		case 99:
			Serial.println(movimiento);
			setEyesColor(255,255,255,0);
			espera = -1;
			movimiento = -1;
			break;
		}
	}

	if (espera != -1)
	{
		espera -= 1;
		ControlMovimientoTemporizado();
		delay(2);
	}
}

int comando = 0;
int disp = 0;
int pos_color = 0;

void receiveEvent(int howMany)
{
  comando = Wire.read();
  if ((comando == COMMAND_POS) || (comando == COMMAND_COLOR) || (comando == EXEC_PROG_MOV))
  {
	  delay(20);
	  disp = Wire.read();
	  pos_color = Wire.read();

	  Serial.print(comando);
	  Serial.print("-");
	  Serial.print(disp);
	  Serial.print("-");
	  Serial.println(pos_color);
	  //EjecutarComando(comando, disp, pos_color);
  }
}


void EjecutarMovimientoProgramado()
{
	  for (int indServo=0; indServo<MAX_SERVOS_POS; indServo++)
	  {
		  //Si tiene tiempo programado y no se está moviendo, asignamos ms de parada y lo activamos para moverse
		  if ((aPosServos[indServo].ms_tiempo_mov > 0) && !aPosServos[indServo].MovTemporizadoActivo)
		  {
			  aPosServos[indServo].ms_inicial = millis();
			  aPosServos[indServo].ms_final = aPosServos[indServo].ms_inicial + aPosServos[indServo].ms_tiempo_mov;
			  aPosServos[indServo].MovTemporizadoActivo = true;
		  }
	  }
}

void EjecutarComando(int comando, int disp, int pos_color)
{
  switch(comando)
  {
  case COMMAND_POS:
	  if (aPosServos[disp].MovTemporizadoActivo)
		  aPosServos[disp].MovTemporizadoActivo = false;

	  aPosServos[disp].iValoIniTemporizado = aPosServos[disp].iValor;
	  aPosServos[disp].iValorFinTemporizado = pos_color;
	  aPosServos[disp].ms_tiempo_mov = long(abs(aPosServos[disp].iValorFinTemporizado -
												   aPosServos[disp].iValoIniTemporizado))*1000 / GRADOS_SEG;
	  break;
  case COMMAND_COLOR:
	  setJointColor(disp, pos_color);
	  break;
  case COMMAND_HEAD_LED:
	  byte red, green, blue;
	  red = (pos_color & 1)*255;
	  green = (pos_color & 2)*255;
	  blue = (pos_color & 4)*255;
	  setEyesColor(red, green, blue, disp);
	  break;
  case EXEC_PROG_MOV:
	  EjecutarMovimientoProgramado();
	  break;
  }
}

void ControlMovimientoTemporizado()
{
	long ms = millis();
	  for (int indServo = 0; indServo < MAX_SERVOS_POS; indServo++)
	  {
		  if (aPosServos[indServo].MovTemporizadoActivo)
		  {
//			  Serial.print(millis());
//			  Serial.print("-");
//			  Serial.println(indServo);

			  if (ms > aPosServos[indServo].ms_final)
			  {
				  aPosServos[indServo].iValor = aPosServos[indServo].iValorFinTemporizado;
				  setJoint(indServo, aPosServos[indServo].iValor);
				  aPosServos[indServo].MovTemporizadoActivo = false;
				  aPosServos[indServo].ms_tiempo_mov = 0;
			  }
			  else
			  {
				  float Avance = 1 - float(abs((aPosServos[indServo].ms_final-ms))) / (aPosServos[indServo].ms_final-aPosServos[indServo].ms_inicial);
				  int iValor = aPosServos[indServo].iValoIniTemporizado + 1.0*(aPosServos[indServo].iValorFinTemporizado-aPosServos[indServo].iValoIniTemporizado)*Avance;

				  if (iValor != aPosServos[indServo].iValor)
				  {
					  aPosServos[indServo].iValor = iValor;
					  setJoint(indServo, iValor);
				  }
			  }
		  }
	  }
}



