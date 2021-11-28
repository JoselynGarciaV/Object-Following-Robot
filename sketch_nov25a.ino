#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

float Kp = 0.07;
float Kd = 0.6;
float Ki = 0.0008;

int P;
int I;
int D;

int lastError = 0;

int ancho;
int alto;
int long area;
signed long int error;

int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 10;
int ENB = 5;

//bool delante;

/*
unsigned long tiempo = 0;
const long inicio = 2000;
const long tiempo_1 = 3000;
const long tiempo_2 = 4000;
const long tiempo_3 = 5000;
*/
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

void setup() {
  Serial.begin(9600);
  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  pixy.init();
}

void loop() {
  PID_control();
  /*
  if(area < 200){
    foward_backwards (0,0, false, false);
  }
  else{
    PID_control();
  }
  */
}


void PID_control(){
  bool delante = false;

  int i;
  uint16_t blocks;
  blocks = pixy.getBlocks();
  
  if(blocks){
    for(i=0; i<=blocks;i++){
      uint16_t ancho = pixy.blocks[i].width;
      uint16_t alto = pixy.blocks[i].height;
      area = ancho * alto;
      //pixy.blocks[i].print();
      //Serial.print(area);
      Serial.print('\n');
      error = 11000 - area;     // referencia a 20 cm son 11000 del area
      Serial.print(error);
      if(error >= 0){
        delante = true;
      }
      else if(error < 0){
        delante = false;
      }
    }
  }
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P*Kp + I*Ki + D*Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
//
//
//  if ((area >= 5000) && (area < 6000)){     //Si se encuentra dentro del rango no mover ni hacia adelante ni hacia atras
//    direccion = false;
//    lados = false;
//    motorspeeda = 0;
//    motorspeedb = 0;
//    
//  }
//  else if((area < 5000)&&(area >= 500)){
//    direccion = true; 
//    lados = false;                //Mover hacia delante
//    motorspeeda = maxspeeda;
//    motorspeedb = maxspeedb;
//    
//  }
//  else if(area >= 6000){            //Mover hacia atras
//    direccion = false;
//    lados = false;
//    motorspeeda = maxspeeda;
//    motorspeedb = maxspeedb;
//    
//  }
//  else if(area < 100){                         //Cuando no detecte el objeto mover hacia los lados
//    lados = true;
//    motorspeeda = 0;
//    motorspeedb = 0;
//  }
//  
//  //else{
// //   motorspeeda = 0;
//  motorspeedb = 0;
//}

  foward_backwards(motorspeeda, motorspeedb, delante);
}

void foward_backwards(int posa, int posb, bool direccion){
  //tiempo = millis();

  if(direccion == true){                // Si dir mover hacia adelante
    analogWrite(ENA,posa);
    analogWrite(ENB,posb);
    digitalWrite(in1, HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4,LOW);
  }
  else if(direccion == false){                   // else mover hacia atras
    analogWrite(ENA,posa);
    analogWrite(ENB,posb);
    digitalWrite(in1, LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4,HIGH);
  }/*
  if (lado == 1){
    analogWrite(ENA,posa);
    analogWrite(ENB,posb);
    digitalWrite(in1, HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4,HIGH);
  }*/
}
