#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

float Kp = 0.8;
float Kd = 0.9;
float Ki = 0.08;

int P;
int I;
int D;
int P2;
int D2;
int I2;
int lastError2 = 0;

int lastError = 0;

int ancho;
int alto;
int long area;
signed long int error;
unsigned long int error2;

int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 10;
int ENB = 5;

bool delante;

const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 160;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 110;

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
//  PID_control_lados();

}


void PID_control(){
  int i;
  uint16_t blocks;
  blocks = pixy.getBlocks();
  
  if(blocks){
    for(i=0; i<=blocks;i++){
      uint16_t ancho = pixy.blocks[i].width;
      uint16_t alto = pixy.blocks[i].height;
      uint16_t posX = pixy.blocks[i].x;
      area = ancho * alto;
      error = 11000 - area;     // referencia a 20 cm son 11000 del area
      error2 = 160 - posX;
      Serial.print(error); 
      Serial.print('\n'); 

      P = error;
      I = I + error;
      D = error  - lastError;
      lastError = error;
      //delante = true;
    
      int motorspeed = P*Kp + I*Ki + D*Kd;
    
      int motorspeeda = basespeeda + motorspeed;
      int motorspeedb = basespeedb - motorspeed;
    
      
      if((error >= 3000) && (error < 10000)){                                  // Si el objeto esta muy lejos, mover los motores a favor a las manecillas del reloj
        delante = true;
        forward_backwards(motorspeeda, motorspeedb, delante);
        break;
        }
      else if((error < -1000) && (error > -30000 )){                            // Si el objeto esta muy cerca, mover los motores en sentido contrario a las manecillas del reloj
        delante = false;
        forward_backwards(motorspeeda, motorspeedb, delante);
        break;
        }
      else if((error >= -1000) && (error < 3000)){       // Zona muerta
        forward_backwards(0, 0, delante); 
        break;
        }
      else if((error >= 10000) && (error < 13000)){      //No activarse con falsos positivos
        forward_backwards(0, 0, delante);
        break;
        }
      //forward_backwards(motorspeeda, motorspeedb, delante);
    }
  }
}/*
void PID_control_lados(){
  int j;
  uint16_t blocks;
  blocks = pixy.getBlocks();
  
  if(blocks){
    for(j=0; j<=blocks;j++){
      uint16_t posX = pixy.blocks[j].x;
      error2 = 160 - posX;            //posicion en el centro de la pantalla x = 160
      
      P2 = error2;
      I2 = I2 + error2;
      D2 = error2  - lastError2;
      lastError2 = error2;
    
      int motorspeed = P*Kp + I*Ki + D*Kd;
    
      int motorspeeda2 = basespeeda + motorspeed;
      int motorspeedb2 = basespeedb - motorspeed;
   
      forward_brake(motorspeeda2, motorspeedb2);
    }
  }
}*/


void forward_backwards(int posa, int posb, bool direccion){

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
  }
}

void forward_brake(int posx, int posy) {
    analogWrite(ENA,posx);
    analogWrite(ENB,posx);
    digitalWrite(in1, HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4,LOW);
}
