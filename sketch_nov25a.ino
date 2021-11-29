#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

float Kp = 5;
float Kd = 2;
float Ki = 0.8;

float Kp2 = 70;
float Kd2 = 50;
float Ki2 = 30;

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
//unsigned long int  error2;
long int  error2;
signed long int errorx;

int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 10;
int ENB = 5;

bool delante;

const uint8_t maxspeeda = 160;
const uint8_t maxspeedb = 160;
const uint8_t basespeeda = 110;
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
  //Serial.println(blocks);
  delay(20);
  
  if(blocks){
    for(i=0; i<=blocks;i++){
      uint16_t ancho = pixy.blocks[i].width;
      uint16_t alto = pixy.blocks[i].height;
      int posX = pixy.blocks[i].x;
      delay(20);
      area = ancho * alto;
      error = 11000 - area;     // referencia a 20 cm son 11000 del area
      errorx = 160 - posX;
      //error2 = abs (errorx);
      //Serial.println(error); 

      //Serial.println(errorx);

      P = error;
      I = I + error;
      D = error  - lastError;
      lastError = error;
      //delante = true;
    
      int motorspeed = P*Kp + I*Ki + D*Kd;
    
      int motorspeeda = /*basespeeda + */motorspeed;
      int motorspeedb = /*basespeedb - */motorspeed;
      //Serial.println(motorspeed);
    
      if ((errorx <= 80) && (errorx >= -80)){
        if((error >= 3000) && (error < 10000)){                                  // Si el objeto esta muy lejos, mover los motores a favor a las manecillas del reloj
          delante = true;
          forward_backwards(motorspeeda, motorspeedb, delante);
          break;
          }
        else if((error < -1000) && (error > -40000 )){                            // Si el objeto esta muy cerca, mover los motores en sentido contrario a las manecillas del reloj
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
      break;   
      }
      
      else{
        P2 = errorx;
        I2 = I2 + errorx;
        D2 = errorx - lastError2;
        lastError2 = errorx;
        
        int motorspeed2 = P2*Kp2 + I2*Ki2 + D2*Kd2;
        int motorspeeda2 = 0;
        int motorspeedb2 = 0;
        Serial.println(errorx);
        
        if (errorx <= -80){
          motorspeeda2 = motorspeed2;
          motorspeedb2 = motorspeed2;
          forward_brake(motorspeeda2, motorspeedb2,true);
        }
        else if(errorx >= 80){
          motorspeeda2 = motorspeed2;
          motorspeedb2 = motorspeed2;
          forward_brake(motorspeeda2, motorspeedb2,false);
        }
        //Serial.println(motorspeed2);
        break;
      }
    }
  }
  
  else if(!blocks){
    forward_brake(0, 0,false);
  }
}

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

void forward_brake(int posx, int posy, bool derizq) {
  if (derizq == true){
    analogWrite(ENA,posx);
    analogWrite(ENB,posy);
    digitalWrite(in1, HIGH); 
    digitalWrite(in2,LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4,HIGH);
  }
    
  else if(derizq == false){
    analogWrite(ENA,posx);
    analogWrite(ENB,posy);
    digitalWrite(in1, LOW); 
    digitalWrite(in2,HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4,LOW);
    
  }


    
}
