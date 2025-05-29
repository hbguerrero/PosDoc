#include <Arduino.h>


#define PWM_FREQ 1000
#define PWMRESOLUTION 12

//Tomado de: https://roboticoss.com/encoder-para-robotica/
const int C1 = 32; // Entrada de la señal A del encoder.
const int C2 = 33; // Entrada de la señal B del encoder.
const int PWM1 = 13;
const int PWM2 = 12;
const int MOTOR0 = 18; //IZQUIERDA
const int MOTOR1 = 19; //DERECHA


volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 10;  // Tiempo de muestreo

double P = 0;
double R = 35020;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
//double R = 22660;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
//double R = 1460;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
double d = 0;
double pwm =0;
int pwm0 = 0;

void encoder(void);

void setup()
{
  Serial.begin(9600);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(MOTOR0, OUTPUT);
  pinMode(MOTOR1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);
  //Serial.println("Numero de conteos");
}

void loop() {
  if (millis() - lastTime >= sampleTime || lastTime==0)
  {  // Se actualiza cada sampleTime (milisegundos)
      lastTime = millis();
      P = (n*360.0)/R;
      //Serial.print("Position: ");Serial.print(P);Serial.print("°; ");
      //Serial.print("d: ");Serial.print(d);Serial.print("; ");
      //Serial.print("pmw: ");Serial.print(pwm);Serial.println("; ");
   }
    
  if (Serial.available() >0){
    String input = Serial.readStringUntil('\n');
    d = input.toDouble();    
  }
   // VALORES NEGATIVOS GIRA DERECHA, VALORS POSITIVOS GIRA IZQUIERDA
  if (d <= 45.0 && d >=-45.0) {    
    if (P>d){
      analogWrite(PWM1, 0); //retrocede o derecha
      analogWrite(PWM2, 64);
      delay(40);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);                            
     
      if(P >= -2 && P <= 2) { 
      analogWrite(MOTOR1, 128);
      analogWrite(MOTOR0, 128);
      }
      if(P < -1) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 128);
      }
      if(P > 1) { 
      analogWrite(MOTOR1, 128);
      analogWrite(MOTOR0, 64);
      }     
     }
     
    else if (P<=d){      
      if (P>d-0.7 ){
        analogWrite(PWM1, 0); //para
        analogWrite(PWM2, 0);
      }
      else{        
        analogWrite(PWM1, 64);  //avance o izquierda
        analogWrite(PWM2, 0);
        delay(40);
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);        
      }

      if(P >= -2 && P <= 2) { 
      analogWrite(MOTOR1, 128);
      analogWrite(MOTOR0, 128);
      }
      if(P < -2) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 128);
      }
      if(P > 2) { 
      analogWrite(MOTOR1, 128);
      analogWrite(MOTOR0, 64);
      }      
    }
  
  else {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  } 
  } 
}

// Encoder precisión cuádruple.
void encoder(void)
{
  
  
  ant=act;
  
  if(digitalRead(C1)) bitSet(act,1); else bitClear(act,1);            
  if(digitalRead(C2)) bitSet(act,0); else bitClear(act,0);

    if(n < R) {
    if(ant == 2 && act ==0) n++;
    if(ant == 0 && act ==1) n++;
    if(ant == 3 && act ==2) n++;
    if(ant == 1 && act ==3) n++;
  }
  else {n = 0;}

  if(n > -R){
    if(ant == 1 && act ==0) n--;
  if(ant == 3 && act ==1) n--;
  if(ant == 0 && act ==2) n--;
  if(ant == 2 && act ==3) n--;    
  }
  else {n = 0;}      
}
