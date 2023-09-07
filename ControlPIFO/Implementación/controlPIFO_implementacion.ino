#include "MeanFilterLib.h"            // Libreria para filtro
#define m1 8 
#define m2 6  
#define mpwm 5
MeanFilter<float> meanFilter(10);

// Variables generales
int Pin_encoder_A = 2, Pin_encoder_B = 3;
volatile unsigned int cuenta=0;
volatile int32_t cuentas=0;
volatile int8_t last_state=0, current_state=0;
const float pi= 3.141592, ts=0.01;
const int cuentas_max= 6533;
volatile float th = 0, thp=0,dth=0, i=0, i_f=0, t_f=0;

// Variables para control PI de orden fraccionario
float kp = 50.0, ki= 700.0, cr=1.34, kii; //50 20
double r=0, u=0, e=0, up=0, ui=0, ud=0, ui_1=0, v;  // e_1=0
double a=0.8, c=0.0; 
float c1[7]= {0, 0, 0, 0, 0 ,0 ,0}, c2[7]= {0, 0, 0, 0, 0 ,0 ,0};
float r1[7]= {0, 0, 0, 0, 0 ,0 ,0}, r2[7]= {0, 0, 0, 0, 0 ,0 ,0}, r3[7]= {0, 0, 0, 0, 0 ,0 ,0};
float pu[7]= {0, 0, 0, 0, 0 ,0 ,0}, pe[7]= {0, 0, 0, 0, 0 ,0 ,0};
float p[4]={0, 0, 0, 0}, q[4]={0, 0, 0, 0};
float b2, b3, b4, b5, b6, b7;
float a1, a2, a3, a4, a5, a6, a7;
double u_1=0, u_2=0, u_3=0, u_4=0, u_5=0, u_6=0, e_1=0, e_2=0, e_3=0, e_4=0, e_5=0, e_6=0; 

void setup() {
  // Configurando pines
  pinMode(m1 ,OUTPUT);
  pinMode(m2,OUTPUT);
  pinMode(mpwm,OUTPUT);
  
  Serial.begin(9600);

  // Configuración de interrupción para timer de 10 ms  
  TCCR1A = 0;                
  TCCR1B = 0;                
  TCNT1  = 0;                
  OCR1A = 0x9B;                                   // 16MHz/1024/100Hz - 1 = 155 = 0x9B 
  TCCR1B |= (1 << WGM12)|(1<<CS10)|(1 << CS12);   // Modo CTC, con prescaler 1024  
  TIMSK1 |= (1 << OCIE1A);                        // Habilita interrupción por comparación
  
  cli();
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_A), Lectura_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_encoder_B), Lectura_Encoder, CHANGE);
  sei();
}

void loop() {
  
  // Se cargan los parámetros para el controlador PIFO usando la bandera 'cuentas'
  if(cuentas < 1){    
    c= pow((2/ts), a);
    poli3(a, p);
    poli3(-a, q);
    convul(p, q, c1); // pu
    convul(q, q, c2);
    // pe=Kp*conv(p,q) + Ki*c^(-1)*conv(q,q)
    kii=ki*pow(c,-1);
    multi(c1, kp, r1);
    multi(c2, kii, r2); 
    sumavc(r1, r2, r3); //pe
    multi(c1, (1.0/c1[0]), pu);
    multi(r3, (1.0/c1[0]), pe);
    b2=pu[1]; b3=pu[2]; b4=pu[3]; b5=pu[4]; b6=pu[5]; b7=pu[6];
    a1=pe[0]; a2=pe[1]; a3=pe[2]; a4=pe[3]; a5=pe[4]; a6=pe[5]; a7=pe[6];    
    }

  // Envio de señal para el motor 
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  analogWrite(mpwm, u);

  // Referencia de torque en Nm   
  r=0.25;

  // Se muestra el t
  Serial.println(t_f);
  Serial.println(u);
}

ISR(TIMER1_COMPA_vect){  
  cuenta++;                   // Bandera 'cuentas' 

  // Error
  e= cr*r - t_f;  
  // Ley de control PI FO 
  u= -b2*u_1-b3*u_2-b4*u_3-b5*u_4-b6*u_5-b7*u_6 +a1*e+a2*e_1+a3*e_2+a4*e_3+a5*e_4+a6*e_5+a7*e_6;
  // Saturador
  if(u >= 255){
    u= 255;
  }
  else if (u <= 0){
    u= 0;
  }
  // Actualizacion de parámetros
  u_6=u_5; u_5=u_4; u_4=u_3; u_3=u_2; u_2=u_1; u_1=u; 
  e_6=e_5; e_5=e_4; e_4=e_3; e_3=e_2; e_2=e_1; e_1=e;
  
  i= analogRead(A0);              // Sensor de corriente
  i_f = meanFilter.AddValue(i);   // Filtro media móvil 
  i_f = abs(i_f);
  i_f = i_f*(30.0/1023.0);        // Escalando con magnitud máxima del sensor
  t_f = (i_f-0.03)/0.072;         // Torque (kg.mm)
  t_f = t_f*9.81/1000;            // Torque (N.m)
}

void Lectura_Encoder()
{    
    if(digitalRead(Pin_encoder_A)==1)
      bitSet(current_state,1);
    else
      bitClear(current_state,1);
     
    if(digitalRead(Pin_encoder_B)==1)
      bitSet(current_state,0);
    else
      bitClear(current_state,0);
  
    if(last_state==3 && current_state==1)
      cuentas++;
    if(last_state==1 && current_state==0) 
      cuentas++;
    if(last_state==0 && current_state==2)
      cuentas++;
    if(last_state==2 && current_state==3)
      cuentas++;   
       
    if(last_state==3 && current_state==2)
      cuentas--;
    if(last_state==2 && current_state==0)
      cuentas--;
    if(last_state==0 && current_state==1)
      cuentas--;
    if(last_state==1 && current_state==3)
      cuentas--;
      
    last_state = current_state; 
}

// Función para convolución
void convul(float va[], float vb[], float y[])
{
    y[0,0] = va[0,0]*vb[0,0];
    y[0,1] = va[0,0]*vb[0,1]+va[0,1]*vb[0,0];
    y[0,2] = va[0,0]*vb[0,2]+va[0,1]*vb[0,1]+va[0,2]*vb[0,0];
    y[0,3] = va[0,0]*vb[0,3]+va[0,1]*vb[0,2]+va[0,2]*vb[0,1]+va[0,3]*vb[0,0];
    y[0,4] = va[0,1]*vb[0,3]+va[0,2]*vb[0,2]+va[0,3]*vb[0,1];
    y[0,5] = va[0,2]*vb[0,3]+va[0,3]*vb[0,2];
    y[0,6] = va[0,3]*vb[0,3];    
}

// Función para multiplicación de vector con escalar
void multi(float va[],  float n, float r[])
{
    r[0,0] = va[0,0]*n;
    r[0,1] = va[0,1]*n;
    r[0,2] = va[0,2]*n;
    r[0,3] = va[0,3]*n;
    r[0,4] = va[0,4]*n;
    r[0,5] = va[0,5]*n;
    r[0,6] = va[0,6]*n;    
}

// Función para obtener el polinomio de orden 3
void poli3(float a, float p[])
{
    p[0,0]= 1; 
    p[0,1]= -a; 
    p[0,2]= pow(a,2.0)/3.0;
    p[0,3]= -a/3.0;
  
}

// Función de  suma de vectores
void sumavc(float aa[], float bb[], float cc[])
{
  cc[0,0] = aa[0,0]+bb[0,0];
  cc[0,1] = aa[0,1]+bb[0,1];
  cc[0,2] = aa[0,2]+bb[0,2];
  cc[0,3] = aa[0,3]+bb[0,3];
  cc[0,4] = aa[0,4]+bb[0,4];
  cc[0,5] = aa[0,5]+bb[0,5];
  cc[0,6] = aa[0,6]+bb[0,6];
}
