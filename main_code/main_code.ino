#include <Arduino.h>
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "BasicStepperDriver.h" // generic
#include <Servo.h>

#define STOPPER_PIN_Y 10
#define STOPPER_PIN_Z 11

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 80
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 80
// Target RPM for Z axis motor
#define MOTOR_Z_RPM 80


// X motor
#define DIR_X 2
#define STEP_X 5

// Y motor
#define DIR_Y 3
#define STEP_Y 6


// Z motor
#define DIR_Z 4
#define STEP_Z 7
#define limiteZ 790
// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

//pin del Servo Gripper
#define pinGripper 12
#define repeticionGripper  2
short upGripper = 90, downGripper = 115;


/*************************************
  DECLARACION DE OBJETOS PRINCIPALES
*************************************/
//Declaracion de motores
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);

//Declaracion del controlador
SyncDriver controller(stepperX, stepperY, stepperZ);

// Declaracion del objeto Servo
Servo Gripper;
/*************************************
*************************************/

void setup() {
    Serial.begin(115200);
    Serial.println (joder);
    //Configuracion del Servo
    Gripper.attach(pinGripper);
  
    //Configuracion del motores
    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
    stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);

    //Configuracion de pines finales de carrera
    pinMode(STOPPER_PIN_Y, INPUT_PULLUP);
    pinMode(STOPPER_PIN_Z, INPUT_PULLUP);

    //*************************************
    //eSTE BLOQUE DEBE DE SER BORRADO PAR 
    //evitar problemas en el controlador
    
    //stepperY.startRotate(100 * -360);
    //stepperZ.startRotate(100 * -360);
    //stepperY.startRotate(10 * 360);
    //stepperZ.startRotate(10 * 360);
    //***********************************
    
    
    habilitarMotores(false);
  


}
 char dato=0;
 unsigned long Contador = 0;
 unsigned long TiempoAhora=0, periodo=5000;
 
 
void loop() {
   
   if (Serial.available()>0){
      dato=Serial.read();
      Serial.println (dato);
   }
   if (dato=='h'){
      goToHome ();    
      dato='s';
   }
   if (dato=='s'){
      habilitarMotores(0);
   }

   if (dato=='a'){
      alimentarPapel();
   }

   if (dato=='t'){
      secuenciaDeCorte(10);
      dato=0;
   }

   if (dato=='p'){
      habilitarMotores(1);
      
      dato=0;
   }

   if (dato=='y'){
      
      stepperY.startRotate(10 * 360);
      Serial.println(goToHome_Y());
      
   }

   if (dato=='z'){
      stepperZ.startRotate(10 * 360);
      Serial.println(goToHome_Z());
      
   }
   
   
   
}

void goToHome (){


  //Levantamos el Griper para evitar que se estrelle
    Gripper.write(upGripper);

    boolean flag = false;
    do{
      stepperY.startRotate(100 * -360);
      flag = goToHome_Y();
    }while(!flag);

    flag = false;
    
    do{
      stepperZ.startRotate(20 * 360);
      flag = goToHome_Z();
    }while(!flag);

   
    
}

void habilitarMotores(boolean x){
  //Declaraciondel pin como salida
  pinMode(8,1);

  //Preguntamos por el codigo x si quiere encender o apagar con 1 apaga, con 0 enciende
  if(x)
    digitalWrite(8, 0);
  else
    digitalWrite(8, 1);
}

void alimentarPapel(){
  habilitarMotores(1);
  controller.rotate(287*1, 0, 0);
  dato=0;
  
}

//void SecuenciaDeCorte(uint16_t medida, short grados, short tamañoCuadro){
void secuenciaDeCorte(int vueltas){
  Serial.print("entra: ");
  Serial.println(vueltas);
  
  habilitarMotores(1);
  controller.rotate(287*1,0,0);
  
  controller.rotate(287*(vueltas-1),int(360*((vueltas-1)*1.5)),0);
  controller.rotate(0,90,0);

  //Desplazar a Z
  stepperZ.rotate(-143);
  esperar(500);
  //bajar a corte
  bajarAcorte();
  esperar(1000);
  //Subir a corte
  subirAcorte();
  esperar(1000);
  controller.rotate(0,360*3,0);
  //esperar(2000);

  //IR AL INICIO
  boolean flag = false;
  do{
     stepperY.startRotate(10 * 360);
     flag = goToHome_Y();
  }while(!flag);


  //Funcion para extraer papel
  extraerPapel();

  flag = false;
  do{
     stepperZ.startRotate(10 * 360);
     flag = goToHome_Z();
  }while(!flag);


  dato=0;  
}

void bajarAcorte(){
  Gripper.write(120);
}

void subirAcorte(){
  Gripper.write(70);
}

boolean goToHome_Y(){
  habilitarMotores(true);
  if (digitalRead(STOPPER_PIN_Y) == 0){
        //Serial.println("STOPPER REACHED");
        //stepperY.startBrake();
        stepperY.stop();
        return true;
  }else{

    unsigned wait_time_micros = stepperY.nextAction();
    return false;
  }
  
}
boolean goToHome_Z(){
  habilitarMotores(true);
  if (digitalRead(STOPPER_PIN_Z) == 1){
        //Serial.println("STOPPER REACHED");
        //stepperY.startBrake();
        stepperZ.stop();
        return true;
  }else{

    unsigned wait_time_micros = stepperZ.nextAction();
    return false;
  }

}


void extraerPapel(){
  
  //Avance Para Extraer Pwerfectamente
  stepperZ.rotate(44);

  for(byte i = 0; i < repeticionGripper; i++){
    bajarAcorte();
    stepperZ.rotate(-limiteZ);
    subirAcorte();
    esperar(2000);
    stepperZ.rotate(limiteZ-20);
  }
  
  
  
}

void esperar(int periodo){
  TiempoAhora = millis();
  while(millis() < TiempoAhora+periodo){
    
  }   
}
