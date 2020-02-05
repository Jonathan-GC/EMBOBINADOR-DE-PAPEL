float sacarGrados(){
  //return (memory.d.tamanioCuadro*360)/(diametroTambor * pi);
  unsigned int flag = memory.d.tamanioCuadro*360;
  unsigned int flag2 = (diametroTambor * pi);
  return (flag)/flag2;
}

void configurar_maquina(){
      //Configuracion del Servo
    Gripper.attach(pinGripper);

    //Estraer las velocidades de la EEProm
    extraerVelocidades(memory.d.velocidad);
    //Configuracion del motores
    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
    stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);

    //Configuracion de pines finales de carrera
    pinMode(STOPPER_PIN_Y, INPUT_PULLUP);
    pinMode(STOPPER_PIN_Z, INPUT_PULLUP);

    Gripper.write(ceroGripper);
    habilitarMotores(false);
  
}

//Funcion para hablitar y desabilitar motores de una manera entendible
void habilitarMotores(boolean x){
  
  //Declaraciondel pin como salida
  pinMode(8,1);

  //Preguntamos por el codigo x si quiere encender o apagar con 1 apaga, con 0 enciende
  if(x){
    digitalWrite(8, 0);
    Gripper.attach(pinGripper);
  }
  else{
    digitalWrite(8, 1);
    //Levantar el servo y luego desactivarlo
    Gripper.write(ceroGripper);
    esperar(600);
    Gripper.detach();
  }
}

void alimentarPapel(){
  habilitarMotores(1);
  controller.rotate(gradosMotor*1, 0, 0);
  habilitarMotores(0);
  
}

//void SecuenciaDeCorte(uint16_t medida, short grados, short tamañoCuadro){
void secuenciaDeCorte(int vueltas){
  Serial.print("entra: ");
  Serial.println(vueltas);

  mostrarPantalla();
  
  habilitarMotores(1);
  controller.rotate(gradosMotor*1,0,0);
  
  controller.rotate(gradosMotor*(vueltas-1),int(360*((vueltas-1)*1.5)),0);



  //Desplazar a Z
  stepperZ.rotate(-150);
  esperar(20);

  for(byte i=0;i < 3;i++){
    //bajar a corte
    bajarAcorte();
    esperar(400);
    
    //Subir a corte
    subirAcorte();
    esperar(400);
  }

  controller.rotate(0,gradosMotor*2,0);
  //Retrocede para evitar que se pegue el papel
  controller.rotate(-60,0,0);
  esperar(500);

  //IR AL INICIO
  boolean flag = false;
  do{
     stepperY.startRotate(10 * 360);
     flag = goToHome_Y();
  }while(!flag);

  

  //Funcion para extraer papel
  extraerPapel();

  controller.rotate(60,0,0);
  flag = false;
  do{
     stepperZ.startRotate(10 * 360);
     flag = goToHome_Z();
  }while(!flag);


  
}

void bajarAcorte(){
  Gripper.write(downGripper);
}

void subirAcorte(){
  Gripper.write(upGripper);
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

  for(byte i = 0; i < memory.d.vecesDelGripper; i++){
    bajarAcorte();
    esperar(1000);
    stepperZ.rotate(-limiteZ);
    subirAcorte();
    esperar(2000);
    //esperar(2000);
    stepperZ.rotate(limiteZ-20);
  }
  
  
  
}

void esperar(int periodo){
  TiempoAhora = millis();
  while(millis() < TiempoAhora+periodo){
    
  }   
}

void moverEjeX(){
  //stepperX.rotate(360);\\
  habilitarMotores(1);
  controller.rotate(360*50,0,0);
  habilitarMotores(0);
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


void funcionPrincipalMaquina(char dato){
  /*
   if (Serial.available()>0){
      dato=Serial.read();
      Serial.println (dato);
   }
   */
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
      //Variables para humectar
      int i_anterior=0, repeticiones = 3;
      
      for(int i= 0; i < memory.d.numeroDeProduccion; i++ ){
          secuenciaDeCorte(memory.d.NroCuadros);
          //Muestra la cantidad dispensada, + 1 para que no de cero de inicio
          cantidadDispensada(i+1);
          
          if((i-i_anterior) >= repeticiones){
            humectar();
            i_anterior = i;
          }
          esperar(3000);
          
      }
      habilitarMotores(false);
      
      
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

   if (dato=='u'){
      subirAcorte();
      
      
   }

   if (dato=='d'){
      bajarAcorte();
      
   }
   if (dato=='m'){
    habilitarMotores(1);
    moverEjeX();
   }
   
   
}

void cantidadDispensada(int factor){
  //memory.d.metrosEnrrollados = memory.d.numeroDeCuadros*memory.d.tamanioCuadro*memory.d.lineasCargadas*factor;
  unsigned long flag;
  flag = memory.d.lineasCargadas * factor;
  memory.d.metrosEnrrollados = flag;
  writeConfiguration();
}

void extraerVelocidades(short dato){
    // Target RPM for X axis motor
   MOTOR_X_RPM=dato;
  // Target RPM for Y axis motor
   MOTOR_Y_RPM=dato;
  // Target RPM for Z axis motor
   MOTOR_Z_RPM=dato;
}

void humectar(){
  //lo mando al final para que no moje carriles
  stepperZ.rotate(-limiteZ);
  Gripper.write(110);
  esperar(3000);
  goToHome();
  
}
