float sacarGrados(){
  //return (memory.d.tamanioCuadro*360)/(diametroTambor * pi);
  unsigned int flag = memory.d.tamanioCuadro*360;
  unsigned int flag2 = (diametroTambor * pi);
  return (flag)/flag2;
}

void configurar_maquina(){
      //Configuracion del Servo
    Gripper.attach(pinGripper);

    //Configuracion del Bomba
    pinMode(pinHumectador, OUTPUT);

    
    //Estraer las velocidades de la EEProm
    extraerVelocidades(memory.d.velocidad);
    //Configuracion del motores
    //A raiz de que descubrí que con 40 puntos abajo de la referencia el motorX mueve bien
    //Se le quitan 40 puntos
    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
    stepperZ.begin(MOTOR_Z_RPM-200, MICROSTEPS);
    
    
    //Configuracion de pines finales de carrera
    pinMode(STOPPER_PIN_Y, INPUT_PULLUP);
    pinMode(STOPPER_PIN_Z, INPUT_PULLUP);
    pinMode(RUN_PIN, INPUT_PULLUP);
    
    
    Gripper.write(ceroGripper);
    gotear(20);
    habilitarMotores(false);
  
}

//Funcion para hablitar y desabilitar motores de una manera entendible
void habilitarMotores(boolean x){
  
  //Declaraciondel pin como salida
  pinMode(8,1);

  
  //Preguntamos por el codigo x si quiere encender o apagar con 1 apaga, con 0 enciende
  if(x){
    digitalWrite(8, 0);
    /* Para borrar en la actiulización *******************************************************
    Gripper.attach(pinGripper);
    ****************************************************/
  }
  else{
    digitalWrite(8, 1);
    
    /* Para borrar en la actiulización *******************************************************
    //Levantar el servo y luego desactivarlo
    Gripper.write(ceroGripper);
    esperar(600);
    Gripper.detach();
    ***************************************************************/
  }
  
}

void alimentarPapel(){
  habilitarMotores(1);
  controller.rotate(gradosMotor*1, 0, 0);
  habilitarMotores(0);
  
}

//void SecuenciaDeCorte(uint16_t medida, short grados, short tamañoCuadro){
void secuenciaDeCorte(int vueltas){

  //esto es interpuesto para corregir el drama del goto Home
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  
  mostrarPantalla();
  habilitarMotores(1);
  //Para que no se quede alimentando
  stepperX.begin(350, MICROSTEPS);
  controller.rotate((gradosMotor/2)*3,0,0);

  //para calibrar los motores nuevamente
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);

  //Modo automatico
  if(memory.d.modoAutomatico){

    //Esperar a que presione enter
    lcd.clear();
    habilitarMotores(false);
    while(digitalRead(RUN_PIN)){
      lcd.setCursor(0,0);
      lcd.print("Continuar:  RUN");
      lcd.setCursor(0,1);
      lcd.print("Humectar :  <<< ");


      
      //btnPressed = readButtons();
  

      //si presiona el boton despliega el menu goteo manual
      if (! digitalRead(pENCO_DT)){
        delay(100);
        goteoManual();
      }
      
    }
    delay(200);
    mostrarPantalla();
    habilitarMotores(true);
  }
  
  
  //controller.rotate(gradosMotor*(vueltas-1),int(360*((vueltas-1)*1)),0);
  //controller.rotate(gradosMotor*(vueltas-1),int(360*vueltas*1.1),0);
  controller.rotate(gradosMotor*(3), 450*3,0);
  controller.rotate(gradosMotor*(5), 375*5,0);
  controller.rotate((gradosMotor/2)*1,150*1,0);
  esperar(200);
  controller.rotate(-200*1,0,0);
  //Desplazar a Z
  stepperZ.rotate(-850);
  esperar(100);

  //Gotear  y humedecer
  gotear(memory.d.tiempoGoteo);

  stepperZ.rotate(850);
  esperar(20);

  //para borrar *****************************************************************
 // for(byte i=0;i < 1;i++){
    //bajar a corte
    //bajarAcorte();
    //esperar(1000);
    //stepperZ.rotate(-15);
    
    //Subir a corte
    //subirAcorte();
    //esperar(400);
  
  //}
  // *****************************************************************

  //stepperY.rotate(450*2);
  
  //Retrocede para evitar que se pegue el papel
  //controller.rotate(-90,0,0);
  //esperar(200);
  //
  
  //stepperZ.begin(100, MICROSTEPS);
  //Segunda bajada
  /*
  for(byte i=0;i < 0;i++){
    stepperZ.rotate(15);
    //bajar a corte
    bajarAcorte();
    esperar(800);
    
    //Subir a corte
    Gripper.write(upGripper);
    stepperZ.rotate(-10);
    esperar(100);
    
  }
  */
  
  //IR AL INICIO
  boolean flag = false;
  stepperY.begin(200, MICROSTEPS);
  do{
    stepperY.startRotate(9 * 360);
    flag = goToHome_Y();
  }while(!flag);

  
  //borrar********************************************************************
  //Funcion para extraer papel
  //extraerPapel();

  //controller.rotate(90,0,0);
  //parte pa ra 
  //flag = false;
 // do{
     //stepperZ.startRotate(5 * 360);
     //flag = goToHome_Z();
  //}while(!flag);
  
  //stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  //********************************************************************
  delay(1500);
}

void bajarAcorte(){
  Gripper.write(downGripper);
}

void subirAcorte(){    
  Gripper.write(upGripper);
}

boolean goToHome_Y(){
  habilitarMotores(true);
  if (digitalRead(STOPPER_PIN_Y) == 1){
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
  //stepperZ.rotate(40);

  if (memory.d.vecesDelGripper == 0){
    esperar(2000);
  }
  else{
    
  
    for(byte i = 0; i < memory.d.vecesDelGripper; i++){
      bajarAcorte();
      esperar(500);
      //stepperZ.rotate(-limiteZ);
      subirAcorte();
      esperar(1500);
  //para borrar**************************************************************************************************
  /*
      boolean flag = false;
      
      do{
        stepperZ.startRotate(9 * 360);
        flag = goToHome_Z();
      }while(!flag);
  */    
    }
    
  //para borrar**************************************************************************************************
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
    stepperY.begin(200, MICROSTEPS);
    do{
      stepperY.startRotate(9 * 360);
      flag = goToHome_Y();
    }while(!flag);

    flag = false;
    
    do{
      stepperZ.startRotate(10 * 360);
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

      //Ir a inicio
      goToHome();
      humectar();
      
      //ubicar motor para cortar
      stepperZ.rotate(-4500);
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
          esperar(500);
          
      }
      
      goToHome();
      habilitarMotores(0);
      
      
      
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
   //MOTOR_Z_RPM=dato;
   MOTOR_Z_RPM=400;
}

void humectar(){
  //lo mando al final para que no moje carriles
  gotear(memory.d.tiempoHumectacion);
  
  /* Para borrar ******************************************************************************
  //stepperZ.rotate(-limiteZ);
  Gripper.write(130);
  esperar(800);
  //goToHome();
  ******************************************************************************/
}

void gotear(short espera){
  digitalWrite(pinHumectador,1);
  esperar(espera);
  digitalWrite(pinHumectador,0);
}

void goteoManual(){
  lcd.clear();
  lcd.print("Enter para Salir");
  lcd.setCursor(0,1);
  lcd.print("RUN para Gotear");
  
  do{
    if(!digitalRead(RUN_PIN))
      digitalWrite(pinHumectador,1);
    else
      digitalWrite(pinHumectador,0);
  }while(digitalRead(pENCO_SW));

  delay(300);
}
