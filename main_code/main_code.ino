#include <Arduino.h>
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "BasicStepperDriver.h" // generic
#include <Servo.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Configuracion del LCD I2C (puede ser necesario cambiar el primer valor con la direccion del LCD)

#define STOPPER_PIN_Y 10
#define STOPPER_PIN_Z 11

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 100
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 100
// Target RPM for Z axis motor
#define MOTOR_Z_RPM 100


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
short upGripper = 160, downGripper = 90, ceroGripper = 180;


//****************************************
//definiciones de Funcionamiento
//****************************************
#define diametroTambor  44
#define pi              3.1416


//----------------------------------------


//****************************************
//Variables de Funcionamiento
//****************************************
short gradosMotor;
short goToHomeVar = false;
short goToAlimentar = false;
short selectorConfiguracion;
short startProduccion= false;
//----------------------------------------

//****************************************
//Funciones de Funcionamiento
//****************************************
void sacrGrados();
  




//----------------------------------------

/******************************************
 * LCD
 *****************************************/

/**
    MACROS, CONSTANTES, ENUMERADORES, ESTRUCTURAS Y VARIABLES GLOBALES
*/
#define COUNT(x) sizeof(x)/sizeof(*x)                   // Macro para contar el numero de elementos de un array
const byte pENCO_SW   = A0;                              // Pin encoder SW
const byte pENCO_DT   = A1;                              // Pin encoder DT
const byte pENCO_CLK  = A2;                              // Pin encoder CLK
const byte rowsLCD    = 2;                              // Filas del LCD
const byte columnsLCD = 16;                             // Columnas del LCD
const byte iARROW     = 0;                              // ID icono flecha

const byte bARROW[]   = {                               // Bits icono flecha
  B00000, 
  B00100, 
  B00110, 
  B11111,
  B00110, 
  B00100, 
  B00000, 
  B00000
};

enum Button { Unknown, Ok, Left, Right } btnPressed;    // Enumerador con los diferentes botones disponibles
enum Screen { Menu1, Menu2, Menu3, Flag, Number };             // Enumerador con los distintos tipos de submenus disponibles

const char *txMENU[] = {                                // Los textos del menu principal, la longitud maxima = columnsLCD-1, rellenar caracteres sobrantes con espacios.
  "1.Atras <<      ",
  "2.Home          ",
  "3.Alimentar     ",
  "4.Velocidad     ",
  "5.Cuadro mm     ",
  "6.Produccion #  ",
  "7.Gripper Out#  ",
  "8.Mostrar Tiempo",
  "9.Lineas habiles", 
  "10.Guardar      ",
  "11.Salir        ",
  "12 Iniciar Prod "
};

const byte iMENU = COUNT(txMENU);                       // Numero de items/opciones del menu principal

enum eSMENU1 { Milliseconds, Seconds, Minutes, Hours }; // Enumerador de las opciones disponibles del submenu 1 (tienen que seguir el mismo orden que los textos)
const char *txSMENU1[] = {        // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
  "  Milisegundos  ",
  "    Segundos    ",
  "     Minutos    ",
  "     Horas      "
};

enum eSMENU2 { GradeC, GradeF };  // Enumerador de las opciones disponibles del submenu 2 (tienen que seguir el mismo orden que los textos)
const char *txSMENU2[] = {        // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
  "     Grados C     ",
  "     Grados F     "
};

/* ESTRUCTURAS CONFIGURACION */
struct MYDATA {         // Estructura STRUCT con las variables que almacenaran los datos que se guardaran en la memoria EEPROM
  int sizeCuadre;
  
  
  int initialized;
  int time_show;
  int time_unit;
  int time_x;
  int time_y;
  int temp_show;
  int temp_unit;
  int temp_x;
  int temp_y;
  short tamanioCuadro;
  short velocidad;
  short numeroDeCuadros;
  short numeroDeProduccion;
  short vecesDelGripper;
  long metrosEnrrollados;
  short lineasCargadas;
};
union MEMORY {     // Estructura UNION para facilitar la lectura y escritura en la EEPROM de la estructura STRUCT
  MYDATA d;
  byte b[sizeof(MYDATA)];
}
memory;




//-----------------------------------------

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
    
    Gripper.write(ceroGripper);
    habilitarMotores(false);

    /*METER lo del LCD*/
    pinMode(pENCO_SW,  INPUT_PULLUP);
    pinMode(pENCO_DT,  INPUT_PULLUP);
    pinMode(pENCO_CLK, INPUT_PULLUP);
  
    // Carga la configuracion de la EEPROM, y la configura la primera vez:
    readConfiguration();
  
    // Inicia el LCD:
    lcd.begin(columnsLCD, rowsLCD);
    lcd.createChar(iARROW, bARROW);

    /*
    // Imprime la informacion del proyecto:
    lcd.setCursor(0, 0); lcd.print("    Maquina.    ");
    lcd.setCursor(0, 1); lcd.print("  Embobinadora  ");
    delay (2000);  lcd.clear();
  
    lcd.setCursor(0, 0); lcd.print("    TecnoBot    ");
    lcd.setCursor(0, 1); lcd.print("Tel: 3054791784");
    delay (2000);  lcd.clear();
  
    lcd.setCursor(0, 0); lcd.print(" Regi  Basculas ");
    lcd.setCursor(0, 1); lcd.print("   del Tolima   ");
    delay (5000);  lcd.clear();
    */

    lcd.setCursor(0, 0); lcd.print("  Configurando  ");
    lcd.setCursor(0, 1);
    for ( int i = 0 ; i < columnsLCD ; i++ )
    {
      lcd.print(".");
      delay(100);
    }
    lcd.clear();
    
    


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

      for(int i= 0; i < 5; i++ ){
          secuenciaDeCorte(10);
          delay(3000);
      }
      dato='s';  
      
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
  controller.rotate(310*1, 0, 0);
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
  stepperZ.rotate(-150);
  esperar(20);
  //bajar a corte
  bajarAcorte();
  //delay(2000);
  esperar(800);
  //Subir a corte
  subirAcorte();
  //delay(2000);
  esperar(500);
  bajarAcorte();
  //delay(2000);
  esperar(800);
  //Subir a corte
  subirAcorte();
  //delay(2000);
  esperar(500);
  controller.rotate(0,287*3,0);
  esperar(500);

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

  for(byte i = 0; i < repeticionGripper; i++){
    bajarAcorte();
    delay(2000);
    stepperZ.rotate(-limiteZ);
    subirAcorte();
    delay(2000);
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
  dato='s';
void openMenu() {
  byte idxMenu       = 0;
  boolean exitMenu   = false;
  boolean forcePrint = true;

  lcd.clear();

  while ( !exitMenu )  {
    btnPressed = readButtons();

    if ( btnPressed == Button::Left && idxMenu - 1 >= 0 )    {
      idxMenu--;
    }
    else if ( btnPressed == Button::Right && idxMenu + 1 < iMENU )    {
      idxMenu++;
    }
    else if ( btnPressed == Button::Ok )    {
      switch ( idxMenu )
      {
        //openSubMenu( byte menuID, Screen screen, int *value, int minValue, int maxValue )
        case 0: readConfiguration();  exitMenu = true; break; //Salir y cancelar cambios
        case 1: openSubMenu( idxMenu, Screen::Flag,   &goToHome, 0, 1); break;
        case 2: openSubMenu( idxMenu, Screen::Number, &goToAlimentar,    0, 4); break; 
        case 3: openSubMenu( idxMenu, Screen::Number, &memory.d.velocidad, 70, 150); break;
        case 4: openSubMenu( idxMenu, Screen::Number, &memory.d.tamanioCuadro, 80, 200); break;
        case 5: openSubMenu( idxMenu, Screen::Number, &memory.d.numeroDeProduccion, 0, 100 ); break;
        case 6: openSubMenu( idxMenu, Screen::Number, &memory.d.vecesDelGripper, 0, 4); break;
        case 7: openSubMenu( idxMenu, Screen::Menu1,  &memory.d.time_unit, 0, COUNT(txSMENU1) - 1 ); break;
        case 8: openSubMenu( idxMenu, Screen::Number, &memory.d.lineasCargadas, 1, 3); break;
        case 9: writeConfiguration(); exitMenu = true; break; //Salir y guardar
        case 10: readConfiguration();  exitMenu = true; break; //Salir y cancelar cambios
        case 11: openSubMenu( idxMenu, Screen::Flag, &startProduccion, 0, 1); break;
        //Queda pendiente el default                                             break; //Salir y guardar
        
      }
      forcePrint = true;
    }


    if ( !exitMenu && (forcePrint || btnPressed != Button::Unknown) )
    {
      forcePrint = false;

      static const byte endFor1 = (iMENU + rowsLCD - 1) / rowsLCD;
      int graphMenu     = 0;

      for ( int i = 1 ; i <= endFor1 ; i++ )
      {
        if ( idxMenu < i * rowsLCD )
        {
          graphMenu = (i - 1) * rowsLCD;
          break;
        }
      }

      byte endFor2 = graphMenu + rowsLCD;

      for ( int i = graphMenu, j = 0; i < endFor2 ; i++, j++ )
      {
        lcd.setCursor(1, j);
        lcd.print( (i < iMENU) ? txMENU[i] : "                    " );
      }

      for ( int i = 0 ; i < rowsLCD ; i++ )
      {
        lcd.setCursor(0, i);
        lcd.print(" ");
      }
      lcd.setCursor(0, idxMenu % rowsLCD );
      lcd.write(iARROW);
    }
  }

  lcd.clear();
}


/**
   MUESTRA EL SUBMENU EN EL LCD.

   @param menuID    ID del menu principal para usarlo como titulo del submenu
   @param screen    Segun el tipo, se representara el submenu de una forma u otra.
   @param value     Puntero a la variable que almacena el dato, y que se modificara.
   @param minValue  Valor minimo que puede tener la variable.
   @param maxValue  Valor maximo que puede tener la variable.
*/
void openSubMenu( byte menuID, Screen screen, int *value, int minValue, int maxValue ) {
  boolean exitSubMenu = false;
  boolean forcePrint  = true;

  lcd.clear();

  while ( !exitSubMenu )
  {
    btnPressed = readButtons();

    if ( btnPressed == Button::Ok )
    {
      exitSubMenu = true;
    }
    else if ( btnPressed == Button::Left && (*value) - 1 >= minValue )
    {
      (*value)--;
    }
    else if ( btnPressed == Button::Right && (*value) + 1 <= maxValue )
    {
      (*value)++;
    }


    if ( !exitSubMenu && (forcePrint || btnPressed != Button::Unknown) )
    {
      forcePrint = false;

      lcd.setCursor(0, 0);
      lcd.print(txMENU[menuID]);

      lcd.setCursor(0, 1);
      lcd.print("<");
      lcd.setCursor(columnsLCD - 1, 1);
      lcd.print(">");

      if ( screen == Screen::Menu1 )
      {
        lcd.setCursor(1, 1);
        lcd.print(txSMENU1[*value]);
      }
      else if ( screen == Screen::Menu2 )
      {
        lcd.setCursor(1, 1);
        lcd.print(txSMENU2[*value]);
      }
      else if ( screen == Screen::Flag )
      {
        lcd.setCursor(columnsLCD / 2 - 1, 1);
        lcd.print(*value == 0 ? "SI" : "NO");
      }
      else if ( screen == Screen::Number )
      {
        lcd.setCursor(columnsLCD / 2 - 1, 1);
        lcd.print(*value);
        lcd.print(" ");
      }
    }

  }

  lcd.clear();
}


/**
    LEE (Y CONFIGURA LA PRIMERA VEZ) LA MEMORIA EEPROM CON LA CONFIGURACION DE USUARIO
*/
void readConfiguration()
{
  for ( int i = 0 ; i < sizeof(memory.d) ; i++  )
    memory.b[i] = EEPROM.read(i);

  if ( memory.d.initialized != 'Y' )  {
    memory.d.initialized = 'Y';
    memory.d.time_show   = 1;
    memory.d.time_unit   = 1;
    memory.d.time_x      = 0;
    memory.d.time_y      = 0;

    memory.d.tamanioCuadro= 105;
    memory.d.velocidad = 100;
    memory.d.numeroDeCuadros = 10;
    memory.d.numeroDeProduccion = 5;
    memory.d.vecesDelGripper = 2;
    memory.d.metrosEnrrollados = 0;
    memory.d.lineasCargadas = 3;

    writeConfiguration();
  }
}


/**
    ESCRIBE LA MEMORIA EEPROM CON AL CONFIGURACION DE USUARIO
*/
void writeConfiguration()
{
  for ( int i = 0 ; i < sizeof(memory.d) ; i++  )
    EEPROM.write( i, memory.b[i] );
}


/**
    LEE LOS DISTINTOS BOTONES DISPONIBLES Y DEVUELVE EL QUE HAYA SIDO PULSADO
        Este bloque de codigo varia dependiendo del tipo de teclado conectado, en mi blog estan
        disponibles los ejemplos para teclados digitales, analogicos, y este para encoder rotatorio.
        Al cambiar de tipo de teclado hay que adaptar tambien el resto de codigo para que haga uso de cada boton segun queramos.
*/
Button readButtons() {
  
  static boolean oldA = HIGH;
  static boolean newA = LOW;
  static boolean newB = LOW;

  btnPressed = Button::Unknown;
  newA = digitalRead(pENCO_DT);
  newB = digitalRead(pENCO_CLK);

  if ( !oldA && newA )
  {
    btnPressed = !newB ? Button::Left : Button::Right;
    delay(50);
  }
  else if ( !digitalRead(pENCO_SW) )
  {
    while (!digitalRead(pENCO_SW));
    btnPressed = Button::Ok;
    delay(50);
  }

  oldA = newA;
  return btnPressed;
}

short sacarGrados(){
  return int((memory.d.tamanioCuadro*360)/(diametroTambor * pi));
}
