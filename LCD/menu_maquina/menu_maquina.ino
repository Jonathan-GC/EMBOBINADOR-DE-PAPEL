/**
    LIBRERIAS NECESARIAS PARA EL FUNCIONAMIENTO DEL CODIGO
*/
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#include <Arduino.h>
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "BasicStepperDriver.h" // generic
#include <Servo.h>

#define STOPPER_PIN_Y 10
#define STOPPER_PIN_Z 11
#define RUN_PIN 9

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200



// X motor
#define DIR_X 2
#define STEP_X 5

// Y motor
#define DIR_Y 3
#define STEP_Y 6


// Z motor
#define DIR_Z 4
#define STEP_Z 7
#define limiteZ 800
// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

//pin del Servo Gripper
#define pinGripper 12
#define pinHumectador A3

short upGripper = 160, downGripper = 85, ceroGripper = 180;

// Target RPM for X axis motor
short MOTOR_X_RPM;
// Target RPM for Y axis motor
short MOTOR_Y_RPM;
// Target RPM for Z axis motor
short MOTOR_Z_RPM;

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




/**
    OBJETOS DE LAS LIBRERIAS
*/
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Configuracion del LCD I2C (puede ser necesario cambiar el primer valor con la direccion del LCD)

//****************************************
//definiciones de Funcionamiento
//****************************************
#define diametroTambor  42
#define pi              3.1416


//----------------------------------------


//****************************************
//Variables de Funcionamiento
//****************************************
short gradosMotor;
short goToHomeVar = false;
short goToAlimentar = false;
short goToDesactivar = false;
short selectorConfiguracion;
short startProduccion= false;
short restaurarFabricaVar=false;
short ModoAutomatico=false;

//----------------------------------------

//****************************************
//Funciones de Funcionamiento
//****************************************
void sacrGrados();
  




//----------------------------------------




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
  "2.INICIAR PRODUC",
  "3.Home          ",
  "4.Alimentar     ",
  "5.Cuadro mm     ",
  "6.Nro Cuadros   ",
  "7.Gripper Out#  ",
  "8.Mostrar Tiempo",
  "9.Velocidad     ",
  "10.Linea habiles",
  "11.Produccion # ",
  "12.Tiempo Goteo ",
  "13.Tiempo humect",
  "14.Gotear Manual",
  "15.Guardar      ",
  "16.Reset Fabrica",
  "17.Modo         "
  
};

const byte iMENU = COUNT(txMENU);                       // Numero de items/opciones del menu principal

enum eSMENU1 { Milliseconds, Seconds, Minutes, Hours }; // Enumerador de las opciones disponibles del submenu 1 (tienen que seguir el mismo orden que los textos)
const char *txSMENU1[] = {        // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
  "  Milisegundos  ",
  "    Segundos    ",
  "     Minutos    ",
  "     Horas      "
};

enum eSMENU2 {Automatico, Manual };  // Enumerador de las opciones disponibles del submenu 2 (tienen que seguir el mismo orden que los textos)
const char *txSMENU2[] = {        // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
  "   Automatico   ",
  "     Manual     "
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
  long  metrosEnrrollados;
  short lineasCargadas;
  short NroCuadros;
  short modoAutomatico;
  short tiempoGoteo;
  short tiempoHumectacion;
};
union MEMORY {     // Estructura UNION para facilitar la lectura y escritura en la EEPROM de la estructura STRUCT
  MYDATA d;
  byte b[sizeof(MYDATA)];
}
memory;


/**
   INICIO Y CONFIGURACION DEL PROGRAMA
*/
void setup() {
 

  Serial.begin(115200);
  
  
  pinMode(pENCO_SW,  INPUT_PULLUP);
  pinMode(pENCO_DT,  INPUT_PULLUP);
  pinMode(pENCO_CLK, INPUT_PULLUP);

  // Carga la configuracion de la EEPROM, y la configura la primera vez:
  readConfiguration();

  // Inicia el LCD:
  lcd.begin(columnsLCD, rowsLCD);
  lcd.createChar(iARROW, bARROW);

  
  // Imprime la informacion del proyecto:
  lcd.setCursor(0, 0); lcd.print("    Maquina.    ");
  lcd.setCursor(0, 1); lcd.print("  Embobinadora  ");
  delay (2000);  lcd.clear();

  lcd.setCursor(0, 0); lcd.print("    TecnoBot    ");
  lcd.setCursor(0, 1); lcd.print("Tel: 3054791784");
  delay (2000);  lcd.clear();

  lcd.setCursor(0, 0); lcd.print("  RegiBasculas  ");
  lcd.setCursor(0, 1); lcd.print("   del Tolima   ");
  delay (5000);  lcd.clear();
  
  
  lcd.setCursor(0, 0); lcd.print("  Configurando  ");
  lcd.setCursor(0, 1);
  for ( int i = 0 ; i < columnsLCD ; i++ )
  {
    lcd.print(".");
    delay(100);
  }
  lcd.clear();

  //Funcion para sacar los grados de la maquina segun papel
  
  gradosMotor = int(sacarGrados());
  Serial.print("Grados Motor: "); Serial.println(gradosMotor); 

  configurar_maquina();
  

}

 //char dato=0; linea para borrar
 unsigned long Contador = 0;
 unsigned long TiempoAhora=0, periodo=5000;


/**
   PROGRAMA PRINCIPAL
*/
void loop() {
  mostrarPantalla();

  if(goToAlimentar){
    funcionPrincipalMaquina('a');
    goToAlimentar = false;
  }
  else if(goToHomeVar){
    funcionPrincipalMaquina('h');
    goToHomeVar = false;
  }
  else if(startProduccion){
    funcionPrincipalMaquina('t');
    startProduccion=false;
  }

}


/**
    MUESTRA EL MENU PRINCIPAL EN EL LCD.
*/
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
        case 1: openSubMenu( idxMenu, Screen::Flag, &startProduccion, 0, 1); exitMenu = true; break;
        case 2: openSubMenu( idxMenu, Screen::Flag,   &goToHomeVar, 0, 1); exitMenu = true; break;
        case 3: openSubMenu( idxMenu, Screen::Number, &goToAlimentar,    0, 4);exitMenu = true; break; 
        case 4: openSubMenu( idxMenu, Screen::Number, &memory.d.tamanioCuadro, 80, 200); break;
        case 5: openSubMenu( idxMenu, Screen::Number, &memory.d.NroCuadros, 0, 20); break;
        case 6: openSubMenu( idxMenu, Screen::Number, &memory.d.vecesDelGripper, 0, 4); break;
        case 7: openSubMenu( idxMenu, Screen::Menu1,  &memory.d.time_unit, 0, COUNT(txSMENU1) - 1 ); break;
        case 8: openSubMenu( idxMenu, Screen::Number, &memory.d.velocidad, 70, 150); break;
        case 9: openSubMenu( idxMenu, Screen::Number, &memory.d.lineasCargadas, 0, 3); break;
        case 10: openSubMenu( idxMenu, Screen::Number, &memory.d.numeroDeProduccion, 0, 100 ); break;
        case 11: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempoGoteo, 0, 1000); break;
        case 12: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempoHumectacion, 0, 1000); break;
        
        case 13: goteoManual(); break;
        case 14: writeConfiguration(); exitMenu = true; break; //Salir y guardar
        case 15: lcd.clear();lcd.print("Reset Fabrica?");esperar(3000); 
                 openSubMenu( idxMenu, Screen::Flag, &restaurarFabricaVar, 0, 1);
                 restaurarFabrica(restaurarFabricaVar); restaurarFabricaVar=false; 
                 //realiza  el cambio nuevamente
                 restaurarFabrica(restaurarFabricaVar); 
                 exitMenu = true; 
                 break;
        case 16: openSubMenu( idxMenu, Screen::Menu2,  &memory.d.modoAutomatico, 0, COUNT(txSMENU2)-1 ); Serial.println(memory.d.modoAutomatico); break;        

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
        lcd.print(*value == 1 ? "SI" : "NO");
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
    Serial.println("entro en initialized");
    memory.d.initialized = 'Y';
    memory.d.time_show   = 1;
    memory.d.time_unit   = 2;
    memory.d.time_x      = 0;
    memory.d.time_y      = 0;

    memory.d.tamanioCuadro= 105;
    memory.d.velocidad = 100;
    memory.d.numeroDeCuadros = 10;
    memory.d.numeroDeProduccion = 5;
    memory.d.vecesDelGripper = 2;
    memory.d.metrosEnrrollados = 0;
    memory.d.lineasCargadas = 3;
    memory.d.NroCuadros=10;
    memory.d.modoAutomatico=false;
    memory.d.tiempoGoteo = 100;
    memory.d.tiempoHumectacion = 45;
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


void mostrarPantalla(){
  static unsigned long tNow      = 0;
  static unsigned long tPrevious = 0;

  tNow = millis();
  btnPressed = readButtons();
  

  //si presiona el boton despliega el menu
  if ( btnPressed == Button::Ok )
    openMenu();
  else if ( btnPressed == Button::Left )
    goteoManual();

  
  // Pinta la pantalla principal cada 1 segundo:
  
  if ( tNow - tPrevious >= 1000 )  {
    tPrevious = tNow;

    if ( memory.d.time_show == 1)
      lcd.clear();

    //Establecer el tiempor de maquina
    if ( memory.d.time_show == 1 )
    {
      lcd.print("Tiempo: ");
      lcd.setCursor(8, 0);
      switch ( memory.d.time_unit )      {
        case eSMENU1::Milliseconds:
          lcd.print(tNow);
          lcd.print(" Mil");
          break;
        case eSMENU1::Seconds:
          lcd.print(tNow / 1000);
          lcd.print(" Seg");
          break;
        case eSMENU1::Minutes:
          lcd.print(tNow / 1000 / 60);
          lcd.print(" Min");
          break;
        case eSMENU1::Hours:
          lcd.print(tNow / 1000 / 60 / 60);
          lcd.print(" Hor");
          break;
      }
    }

    //Establecer el perfil de los metros enrollados
    lcd.setCursor(0,1);
    lcd.print("Un: ");
    lcd.print(memory.d.metrosEnrrollados);
    lcd.print(" m: ");
    double metraje;
    metraje=(memory.d.tamanioCuadro*memory.d.NroCuadros);
    metraje*=memory.d.lineasCargadas;
    metraje/=1000;
    //Serial.println(metraje);
    lcd.print(memory.d.metrosEnrrollados*metraje);
  }

}

void restaurarFabrica(boolean value){
  if(value){
    //Restaurar memoria cambia el valor a N
    memory.d.initialized = 'N';
    //Lo escribe en la configuracion!
    writeConfiguration();

    //Vuelve y lo manda a leer para cargar los datos
    readConfiguration();
  }else{

    //Lo carga Con Y para que se pueda habilitar nuevamente la maquina 
    memory.d.initialized = 'Y';
    //Lo escribe en la configuracion!
    writeConfiguration();
    //Vuelve y lo manda a leer para cargar los datos
    readConfiguration();
    
  }
  
}


//void leerMovimientoLateral()
