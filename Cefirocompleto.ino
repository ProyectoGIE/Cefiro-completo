/*
***********************************************************************
***********************************************************************
 *           
 *           **********      **********     **********
 *           **********      **********     **********
 *           **                  **         **
 *           **                  **         **
 *           **   *****          **         **********
 *           **   *****          **         **********
 *           **      **          **         ** 
 *           **      **          **         **
 *           **********  **  ********** **  **********
 *           **********  **  ********** **  **********
 *               GRUPO DE INNOVACIÓN EDUCATIVA
************************************************************************ 
************************************************************************
                    ESTACIÓN METEOROLÓGICA
     ESTE PROYECTO HA SIDO REALIZADO PARA EL PROYECTO GLOBE - CLIMA
                FEBRERO - MARZO y ABRIL DE 2016
*/

int UVOUT = A5; // Salida del sensor ML8511
int REF_3V3 = A4; // Usamos 3,3 voltios para aumentar la resolución y precisión del sensor


#include <SFE_BMP180.h> 
/*Cargamos las librerías del sensor de presión BMP180
 * Puede encontrarse una descripción más detallada de sus características 
 * técnicas en la página web:
 * http://www.mouser.es/ProductDetail/Bosch-Sensortec/BMP180/?qs=d72FGnIDsgTlLIC5YM2WKA%3D%3D
 */
#include <Wire.h> 
/* Cargamos la librería encargada de gestionar los protocolos I2C. Debe tenerse en
 *  cuenta que en los Arduinos UNO la SDA (Línea de datos) y la de reloj (SCL) están 
 *  en el puerto ananlógico 4 (SDA) y 5 (SCL). En otros Arduinos nos encontramos con 
 *  los siguientes puertos:
Uno, Ethernet A4 (SDA), A5 (SCL)
Mega2560  20 (SDA), 21 (SCL)
Leonardo  2 (SDA), 3 (SCL)
Due 20 (SDA), 21 (SCL), SDA1, SCL1
Si repasamos un poco, la tecnología I2C fue desarrollada por Philips y cuenta con 7 
bits de datos y uno de reloj, lo cual nos permite enganchar en paralelo un total de
2^7=128 dispositivos en el mismo puerto.
 */
#include <LiquidCrystal_I2C.h>    
/*
 * LiquidCrystal es el driver/librería necesario para usar la pantalla LCD bajo tecnología I2C.
 */

#include <DHT11.h>
/*
 * DHT11.h es la librería correspondiente al sensor de temperatura y humedad relativa que estamos 
 * usando.* 
 */
#include <math.h>
/*
 * Usamos math.h para que Arduino incluya las librerías matemáticas que nos van a permitir cal-
 * cular el punto de rocío, imprescindible para determinar en función de la temperatura y
 * humedad, la aparición de niebla, dato importante en navegación y aeronaútica.
 */

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Dirección de nuestra pantalla LCD
int pin=5; // Declaramos una variable de tipo entero y la asociamos a la línea de reloj (SCL) del Arduino
DHT11 dht11(pin); // Llamamos al sensor de temperatura y humedad y leemos.
SFE_BMP180 pressure; // Igual hacemos con el de presión.
//Se declaran las variables. Es necesario tomar en cuenta una presión inicial
//esta será la presión que se tome en cuenta en el cálculo de la diferencia de altura
double PresionBase;
//Leeremos presión y temperatura. Calcularemos la diferencia de altura
double Presion = 0; // Usamos variables tipo double que ralentizan el cálculo pero que son necesarias
double Altura = 0;  // y les asignamos un valor de cero inicialmente
double Temperatura = 0;
char status;



 
#include <SD.h>
/*
 * Esta librería es la que se encarga de cargar los drivers para usar el almacenamiento de datos 
 * en la tarjeta SD.
 */
// definimos el pin que enciende la SD
#define MEM_PW 8


File myFile; 




void setup() {            // La sentencia setup inicializa todo el entorno del software
   Serial.begin(9600);     // Ponemos la velocidad de trasmisión del puerto serie a 9600 baudios
   Serial.print("Iniciando tarjeta SD ...");  //Activamos la SD y mandamos este mensaje a la pantalla del PC
   pinMode(UVOUT, INPUT);
   pinMode(REF_3V3, INPUT);    // CTRL + SHIFT + M inicializa desde la ide de arduino la lectura
                               // a través del puerto serie
   pinMode(MEM_PW, OUTPUT);   // Definimos la función del pin (entrada de datos o salida de los mismos)
   digitalWrite(MEM_PW, HIGH);  // Lo ponemos en estado lógico 1
  
  if (!SD.begin(10)) {
    Serial.println("Algo falla!"); //En caso de algún fallo, manda mensaje al PC
    return;
  }
  Serial.println("Todo ha ido bien.");  // Si todo va bien manda mensaje al PC con cambio de línea
 
  // Cada vez que apagamos o extraemos la tarjeta SD borramos el fichero anterior
    SD.remove("meteo.txt");
   
  // Comprobamos si el fichero existe
  if (SD.exists("meteo.txt")) {
    Serial.println("meteo.txt ya existe.");
   
  }
  else {
    Serial.println("meteo.txt no existe.");
  }
 
 
 

  lcd.begin(16,2);
  /* Indicamos medidas de la pantalla LCD. (16,2) quiere decir 16 caracteres por dos filas.
   *  Si se colocase una pantalla de 20x4 (20 caracteres por 4 filas) se tendría que escribir
   *  la sentencia lcd.begin(20,4);
   */
//Se inicia el sensor y se hace una lectura inicial
SensorStart();
}

void loop() {   //Este es el ciclo que repite el microcontrolador una y otra vez (LOOP)
      
       int uvLevel = averageAnalogRead(UVOUT);
       int refLevel = averageAnalogRead(REF_3V3);
       // Usamos 3.3 voltios para aumentar la precisión de la medida
       float outputVoltage = 3.3 / refLevel * uvLevel;  
       float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
       int err; //Definimos una variable de tipo entera llamada err
       float temp, hum; // Lo propio pero tipo float para la temperatura y humedad
       if((err = dht11.read(hum, temp)) == 0)    // Si devuelve 0 es que ha leido bien
          {
             lcd.clear();             // Borramos nuestra pantalla LCD
             lcd.setCursor(0,0);      // Posicionamos el cursor en el carácter 0 de la primera línea
             lcd.print("Temp.: ");    // Imprimimos el texto para la temperatura
             lcd.print(temp);         // Le pasamos a la pantalla la temperatura del sensor DHT11.
             lcd.print(" C");         // Indicamos la escala centígrada en pantalla.
             lcd.setCursor(0,1);      // Carácter 0 de la segunda línea donde escribiremos el valor de
                                      // la humedad
             lcd.print("Hdad: ");     // Colocamos el texto humedad en la pantalla LCD
             lcd.print(hum);          // Pasamos el valor del sensor DHT11
             lcd.print(" %");         // Indicamos en pantalla la escala en tanto por ciento
             delay(2000);
             lcd.clear();             // Borramos nuestra pantalla LCD
             lcd.setCursor(0,0);      // Posicionamos el cursor en el carácter 0 de la primera línea
             lcd.print("UV (mW/cm^2): ");    // Imprimimos el texto para la temperatura
             lcd.setCursor(0,1);      // Carácter 0 de la segunda línea donde escribiremos el valor de
                                      // la radiación uv
             lcd.print(uvIntensity);          // Pasamos el valor del sensor ML8511
             
          }
       else     // Si no devuelve el valor 0 es que algo, lo que sea, ha ido mal y paso mensaje al PC
          {
             Serial.println(); // Imprimo un cambio de línea
             Serial.print("Error Num :"); // Imprimo el tipo de error
             Serial.print(err); // Paso el valor a través del puerto serie
             Serial.println(); // Cambio de línea
          }
       delay(2000);
/*       
 *        La sentencia delay(2000) espera dos segundos (2000 milisegundos) y vuelve a hacer una lectura de
 *        parámetros meteorológicos. Posteriormente, se podrán modificar/cambiar estos valores a fin de hacer
 *        una lectura más adecuada a la necesidad real. Por ejemplo cada 5 minutos.
 */


ReadSensor();  //Ejecuta la lectura de sensores.
  
  float pRocio = pow((hum / 100), 0.125)*(112+0.9* temp)+0.1*temp - 112; // Calcula la temperatura de rocío
  /*El punto de rocío es un parámetro que nos dice a qué temperatura se producen los bancos de niebla. Es vital en
   * navegación marítima y aeronaútica. Puede verse detenidamente su cálculo en la página web: 
   * https://es.wikipedia.org/wiki/Punto_de_roc%C3%ADo
   */
  

lcd.clear();            // Elimina todos los simbolos del LCD
lcd.setCursor(0,0);     // Posiciona la primera letra en fila 0 columna 0     
// Imprimimos en la pantalla LCD los datos
lcd.print("Hrel.: ");
lcd.print(Altura);    // Imprimimos la altura relativa en metros
lcd.print(" m");
lcd.setCursor(0,1);
lcd.print("P: ");     // Imprimimos la presión atmosférica en mbar
lcd.print(Presion);
lcd.print(" mbar ");

 myFile = SD.open("meteo.txt", FILE_WRITE);     // Abro el fichero meteo.txt en en la tarjeta sd
 
  // Si el fichero abre bien, escribe dentro de él:
  if (myFile) {
    Serial.print("Escribo en meteo.txt...");      // Primero paso datos a través del puerto serie
    Serial.println("");
    Serial.print(temp);                           // Imprimimos la temperatura en ºC
    Serial.print(" C ");
    Serial.print(hum);                            // Ahora la humedad en %
    Serial.print(" % ");
    Serial.print(Presion);                        // Presión en mbar
    Serial.print(" mbar ");
    Serial.print(pRocio);                         // Punto o temperatura de rocío en ºC
    Serial.print(" C ");
    Serial.print(uvIntensity);
    Serial.println(" mW/cm^2");  // Radiación ultravioleta
    
    // Ahora le toca al salvado de los datos anteriores en la tarjeta SD
    myFile.print(temp);                 // Primero la temperatura en ºC
    myFile.print(" ºC ");
    myFile.print(hum);                  // Humedad relativa en %
    myFile.print(" % ");
    myFile.print(Presion);              // La presión en mbar
    myFile.print(" mbar ");
    myFile.print(pRocio);               // Punto de rocío (DEW POINT)
    myFile.print(" C "); 
    myFile.print(uvIntensity);
    myFile.println(" mW/cm^2");   
    //este println hace el cambio de carro
    //de esta forma tengo temperatura, humedad y presión en una matriz de 5 x n
    //que me permite abrirla desde excel  o cualquier otro sistema de hacer
    //representaciones gráficas
    
    myFile.close();                     // cierra el archivo al acabar
    Serial.println("Hecho.");            // Manda mensaje confirmándolo
  } else {
    // si el fichero no se abre indica un error
    Serial.println("error abriendo meteo.txt");
  }
 
delay(1000);
//Espera un segundito y vuelve a leer
}


//Realiza una media aritmética de un buen número de medidas

int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
  return(runningValue);  
}



//Hacemos una función map en coma flotante
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





void SensorStart() {
//Secuencia de inicio del sensor
if (pressure.begin())
Serial.println("BMP180 Ok");
else
{
Serial.println("BMP180 falló al inicializarlo?)\n\n");
while (1);
}
//Se inicia la lectura de temperatura
status = pressure.startTemperature();
if (status != 0)  {
delay(status);
//Se lee una temperatura inicial
status = pressure.getTemperature(Temperatura);
if (status != 0)    {
//Se inicia la lectura de presiones
status = pressure.startPressure(3);
if (status != 0)     
{
delay(status);
//Se lee la presión inicial incidente sobre el sensor en la primera ejecución
status = pressure.getPressure(PresionBase, Temperatura);
}
}
}
}
void ReadSensor() {
//En este método se hacen las lecturas de presión y temperatura y se calcula la altura
//Se inicia la lectura de temperatura
status = pressure.startTemperature();
if (status != 0)
{
delay(status);
//Se realiza la lectura de temperatura
status = pressure.getTemperature(Temperatura);
if (status != 0)
{
//Se inicia la lectura de presión
status = pressure.startPressure(3);
if (status != 0)
{
delay(status);
//Se lleva a cabo la lectura de presión
//considerando la temperatura que afecta la lectura del sensor
status = pressure.getPressure(Presion, Temperatura);
if (status != 0)
{
//Cálculo de la altura en base a la presión leída en el Setup
Altura = pressure.altitude(Presion, PresionBase);
}
else Serial.println("Error en la lectura de presion\n");
}
else Serial.println("Error iniciando la lectura de presion\n");
}
else Serial.println("Error en la lectura de temperatura\n");
}
else Serial.println("Error iniciando la lectura de temperatura\n");
}
