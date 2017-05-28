/* GeoCaja v1.0.1
 * 
 * 8 filas de caracteres, de 8 pix cada una en alto, con 21 caracterres
 * 
 * v0.1
 * - HW Pruebas de GPS
 * - HW Pruebas de display
 * v0.2
 * - SW Menú basico habilitado
 * - HW Potenciometro conectado y asociado
 * - SW Optimizacion de memoria RAM
 * v0.3
 * - SW Activado modo geoCaja
 * - SW Activado modo temporizdo
 * v0.5
 * - SW Creada seccion superior en oled (Banner)
 * - HW Probadas luces
 * v0.6
 * - SW Creado menu de salida
 * v0.7
 * - SW Correcion de errores
 * - SW Agregadas varias imágenes y logos
 * - SW Optimizacion de memoria RAM
 * v0.8
 * - SW Arreglados varios bugs
 * - HW Puerta activada
 * v0.9
 * - SW Optimizacion de memoria de programa
 * v1.0
 * - SW Tolerancia ajustable
 * - SW - HW Prueba de funcionalidad completa exitosa
 * v1.2
 * - SW Solucionado bug que permitía abrir la caja si se reiniciaba varias veces
 * v1.4
 * - SW Agregado buzzer
 * - Solucionado problema que impedía que la caja cerrara
 * v1.6
 * - HW Agregado sellado de silicona interno
 * - HW Agregado sello final en la batería de la caja
 * v1.8
 * - SW Agregado reloj
 * v2.0
 * - Convertido a 
 * TO-DO
 * - Agregar brújula i2c
 * - Usar define para direcciones de memoria de la eeprom
 * - Reducir tamaño de memoria de programa
 * - Simplificar las instancias de menú a una unica funcion que acepte F() (sin caer en gold plating)
 * Bugs conocidos:
 * - Al seleccionar la primera opcion del menu, despues del primer inicio, la caja se reinicia pero vuelve a funcionar de inmediato
 * - Después de un rato mirando los datos GPS, la caja podría reiniciarse
 * - Si se corrompe el bit 0, la caja podria reiniciarse completamente
 * Modos de operacion : 
 * 0 - Desbloqueado
 * 1 - Bloqueado
 * 2 - Bloqueo Temporizado
 * 
 * Las licencias de uso de las librerías se encuentran en sus respectivos códigos fuente
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <Wire.h>

#define NOP dummy++
#define gpsSerial Serial5
#define dbgSerial SerialUSB
#define btSerial Serial

//#define btSerial Serial5
#define KEY_HIGH 256
#define VERSION_NOW "TerraCaja v2.0"

unsigned long dummy = 0;

const byte doorSens = 2, beepPin = 3, lightsPin = 6, bot1Pin = 8, servoPin = 9, actPin = 10, servoEnable = 11 ,stepLed = 13,potPin = A1, vBattPin = A2;
const byte lockPos = 0, openPos = 180;

//Imagenes

const byte gpsIcon[]  PROGMEM = {0x20, 0x44, 0xAE, 0x1C, 0x38, 0x15, 0x02, 0x04};
const byte updIcon[]  PROGMEM = {0x10, 0x7A, 0x91, 0xA1, 0x85, 0x89, 0x5E, 0x08};
const byte noBatL[]   PROGMEM = {0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF};
const byte noBatR[]   PROGMEM = {0xFE, 0x02, 0x03, 0x03, 0x03, 0x03, 0x02, 0xFE};
const byte lowBatL[]  PROGMEM = {0xFF, 0x80, 0xB8, 0xB8, 0xB8, 0xB8, 0x80, 0xFF};
const byte lowBatR[]  PROGMEM = {0xFC, 0x02, 0x03, 0x03, 0x03, 0x03, 0x02, 0xFE};
const byte midBatL[]  PROGMEM = {0xFF, 0x80, 0xBB, 0xBB, 0xBB, 0xBB, 0x80, 0xFF};
const byte midBatR[]  PROGMEM = {0xFC, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0xFE};
const byte fullBatL[] PROGMEM = {0xFF, 0x80, 0xBB, 0xBB, 0xBB, 0xBB, 0x80, 0xFF};
const byte fullBatR[] PROGMEM = {0xFE, 0x02, 0xBB, 0xBB, 0xBB, 0xBB, 0x02, 0xFE};

TinyGPSPlus gps;
Adafruit_SSD1306 display(30);
Servo lockTapa;

byte outKey[]= {16,32,64}; //llave por defecto

byte mode = 0;

float tgtLat = 0.0;
float tgtLon = 0.0;
float nowLat = 0.0;
float nowLon = 0.0;

long tgtTime = 0;
unsigned long gpsCharsLast = 0;
bool lGpsChars = false;
unsigned int tol = 10;
bool beeper = true;
void beep(int del, int delAfter)
{
  if(beeper)
  {
    digitalWrite(beepPin,HIGH);
    delay(del);
    digitalWrite(beepPin,LOW);
    delay(delAfter);
  }
}

void EEPROMwrite()
{
	
}

void EEPROMput()
{
	
}

byte EEPROMread()
{
	
}

long EEPROMgetLong()
{
	
}
int EEPROMgetInt()
{
	
}
byte EEPROMgetByte()
{
	
}

void setup() 
{
  pinMode(actPin,OUTPUT);
  pinMode(lightsPin,OUTPUT);
  pinMode(servoEnable,OUTPUT);
  pinMode(bot1Pin,INPUT_PULLUP);
  pinMode(beepPin,OUTPUT);
  digitalWrite(beepPin,LOW);
  digitalWrite(actPin,LOW);
  digitalWrite(lightsPin,LOW);
  digitalWrite(servoEnable,LOW);
  //Inicializar
  /*
   * Mapeo de memoria
   * Los datos vacios se representan con 0
   * 
   * 0x00 Byte de estado
   * 0x01 Byte de ultimo modo
   * 0x02 - 0x05 Latitud objetivo
   * 0x06 - 0x09 Longitud objetivo
   * 0x0A - 0x0D Tiempo objetivo 
   * 0x0E - 0x11 Ultima vez encendido
   * 0x12 - 0x15 llave
   * 0x16 - 0x17 tolerancia en metros
   */
  attachInterrupt(digitalPinToInterrupt(doorSens),ISRdoorLights,CHANGE);
  digitalWrite(lightsPin, digitalRead(doorSens));
  
  gpsSerial.begin(9600);
  dbgSerial.begin(115200);
  btSerial.begin(9600); 
  btSerial.print("AT");
  delay(100);
  bool btError = false;
  if(btSerial.readString() != "OK") btError = true;
  btSerial.print("AT+PIN1231");
  btSerial.print("AT+NAMETerraCaja");
  dbgSerial.println(F("Start"));
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //Iniciar pantalla
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();
  /*
  if(EEPROM.read(0x00) == 0x01) //Estado válido
  {
    mode = EEPROM.read(0x01);
    EEPROM.get(0x02, tgtLat);
    EEPROM.get(0x06, tgtLon);
    EEPROM.get(0x0A, tgtTime);
    EEPROM.get(0x16, tol);
  }
  else //Estado inválido
  {
    for(unsigned int i = 0x0000; i < 1024; i++) //EEPROM.update(i,0x00);
    EEPROM.update(0x00,0x01);
    display.println(F("Datos corruptos"));
    display.display();
  }*/
  digitalWrite(servoEnable,HIGH);
  lockTapa.attach(servoPin);
  mode == 0 ? lockTapa.write(openPos) : lockTapa.write(lockPos);
  // beep(50,50);
  // beep(50,50);
  // beep(100,150);
  delay(750);
  digitalWrite(servoEnable,LOW);
  // protolinea display.println(F("                     "));
  while(nowLat == 0.0 && nowLon == 0.0 && false)
  {
    gpsGet(500);
    newScreen();
    display.setCursor(0,24);
    display.println(F("Buscando satelites"));
    if(gps.satellites.value() >= 4) display.println(F("Fijando posicion"));
    nowLat = gps.location.lat();
    nowLon = gps.location.lng();
    display.display();
  }
  setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  adjustTime(-3 * SECS_PER_HOUR);
  nowLat = gps.location.lat();
  nowLon = gps.location.lng();
  // Si esta desbloqueado, menu
  if(mode == 0)
  {
    lockTapa.write(openPos);
    while(mode == 0)
    {
      byte menuSelect = map(analogRead(potPin),0,1024,0,6);
      if(menuSelect == 6) menuSelect--;
      //Dibujar puntero
      newScreen();
      printBaseMenu();
      display.setCursor(7,16+(8*menuSelect));
      display.print(F(">"));
      display.display();
      if(!digitalRead(bot1Pin))
      {
        if(menuSelect == 0) //geomodo OK
        {
          byte choiceSelect = 0;
          gpsGet(1000);
          nowLat = gps.location.lat();
          nowLon = gps.location.lng();
          while(digitalRead(bot1Pin))
          {    
            choiceSelect = map(1024 - analogRead(potPin),0,1024,0,2);
            newScreen();
            display.println(F("La caja se bloqueara")); 
            display.println(F("  en las siguientes")); 
            display.println(F("     coordenadas"));
            display.print(F("lon=")); display.println(nowLon,8);
            display.print(F("lat=")); display.println(nowLat,8);
            display.println(F("   Desea continuar?"));
            display.println(F(" [ ] No       [ ] Si "));
            if(choiceSelect == 2) choiceSelect--;
            display.setCursor(12+choiceSelect*78,56);
            display.print(F("X"));
            display.display();
            delay(100);
          }
          if(choiceSelect)
          {
            tgtLat = nowLat;
            tgtLon = nowLon;
            mode = 1;
          }
        }
        else if(menuSelect == 1) //temporizado OK
        {
          newScreen();
          display.println(F("  MODO  TEMPORIZADO"));
          display.println(F(" Pasar entre medidas"));
          display.println(F("con el boton, ajustar"));
          display.println(F("unidades con el dial"));
          display.display();
          delay(1500);
          int hrss = 0, mins = 0, secs = 0;
          while(digitalRead(bot1Pin)) //Seleccion horas
          {
            newScreen();
            hrss = map(1024 - analogRead(potPin),0,1024,0,96);
            display.println(F("  MODO  TEMPORIZADO"));
            display.print(F("Horas    = "));
            display.setCursor(80, 16);
            display.println(hrss);
            display.println(F("Minutos  = "));
            display.display();
            delay(100);
          }
          while(!digitalRead(bot1Pin)) delay(15);
          while(digitalRead(bot1Pin)) //Seleccion minutos
          {
            newScreen();
            mins = map(1024 - analogRead(potPin),0,1024,0,61);
            display.println(F("  MODO  TEMPORIZADO"));
            display.print(F("Horas    = "));
            display.setCursor(80, 16);
            display.println(hrss);
            display.print(F("Minutos  = "));
            display.setCursor(80, 24);
            display.println(mins);
            display.display();
            delay(100);
          }
          while(!digitalRead(bot1Pin)) delay(15);
          byte choiceSelect = 0;
          while(digitalRead(bot1Pin)) //Confirmar
          {
            newScreen();
            choiceSelect = map(1024 - analogRead(potPin),0,1024,0,2);
            display.println(F("  MODO  TEMPORIZADO"));
            display.print(F("Horas    = "));
            display.setCursor(80,16);
            display.println(hrss);
            display.print(F("Minutos  = "));
            display.setCursor(80, 24);
            display.println(mins);
            display.println(F(" Desea bloquear con"));
            display.println(F("  estos parametros?"));
            display.println(F(" [ ] No       [ ] Si"));
            if(choiceSelect == 2) choiceSelect--;
            display.setCursor(12+choiceSelect*78,48);
            display.print(F("X"));
            display.display();
            delay(100);
          }
          if(choiceSelect)
          {
            tgtTime = now() + (long)secs + (long)mins*60L + (long)hrss*3600L;
            mode = 2;
          }
        }
		else if(menuSelect == 2) //conexion a terraOracle
        {
          btSerial.println("ready");
		  bool sucConnect = false;
          while(!sucConnect)
          {
			newScreen();
            display.println(F("Conectando a TerraOracle"));
            display.display();
            delay(100);
          }
        }
        else if(menuSelect == 3) //DataGPS, OK
        {
          while(1-digitalRead(bot1Pin)) delay(10);  
          while(digitalRead(bot1Pin))
          {
            gpsGet(1000);
            nowLat = gps.location.lat();
            nowLon = gps.location.lng();
            newScreen();
            display.print(F("lon="));
            display.println(nowLon,8);
            display.print(F("lat="));
            display.println(nowLat,8);
            display.print(F("chars="));
            display.println(gps.charsProcessed());
            display.print(F("alt="));
            display.println(gps.altitude.meters());
            display.print(F("hdop="));
            display.println(gps.hdop.value());
            delay(10);
            display.display();
          }
          while(1-digitalRead(bot1Pin)) delay(10);  
        }
        else if(menuSelect == 4) //reporte, OK
        {
          while(1-digitalRead(bot1Pin)) NOP;  
          while(digitalRead(bot1Pin))
          {
            int vBatt = analogRead(vBattPin); // de 512 a 700
            delay(10);
            newScreen();
            display.println(F(" DATOS DEL SISTEMA"));
            display.println(F(VERSION_NOW));
            display.print(F("vBatt = "));
            display.println(2*((float)vBatt*3.3)/1024);
            display.print(F("vStat = "));
            if(vBatt< 580) display.println(F("low"));
            else if (vBatt < 650) display.println(F("mid"));
            else if (vBatt > 650) display.println(F("high"));
            display.print(F("vRaw = "));
            display.println(vBatt);
            display.print(F("tapaStatus = "));
            display.println(digitalRead(doorSens));
            display.print(F("t = "));
            display.println(now());
            display.display();
          }
          while(1-digitalRead(bot1Pin)) delay(10); 
        }
        else if(menuSelect == 5)//Menu especial
        {
          newScreen();
          while(!digitalRead(bot1Pin)) delay(15);
          while(digitalRead(bot1Pin)) //Seleccion tolerancia
          {
            newScreen();
            tol = map(1024 - analogRead(potPin),0,1024,10,100);
            display.println(F("   CONFIGURACION"));
            display.print(F("Tolerancia = "));
            display.setCursor(80, 16);
            display.print(tol);
            display.println("m");
            display.display();
            delay(100);
          }
        }
      }
      delay(100);
    }
    while(digitalRead(doorSens))
    {
      newScreen();
      display.setTextSize(2);
      display.println(F("Cierre la"));
      display.println(F("  tapa"));
      display.display();
      beep(50,50);
      beep(50,1000);
    }
    newScreen();
    display.setTextSize(2);
    display.println(F("Cerrando"));
    display.println(F("Tapa"));
    display.display();
    delay(500);
    digitalWrite(servoEnable,HIGH);
    lockTapa.write(lockPos);
    delay(1000);
    digitalWrite(servoEnable,LOW);
    //EEPROM.write(0x00, 0x00);
    //EEPROM.put(0x01, mode);
    //EEPROM.put(0x02, tgtLat);
    //EEPROM.put(0x06, tgtLon);
    //EEPROM.put(0x0A, tgtTime);
    //EEPROM.put(0x16, tol);
    //EEPROM.write(0x00, 0x01);
  }
}

void loop() 
{
  gpsGet(1000);
  nowLat = gps.location.lat();
  nowLon = gps.location.lng();
  if(mode == 1)
  {
    while(gps.satellites.value() < 3)
    {
      gpsGet(500);
      newScreen();
      display.println(F(" TerraCaja bloqueada"));
      display.println(F(" Necesitas Satelites"));
      display.println(F("   GPS  adicionales"));
      display.println(F("  Por favor ubiquese"));
      display.println(F("     en un lugar"));
      display.println(F("  con vista al cielo"));
      display.display();
    }
    unsigned long distanceToTgt = (unsigned long)TinyGPSPlus::distanceBetween(
      nowLat,
      nowLon,
      tgtLat, 
      tgtLon);
    double courseToTgt =  TinyGPSPlus::courseTo(
      nowLat,
      nowLon,
      tgtLat, 
      tgtLon);
    newScreen();
    display.println(F("TerraCaja bloqueada"));
    display.print(F("Distancia  : "));
    display.print(distanceToTgt);
    display.println(F("(m)"));
    display.print(F("Curso al obj: "));
    display.println(TinyGPSPlus::cardinal(courseToTgt));
    display.print(F("Tolerancia  : "));
    display.println(tol);
    //128x64
    display.display();
    while(distanceToTgt < tol)
    {
      gpsGet(1000);
      newScreen();
      nowLat = gps.location.lat();
      nowLon = gps.location.lng();
      distanceToTgt =  (unsigned long)TinyGPSPlus::distanceBetween(
      nowLat,
      nowLon,
      tgtLat, 
      tgtLon);
      courseToTgt =  TinyGPSPlus::courseTo(
      nowLat,
      nowLon,
      tgtLat, 
      tgtLon);
      if(distanceToTgt > tol) break;
      display.println(F("TerraCaja en posicion"));
      display.println(F(" Pulse el boton para"));
      display.println(F("     desbloquear"));
      display.print(F("Distancia: "));
      display.print(distanceToTgt);
      display.println(F("(m)"));
      display.print(F("Curso al obj: "));
      display.println(TinyGPSPlus::cardinal(courseToTgt));
      display.print(F("Tolerancia  : "));
      display.println(tol);
      display.display();
      if(!digitalRead(bot1Pin))
      {
        newScreen();
        display.println(F(" La caja se va a"));
        display.println(F("     reiniciar"));
        display.display();
        //EEPROM.write(0x00, 0x00);
        //EEPROM.put(0x01, 0x00);
        //EEPROM.put(0x02, 0x00000000UL);
        //EEPROM.put(0x06, 0x00000000UL);
        //EEPROM.put(0x0A, 0x00000000UL);
        //EEPROM.put(0x0E, 0x00000000UL);
        //EEPROM.put(0x16, tol);
        //EEPROM.write(0x00, 0x01);
        digitalWrite(servoEnable,HIGH);
        lockTapa.write(openPos);
        delay(1000);
        digitalWrite(servoEnable,LOW);
        unsigned long obTime = millis();
        while(obTime + 20000UL > millis()) NOP;
        NOP;
      }
    }
  }
  else if(mode == 2)
  {
    newScreen();
    display.println(F("  Modo temporizado"));
    display.println(F("Tiempo para apertura"));
    display.println(tgtTime-now());
    if(tgtTime <= now())
    {
      newScreen();
      display.println(F("   TIEMPO CUMPLIDO"));
      display.println(F("    Puedes abrir"));
      display.println(F("      la caja"));
      display.display();
      unsigned long obTime = millis();
      digitalWrite(servoEnable,HIGH);
      lockTapa.write(openPos);
      delay(1000);
      digitalWrite(servoEnable,LOW);
      //EEPROM.write(0x00, 0x00);
      //EEPROM.put(0x01, 0x00);
      //EEPROM.put(0x02, 0x00000000UL);
      //EEPROM.put(0x06, 0x00000000UL);
      //EEPROM.put(0x0A, 0x00000000UL);
      //EEPROM.put(0x0E, now());
      //EEPROM.put(0x16, tol);
      //EEPROM.write(0x00, 0x01);
      while(obTime + 20000UL > millis()) NOP;
    }
    display.display();
  }
  if(!digitalRead(bot1Pin)) //menu de salida
  {
    unsigned long exitStart = millis();
    unsigned long exitEnd = 0;
    while(!digitalRead(bot1Pin)) delay(10);
    exitEnd = millis();
    if(exitStart+5000 <= exitEnd)
    {
      //Menu de salida
      newScreen();
      display.println(F("   SALIDA ESPECIAL"));
      display.println(F("Al ingresar la clave"));
      display.println(F("correcta, se perderan"));
      display.println(F("   todos los datos"));
      delay(2000);
      byte key0 = 0;
      byte key1 = 0;
      byte key2 = 0;
      while(digitalRead(bot1Pin))
      {
        newScreen();
        display.println(F(" SALIDA ESPECIAL"));
        key0 = map(1024 - analogRead(potPin),0,1024,0,KEY_HIGH); //Ultimo byte no cuenta
        display.print(F("Llave 1  = "));
        display.setCursor(80, 16);
        display.println(key0);
        display.println(F("Llave 2  = "));
        display.println(F("Llave 3  = "));
        display.display();
        delay(100);
      }
      while(!digitalRead(bot1Pin)) delay(15);
      while(digitalRead(bot1Pin))
      {
        newScreen();
        key1 = map(1024 - analogRead(potPin),0,1024,0,KEY_HIGH);
        display.println(F(" SALIDA ESPECIAL"));
        display.print(F("Llave 1  = "));
        display.setCursor(80, 16);
        display.println(key0);
        display.print(F("Llave 2  = "));
        display.setCursor(80, 24);
        display.println(key1);
        display.println(F("Llave 3  = "));
        display.display();
        delay(100);
      }
      while(!digitalRead(bot1Pin)) delay(15);
      while(digitalRead(bot1Pin))
      {
        newScreen();
        key2 = map(1024 - analogRead(potPin),0,1024,0,KEY_HIGH);
        display.println(F(" SALIDA ESPECIAL"));
        display.print(F("Llave 1  = "));
        display.setCursor(80, 16);
        display.println(key0);
        display.print(F("Llave 2  = "));
        display.setCursor(80, 24);
        display.println(key1);
        display.print(F("Llave 3  = "));
        display.setCursor(80, 32);
        display.println(key2);
        display.display();
        delay(100);
      }
      while(!digitalRead(bot1Pin)) delay(15);
      while(digitalRead(bot1Pin)) //Confirmar
      { 
        newScreen();
        if(outKey[0] == key0 && outKey[1] == key1 && outKey[2] == key2)
        {
          /*
           * Mapeo de memoria
           * Los datos vacios se representan con 0
           * 
           * 0x00 Byte de estado
           * 0x01 Byte de ultimo modo
           * 0x02 - 0x05 Latitud objetivo
           * 0x06 - 0x09 Longitud objetivo
           * 0x0A - 0x0D Tiempo objetivo 
           * 0x0E - 0x11 Ultima vez encendido
           * 0x12 - 0x15 llave
           * 0x16 - 0x17 tolerancia en metros
           */
          //EEPROM.write(0x00, 0x00);
          //EEPROM.put(0x01, 0x00);
          //EEPROM.put(0x02, 0x00000000UL);
          //EEPROM.put(0x06, 0x00000000UL);
          //EEPROM.put(0x0A, 0x00000000UL);
          //EEPROM.put(0x0E, 0x00000000UL);
          //EEPROM.put(0x16, tol);
          //EEPROM.write(0x00, 0x01);
          display.println(F("   SALIDA ESPECIAL"));
          display.println(F("    Clave aceptada"));
          display.println(F("   Reinicie la caja"));
          display.println(F("   para desbloquear"));
          display.display();
          unsigned long obTime = millis();
          while(obTime + 20000UL > millis()) NOP;
        }
        else
        {
          display.println(F("  Clave incorrecta"));
          display.display();
        }
      }
    }
  }
}

static void gpsGet(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void printBaseMenu()
{
  display.setCursor(0,8);
  display.println(F("  MODO DESBLOQUEADO"));
  display.println(F("[ ] Modo uniPosicion"));
  display.println(F("[ ] Modo temporizado"));
  display.println(F("[ ] Modo TerraOracle"));
  display.println(F("[ ] Datos GPS"));
  display.println(F("[ ] Estado actual"));
  display.println(F("[ ] Ajuste tolerancia"));
}

void newScreen()
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.drawBitmap(0, 0,  gpsIcon, 8, 8, 1);
  display.print(F("  = "));
  display.print(gps.satellites.value());
  display.print(F(" sat "));
//  display.print(hour());
//  display.print(F(":"));
//  display.print(minute());
  if(gpsCharsLast + 100 < gps.charsProcessed())
  {
    gpsCharsLast = gps.charsProcessed();
    lGpsChars = !lGpsChars;
  }
  if(lGpsChars) display.drawBitmap(90, 0,  updIcon, 8, 8, 1);
  int vBatt = analogRead(vBattPin);
  if (vBatt < 550)
  {
    display.drawBitmap(100, 0, noBatL, 8, 8, 1);
    display.drawBitmap(108, 0, noBatR, 8, 8, 1);
  }
  else if (vBatt < 600)
  {
    display.drawBitmap(100, 0, lowBatL, 8, 8, 1);
    display.drawBitmap(108, 0, lowBatR, 8, 8, 1);
  }
  else if (vBatt <= 650)
  {
    display.drawBitmap(100, 0, midBatL, 8, 8, 1);
    display.drawBitmap(108, 0, midBatR, 8, 8, 1);
  }
  else if (vBatt > 650)
  {
    display.drawBitmap(100, 0, fullBatL, 8, 8, 1);
    display.drawBitmap(108, 0, fullBatR, 8, 8, 1);
  }
  display.setCursor(0,8);
}

void ISRdoorLights()
{
  digitalWrite(lightsPin, digitalRead(doorSens));
  digitalWrite(actPin, digitalRead(doorSens)) ;
}
