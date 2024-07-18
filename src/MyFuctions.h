#ifndef _MYFUCTIONS_H
#define _MYFUCTIONS_H

#include <Arduino.h>
#include <SmartGreenhouse.h>
#include <FirebaseESP32.h>  // Para la comunicación con Firebase
#include <Wire.h>           // Para la comunicación I2C con los sensores
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"         // Para el sensor de temperatura y humedad DHT
#include <BluetoothSerial.h>
#include <NTPClient.h>
#include <Preferences.h>
//#include <Adafruit_ADS1015.h> // Para los sensores analógicos

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

//EEPROM
//Preferences preferences;
Preferences espEeprom;

#define LED_ESP 2
/*#define CH1 25
#define CH2 13
#define CH3 12
#define CH4 14
#define CH5 27
#define CH6 26
#define CH7 2
#define CH8 4*/

//MAQUINA DE ESTADOS
#define STATE_UID       1
#define STATE_NAME      2
#define STATE_PASS      3
#define STATE_WIFINIT   4

// Definiciones de pines
#define DHT_PIN 33        // Pin del sensor DHT
#define LM35_PIN 32        // Pin Analogico 0 "A0" para sensor LM35
#define BUTTON_PIN 2     // Pin del pulsador
#define NUM_ANALOG_SENSORS 6 // Número de sensores analógicos
#define NUM_DIGITAL_OUTPUTS 8 // Número de salidas digitales


/* 1. Define the API Key */
#define API_KEY "AIzaSyCu7jNCoSoW3hLku1biL2RQ0tIvp0jzFsA"

/* 2. Define the RTDB URL */
#define DATABASE_URL "ejemploesp8266-533d5-default-rtdb.firebaseio.com"

/* 3. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "daferd19@gmail.com"
#define USER_PASSWORD "123456"

//uint8_t channelAr[]={CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8};

//VARIABLES PARA EL CONTROL DE BLUETHOOT
BluetoothSerial SerialBT;   //Declaramos un objeto de tipo BluetoothSerial.
byte STATE = STATE_UID;     //Variable para control de los estados en la configuración Bluethoot
bool bandBT = false;        //Bandera para controlar la recepcion de datos serial



//VARIABLES PARA LA CONFIGURACION DE LA RED WIFI
volatile bool modoConfigBT = false; 
bool modoConfigFB = true;
bool modoConfigUID = true;
bool bandUID = true;
String SSID="REDGOINN";  //"JuanD",   REDGOINN,         Inverna        Moto_AH    
String PASS="900791927G";  //"Cata1979",  900791927G   wiracocha       12345678
String userID="";
String UID="mSeD55cmDSeuSRx9T8yceAehvpA2";  //2k147bi5U8WDFrt3OWHOc0KMg7D3

//Variables para el control del parpadeo del led
uint32_t tiempoActual = 0;
uint32_t tiempoComp = 0;  
//Variables para el control del parpadeo del led
uint32_t tiempoActualAux = 0;
uint32_t tiempoCompAux = 0;  

// Configuración del sensor DHT
DHT dht(DHT_PIN, DHT22);

float tempLm35 = 1; // Variable para almacenar la temperatura en grados Celsius



String USER_PATH = "/users/2k147bi5U8WDFrt3OWHOc0KMg7D3";

//Variable y Estructuras para el control del Timer
/*struct ChannelServer {
  uint8_t numberChannel;
  uint8_t state;
};
ChannelServer channel;*/
//Variables para configurar la hora mediante Wifi
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
ESP32Time rtc(0);

FirebaseAuth auth;
FirebaseConfig config;

// Define Firebase Data object
FirebaseData fbdo,fbdoStreaming;
FirebaseJson jsFb;
FirebaseJsonData jsData;
String jsStr;

SmartGreenhouse garden;

unsigned long sendDataPrevMillis = 0;

int count = 0;

volatile unsigned char contIntentos=0; //Contador de intentos para la conexion Wifi
#define MAX_INTENT 100   //Maximo de intentos para hacer conexion con Wifi

// Variables globales
float temperature = 0.0;
float humidity = 0.0;
bool buttonState = false;
int analogSensorValues[NUM_ANALOG_SENSORS];
bool digitalOutputs[NUM_DIGITAL_OUTPUTS];

String lastCharTimer = "";
String lastCharChannel = "";

struct channel{
  uint8_t pin = 1;
  bool state = false;
};

String userPath = "/users/1111111111"+UID;

#define HOME         1
//VARIABLES PARA EL CONTROL MAQUINA DE ESTADOS PRINCIPAL
unsigned char state = HOME;  //HOME

bool flagTimer = false;
bool flagTimerCh1 = false;
hw_timer_t *timer1 = NULL; // Apuntador a variable de tipo hw_timer_t que luego usaremos en la función de configuración de Arduino.

bool stateChannel1 = false;
bool stateChannel2 = false;
bool stateChannel3 = false;
bool stateChannel4 = false;
bool stateChannel5 = false;
bool stateChannel6 = false;


// Prototipos de funciones
void readSensorsTask(void *pvParameters);
void controlOutputsTask(void *pvParameters);


void testHwm(char * taskName);
bool InitWiFi(String SSID, String PASS);
bool initFirebase(String email, String pass, String path);
void firebaseCallback(StreamData data);
void timeoutCallback(bool timeCallback);
void printMsg(String label,int val);
void ledBlinkMillis(unsigned int velocidad);
void updateTime();
void getInfoTimerChanel();
float readLm35();
void loadEeprom();
void CausaError(void);

void IRAM_ATTR onTimer1(){
  flagTimer = true;
  //printMsg("Timmer ", 1);
}







void readSensorsTask(void *pvParameters) {
  
  while (1) {
    //garden.sensorValue[DHT_PIN]=garden.readDigitalSensor(DHT_PIN);
    //garden.sensorValue[DHT_PIN]=random(25,40);
    //float temperatureDHT = dht.readTemperature();
    
    float temperatureDHT = random(25,40);
    printMsg("Temperatura: ", temperatureDHT);
    Firebase.set(fbdo, userPath + "/sensors/0",temperatureDHT);
    vTaskDelay(pdMS_TO_TICKS(1000));
    //float humidityDHT = dht.readHumidity();
    float humidityDHT = random(25,40);
    Firebase.set(fbdo, userPath +"/sensors/1",humidityDHT);
    testHwm("ReadSensorsTask");
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

void controlOutputsTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {

    //printMsg("HOLq",1);

    //vTaskDelay(pdMS_TO_TICKS(1000));
    //testHwm("ControlOutputsTask"); // Esta función consume 512 bytes aproximadamente
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void testHwm(char * taskName){

    /* 
    Permite conocer la cantidad de Memoria RAM que NO se esta usando por la tarea, es decir cuanta 
    RAM sobra de la cantidad asignada en xTaskCreatePinnedToCore() 
    
    Nota: Se debe tener en cuenta que esta funcion por si misma consume aproximadamente 512 bytes aproximadamente,
    es decir, se debe tener en cuenta al momento de definir la cantidad de memoria RAM 
    */

    static int stack_hwm, stack_hwm_temp;
    stack_hwm_temp = uxTaskGetStackHighWaterMark(nullptr);
    if (!stack_hwm || (stack_hwm_temp < stack_hwm)){
        stack_hwm = stack_hwm_temp;
        Serial.printf("%s has stack hwm %u\n", taskName, stack_hwm);
    }
    
}

//Generales
bool InitWiFi(String SSID, String PASS){  

    bool state=false;    
    boolean bandStateWifi = false;
    unsigned char tamano1= SSID.length();
    char nombreRed[tamano1];

    SSID.toCharArray(nombreRed,tamano1+1);

    unsigned char tamano2= PASS.length();
    char passRed[tamano2];

    PASS.toCharArray(passRed,tamano2+1);

    
    WiFi.mode(WIFI_STA);                      // Estacion 
    //WiFi.mode(WIFI_AP);                     // Punto de Acceso 
    //WiFi.mode(WIFI_MODE_APSTA);             // Ambos 

    WiFi.begin(nombreRed,passRed);                   // Inicializamos el WiFi con nuestras credenciales.
    Serial.print("***** INCIANDO PROCESO DE CONEXIÓN A RED: ");Serial.print(nombreRed);Serial.print(" *****");Serial.println(" ");
    tiempoCompAux = millis();
    bool stateLed = false;
    //while(((WiFi.status() != WL_CONNECTED) && contIntentos == 0) || (contIntentos <= MAX_INTENT)){     // Se quedata en este bucle hasta que el estado del WiFi sea diferente a desconectado.  
    while(WiFi.status() != WL_CONNECTED){     // Se quedata en este bucle hasta que el estado del WiFi sea diferente a desconectado.  
        
        Serial.print(".");
        ledBlinkMillis(200);
         // Si el intervalo ha pasado, vuelve a intentar la conexión
        //tiempoCompAux = millis();
        if (millis() - tiempoCompAux >= 40000) {
          tiempoCompAux = millis();
          Serial.println("Intentando reconectar al WiFi...");
          WiFi.disconnect(); // Desconectar antes de intentar reconectar
          WiFi.begin(nombreRed,passRed);                   // Inicializamos el WiFi con nuestras credenciales.
          //delay(100);
        }
    }

    if(WiFi.status() == WL_CONNECTED){        // Si el estado del WiFi es conectado entra al If
      contIntentos = 0;
      Serial.println();
      Serial.println();
      Serial.println("Conexion exitosa!!!");
      Serial.println("");
      Serial.print("Tu IP es: ");
      Serial.println(WiFi.localIP());

      updateTime();
      Serial.println("**************************************************");Serial.println();

      delay(500);
      state = true;
    }else{
      state = false;
      Serial.println("error WIFI 1");
      //fuente.encenderTodoLuces(255,0,0);
      //fuente.ledBlinkMillis(50);
      contIntentos = 0;
      InitWiFi(SSID,PASS);
    }

    return state;
}

bool initFirebase(String email, String pass, String path){

      bool confirm = false;
    /* Assign the api key (required) */
      config.api_key = API_KEY;

      /* Assign the user sign in credentials */
      auth.user.email = email; 
      auth.user.password = pass; 

      /* Assign the RTDB URL (required) */
      config.database_url = DATABASE_URL;
      // Inicializar Firebase
      /* Assign the callback function for the long running token generation task */
      config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

      Firebase.reconnectWiFi(true);
      Firebase.setDoubleDigits(5);     

      // required for large file data, increase Rx size as needed.
      fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

      // Inicializamos Firebase, mediante el URL y secreto de la base de datos del proyecto en Firebase.
      //Firebase.begin(DB_URL, SECRET_KEY);
      Firebase.begin(&config, &auth);

      fbdo.keepAlive(5, 5, 1);
                  
      if(!Firebase.beginStream(fbdoStreaming, path)){   
          confirm = false;
      }else{
          Firebase.setStreamCallback(fbdoStreaming, firebaseCallback, timeoutCallback);
          confirm = true;
      }

      return confirm;
}

void firebaseCallback(StreamData data){
    String namePath = "";
    Serial.println("Cambios en la base de datos");
    Serial.println(data.dataType());            // Que tipo de dato esta llegando

    if(data.dataType().equals("json")){

        namePath = fbdoStreaming.dataPath();
        lastCharChannel = namePath.substring(17,18);  //18 posicion en el paht del canal
        lastCharTimer = namePath.substring(namePath.length() - 1);

        Serial.println(namePath);
        Serial.print("ultima: ");
        Serial.println(lastCharChannel);


        if(namePath.equals("/channels/channel"+lastCharChannel+"/timers/timer"+lastCharTimer)){
        //if(namePath.equals("/timers/enableTimersLum")){
            getInfoTimerChanel();
        }
    }

    if(data.dataType().equals("boolean")){
        String ruta = "Ch1Timer"+lastCharTimer+"State";
        boolean valorBool = fbdoStreaming.boolData();
        namePath = fbdoStreaming.dataPath();
        
        lastCharChannel = namePath.substring(17,18);  ////se recupera el texto correspondiente al numero del timer , ejemplo: "2", el valor 30 corresponde a la posición del numero del timer en el PATH
        //Serial.println("/timers/timersLuminarias/timer"+lastCharTimer);
        lastCharTimer = namePath.substring(31,32);

        Serial.print("The change is in: "); Serial.println(namePath);
        Serial.print("Ultima letra: "); Serial.println(lastCharTimer);
        
        //REVISAR SI ES NECESARIO 03/04/2024
        if(namePath.equals("/channels/channel"+lastCharChannel+"/state")){  //Con la ultima letra del namePath se define cual fue el canal que se modifico
            //Firebase.get(fbdo,"users");
            
            printMsg("Modificado Canal",lastCharChannel.toInt());
            garden.channel.state=valorBool;
            garden.channel.numberChannel=garden.ch[lastCharChannel.toInt()-1];
            //garden.channel.numberChannel=CH1;
            String nameEeprom = "Ch"+lastCharChannel+"State";
            //printMsg(nameEeprom, 123);
            garden.enableChannel(garden.channel);
            espEeprom.putBool(nameEeprom.c_str(),valorBool);
            //digitalWrite(LED_ESP,!valorBool);


            
            switch (lastCharChannel.toInt()){  //Se desactivan los eventos relacionados al canal, por si hubiese alguno activado de manera que se deje trabajar en modo manual (Con los botones)
                case 1:{
                    //enableCh1flag = true;
                    garden.eventsChannel1[0].state = false;
                    espEeprom.putBool("Ch1Timer1State",false);
                    garden.eventsChannel1[1].state = false;
                    espEeprom.putBool("Ch1Timer2State",false);
                    garden.eventsChannel1[2].state = false;
                    espEeprom.putBool("Ch1Timer3State",false);
                    garden.eventsChannel1[3].state = false;
                    espEeprom.putBool("Ch1Timer4State",false);
                    garden.eventsChannel1[4].state = false;
                    espEeprom.putBool("Ch1Timer5State",false);
                    garden.eventsChannel1[5].state = false;
                    espEeprom.putBool("Ch1Timer6State",false);
                break;}               
                case 2:{
                    //enableCh2flag = true;
                    garden.eventsChannel2[0].state = false;
                    espEeprom.putBool("Ch2Timer1State",false);
                    garden.eventsChannel2[1].state = false;
                    espEeprom.putBool("Ch2Timer2State",false);
                    garden.eventsChannel2[2].state = false;
                    espEeprom.putBool("Ch2Timer3State",false);
                    garden.eventsChannel2[3].state = false;
                    espEeprom.putBool("Ch2Timer4State",false);
                    garden.eventsChannel2[4].state = false;
                    espEeprom.putBool("Ch2Timer5State",false);
                    garden.eventsChannel2[5].state = false;
                    espEeprom.putBool("Ch2Timer6State",false);
                break;}
                case 3:{
                    //enableCh3flag = true;
                    garden.eventsChannel3[0].state = false;
                    espEeprom.putBool("Ch3Timer1State",false);
                    garden.eventsChannel3[1].state = false;
                    espEeprom.putBool("Ch3Timer2State",false);
                    garden.eventsChannel3[2].state = false;
                    espEeprom.putBool("Ch3Timer3State",false);
                    garden.eventsChannel3[3].state = false;
                    espEeprom.putBool("Ch3Timer4State",false);
                    garden.eventsChannel3[4].state = false;
                    espEeprom.putBool("Ch3Timer5State",false);
                    garden.eventsChannel3[5].state = false;
                    espEeprom.putBool("Ch3Timer6State",false);
                break;}
                case 4:{
                    //enableCh4flag = true;
                    garden.eventsChannel4[0].state = false;
                    espEeprom.putBool("Ch4Timer1State",false);
                    garden.eventsChannel4[1].state = false;
                    espEeprom.putBool("Ch4Timer2State",false);
                    garden.eventsChannel4[2].state = false;
                    espEeprom.putBool("Ch4Timer3State",false);
                    garden.eventsChannel4[3].state = false;
                    espEeprom.putBool("Ch4Timer4State",false);
                    garden.eventsChannel4[4].state = false;
                    espEeprom.putBool("Ch4Timer5State",false);
                    garden.eventsChannel4[5].state = false;
                    espEeprom.putBool("Ch4Timer6State",false);
                break;}

                default:
                  break;
            }
            
            printMsg("stateChannel"+lastCharChannel+": ", garden.channel.state);
        }

        
        if(namePath.equals("/channels/channel"+lastCharChannel+"/timers/timer"+lastCharTimer+"/state")){  //Se vigila si se activa o se desactiva e
            switch (lastCharChannel.toInt()){
                case 1:{
                    garden.eventsChannel1[lastCharTimer.toInt()-1].state = valorBool;
                    ruta = "Ch1Timer"+lastCharTimer+"State";
                    espEeprom.putBool(ruta.c_str(),valorBool);
                  break;}
              case 2:{
                    garden.eventsChannel2[lastCharTimer.toInt()-1].state = valorBool;
                    ruta = "Ch2Timer"+lastCharTimer+"State";
                    espEeprom.putBool(ruta.c_str(),valorBool);
                  break;}
            
              default:
                break;
            }
            printMsg("stateTimer"+lastCharTimer+": ", valorBool);
        }
        
        delay(2000);
    }

}

void getInfoTimerChanel(){

    Firebase.get(fbdoStreaming, "/users/"+UID+"/channels/channel"+lastCharChannel+"/timers/timer"+lastCharTimer);
    jsStr = fbdoStreaming.jsonString();
    jsFb.setJsonData(jsStr);

    Serial.println(jsStr);

    String ruta = "Ch1Timer" + lastCharTimer + "State"; //Se inicializa con una ruta cualquiera

    switch (lastCharChannel.toInt()){
        case 1:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel1[lastCharTimer.toInt()-1].state = jsData.boolValue;
            Serial.print("State: "); Serial.println(jsData.boolValue); 
            ruta = "Ch1Timer" + lastCharTimer + "State";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);
            
            jsFb.get(jsData,"/action");
            Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].action = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Action";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/sun");
            Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/mon");
            Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/tues");
            Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/wend");
            Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/fri");
            Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/satu");
            Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            ruta = "Ch1Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/hour");
            Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            ruta = "Ch1Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            jsFb.get(jsData,"/min");
            Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel1[lastCharTimer.toInt()-1].min = jsData.intValue; 
            ruta = "Ch1Timer" + lastCharTimer + "Min";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel1) / sizeof(garden.eventsChannel1[0]);
            garden.sortEventsByTime(garden.eventsChannel1, numEvents);
            garden.printEventTimes(garden.eventsChannel1, numEvents);
        break;}
        case 2:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel2[lastCharTimer.toInt()-1].state = jsData.boolValue;
            Serial.print("State: ");Serial.println(jsData.boolValue); 
            ruta = "Ch2Timer" + lastCharTimer + "State";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/action");
            Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].action = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Action";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/sun");
            Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/mon");
            Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/tues");
            Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/wend");
            Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/fri");
            Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/satu");
            Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            ruta = "Ch2Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/hour");
            Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            ruta = "Ch2Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            jsFb.get(jsData,"/min");
            Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].min = jsData.intValue; 
            ruta = "Ch2Timer" + lastCharTimer + "Min";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel2) / sizeof(garden.eventsChannel2[0]);
            garden.sortEventsByTime(garden.eventsChannel2, numEvents);
            garden.printEventTimes(garden.eventsChannel2, numEvents);
        break;}
        case 3:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel3[lastCharTimer.toInt()-1].state = jsData.boolValue;
            Serial.print("State: ");Serial.println(jsData.boolValue); 
            ruta = "Ch3Timer" + lastCharTimer + "State";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/action");
            Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].action = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Action";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/sun");
            Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/mon");
            Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/tues");
            Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/wend");
            Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/fri");
            Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/satu");
            Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            ruta = "Ch3Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/hour");
            Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            ruta = "Ch3Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            jsFb.get(jsData,"/min");
            Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].min = jsData.intValue; 
            ruta = "Ch3Timer" + lastCharTimer + "Min";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel3) / sizeof(garden.eventsChannel3[0]);
            garden.sortEventsByTime(garden.eventsChannel3, numEvents);
            garden.printEventTimes(garden.eventsChannel3, numEvents);
        break;}
        case 4:{
            jsFb.get(jsData,"/state");
            Serial.print("State: ");Serial.println(jsData.boolValue); 
            garden.eventsChannel4[lastCharTimer.toInt()-1].state = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "State";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/action");
            Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].action = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Action";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/sun");
            Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/mon");
            Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/tues");
            Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/wend");
            Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/fri");
            Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/satu");
            Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            ruta = "Ch4Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(ruta.c_str(),jsData.boolValue);

            jsFb.get(jsData,"/hour");
            Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            ruta = "Ch4Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            jsFb.get(jsData,"/min");
            Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].min = jsData.intValue; 
            ruta = "Ch4Timer" + lastCharTimer + "Min";
            espEeprom.putInt(ruta.c_str(),jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel4) / sizeof(garden.eventsChannel4[0]);
            garden.sortEventsByTime(garden.eventsChannel4, numEvents);
            garden.printEventTimes(garden.eventsChannel4, numEvents);
        break;}
        default:
          break;
    }
    //stateDefine(events);
}

void printMsg(String label,int val){
  Serial.print(label); Serial.println(val);
}

void timeoutCallback(bool timeCallback){
  if (timeCallback)
    Serial.println("stream timed out, resuming...\n");

  if (!fbdo.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", fbdo.httpCode(), fbdo.errorReason().c_str());
}

void ledBlinkMillis(unsigned int velocidad){
  tiempoActual = millis();
  if (tiempoActual - tiempoComp >= velocidad){
      //contIntentos++;
      tiempoComp = tiempoActual;
      digitalWrite(LED_ESP, digitalRead(LED_ESP) ^ 1); // invierte estado del LED
  }
}

void updateTime(){
    timeClient.begin();
    timeClient.setTimeOffset(-18000);
    timeClient.update();
    delay(200);
    rtc.setTime(timeClient.getEpochTime());
         //    seg,min,hora,dia,mes,año
    //rtc.setTime(0,58,23,29,1,2024);
    Serial.print("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());
}

float readLm35(){

  int valorSensor = 0;
  float temp=0;
  float tempAux=0;
  
  for (size_t i = 0; i < 5; i++){
    valorSensor =  analogRead(LM35_PIN);       // Lee el valor analógico del sensor LM35
    float tempAux = float(valorSensor) * (4095.0 / 3330.0);  // Convierte el valor analógico a temperatura en grados Celsius
    temp = temp + ((tempAux/10)+3.3);
  }

  temp = temp/5;

  //valorSensor = valorSensor / 5;
  //valorSensor = analogRead(LM35_PIN);       // Lee el valor analógico del sensor LM35


  
  
                           // En el caso del LM35, cada 10 mV se corresponde con 1°C

  // Imprime la temperatura en el Monitor Serie
  Serial.print("TemperaturaLM35: ");
  Serial.print(int(temp));
  Serial.println(" °C");
  return temp;
}

void loadEeprom(){
  unsigned char delayEprom = 10;
  //Se recupera información alamacenada en la memoria eemprom sobre el estado de los canales y temporizadores  
  garden.eventsChannel1[0].state=espEeprom.getBool("Ch1Timer1State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[0].action=espEeprom.getBool("Ch1Timer1Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].hour=espEeprom.getInt("Ch1Timer1Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[0].min=espEeprom.getInt("Ch1Timer1Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[0].sun=espEeprom.getBool("Ch1Timer1Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].mon=espEeprom.getBool("Ch1Timer1Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].thurs=espEeprom.getBool("Ch1Timer1Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].wend=espEeprom.getBool("Ch1Timer1Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].tues=espEeprom.getBool("Ch1Timer1Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].fri=espEeprom.getBool("Ch1Timer1Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[0].satu=espEeprom.getBool("Ch1Timer1Satu",true);
  delay(delayEprom); 

  garden.eventsChannel1[1].state=espEeprom.getBool("Ch1Timer2State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[1].action=espEeprom.getBool("Ch1Timer2Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].hour=espEeprom.getInt("Ch1Timer2Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[1].min=espEeprom.getInt("Ch1Timer2Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[1].sun=espEeprom.getBool("Ch1Timer2Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].mon=espEeprom.getBool("Ch1Timer2Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].thurs=espEeprom.getBool("Ch1Timer2Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].wend=espEeprom.getBool("Ch1Timer2Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].tues=espEeprom.getBool("Ch1Timer2Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].fri=espEeprom.getBool("Ch1Timer2Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[1].satu=espEeprom.getBool("Ch1Timer2Satu",true);
  delay(delayEprom); 

  garden.eventsChannel1[2].state=espEeprom.getBool("Ch1Timer3State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[2].action=espEeprom.getBool("Ch1Timer3Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].hour=espEeprom.getInt("Ch1Timer3Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[2].min=espEeprom.getInt("Ch1Timer3Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[2].sun=espEeprom.getBool("Ch1Timer3Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].mon=espEeprom.getBool("Ch1Timer3Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].thurs=espEeprom.getBool("Ch1Timer3Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].wend=espEeprom.getBool("Ch1Timer3Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].tues=espEeprom.getBool("Ch1Timer3Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].fri=espEeprom.getBool("Ch1Timer3Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[2].satu=espEeprom.getBool("Ch1Timer3Satu",true);
  delay(delayEprom); 

  garden.eventsChannel1[3].state=espEeprom.getBool("Ch1Timer4State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[3].action=espEeprom.getBool("Ch1Timer4Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].hour=espEeprom.getInt("Ch1Timer4Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[3].min=espEeprom.getInt("Ch1Timer4Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[3].sun=espEeprom.getBool("Ch1Timer4Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].mon=espEeprom.getBool("Ch1Timer4Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].thurs=espEeprom.getBool("Ch1Timer4Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].wend=espEeprom.getBool("Ch1Timer4Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].tues=espEeprom.getBool("Ch1Timer4Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].fri=espEeprom.getBool("Ch1Timer4Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[3].satu=espEeprom.getBool("Ch1Timer4Satu",true);
  delay(delayEprom);

  garden.eventsChannel1[4].state=espEeprom.getBool("Ch1Timer5State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[4].action=espEeprom.getBool("Ch1Timer5Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].hour=espEeprom.getInt("Ch1Timer5Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[4].min=espEeprom.getInt("Ch1Timer5Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[4].sun=espEeprom.getBool("Ch1Timer5Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].mon=espEeprom.getBool("Ch1Timer5Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].thurs=espEeprom.getBool("Ch1Timer5Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].wend=espEeprom.getBool("Ch1Timer5Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].tues=espEeprom.getBool("Ch1Timer5Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].fri=espEeprom.getBool("Ch1Timer5Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[4].satu=espEeprom.getBool("Ch1Timer5Satu",true);
  delay(delayEprom);  

  garden.eventsChannel1[5].state=espEeprom.getBool("Ch1Timer6State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel1[5].action=espEeprom.getBool("Ch1Timer6Action",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].hour=espEeprom.getInt("Ch1Timer6Hour",6);
  delay(delayEprom); 
  garden.eventsChannel1[5].min=espEeprom.getInt("Ch1Timer6Min",0);
  delay(delayEprom); 
  garden.eventsChannel1[5].sun=espEeprom.getBool("Ch1Timer6Sun",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].mon=espEeprom.getBool("Ch1Timer6Mon",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].thurs=espEeprom.getBool("Ch1Timer6Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].wend=espEeprom.getBool("Ch1Timer6Wend",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].tues=espEeprom.getBool("Ch1Timer6Tues",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].fri=espEeprom.getBool("Ch1Timer6Fri",true);
  delay(delayEprom); 
  garden.eventsChannel1[5].satu=espEeprom.getBool("Ch1Timer6Satu",true);
  delay(delayEprom); 

  garden.eventsChannel2[0].state=espEeprom.getBool("Ch2Timer1State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel2[0].action=espEeprom.getBool("Ch2Timer1Action",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].hour=espEeprom.getInt("Ch2Timer1Hour",6);
  delay(delayEprom); 
  garden.eventsChannel2[0].min=espEeprom.getInt("Ch2Timer1Min",0);
  delay(delayEprom); 
  garden.eventsChannel2[0].sun=espEeprom.getBool("Ch2Timer1Sun",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].mon=espEeprom.getBool("Ch2Timer1Mon",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].thurs=espEeprom.getBool("Ch2Timer1Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].wend=espEeprom.getBool("Ch2Timer1Wend",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].tues=espEeprom.getBool("Ch2Timer1Tues",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].fri=espEeprom.getBool("Ch2Timer1Fri",true);
  delay(delayEprom); 
  garden.eventsChannel2[0].satu=espEeprom.getBool("Ch2Timer1Satu",true);
  delay(delayEprom); 

  garden.eventsChannel2[1].state=espEeprom.getBool("Ch2Timer2State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel2[1].action=espEeprom.getBool("Ch2Timer2Action",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].hour=espEeprom.getInt("Ch2Timer2Hour",6);
  delay(delayEprom); 
  garden.eventsChannel2[1].min=espEeprom.getInt("Ch2Timer2Min",0);
  delay(delayEprom); 
  garden.eventsChannel2[1].sun=espEeprom.getBool("Ch2Timer2Sun",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].mon=espEeprom.getBool("Ch2Timer2Mon",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].thurs=espEeprom.getBool("Ch2Timer2Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].wend=espEeprom.getBool("Ch2Timer2Wend",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].tues=espEeprom.getBool("Ch2Timer2Tues",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].fri=espEeprom.getBool("Ch2Timer2Fri",true);
  delay(delayEprom); 
  garden.eventsChannel2[1].satu=espEeprom.getBool("Ch2Timer2Satu",true);
  delay(delayEprom); 

  garden.eventsChannel2[2].state=espEeprom.getBool("Ch2Timer3State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel2[2].action=espEeprom.getBool("Ch2Timer3Action",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].hour=espEeprom.getInt("Ch2Timer3Hour",6);
  delay(delayEprom); 
  garden.eventsChannel2[2].min=espEeprom.getInt("Ch2Timer3Min",0);
  delay(delayEprom); 
  garden.eventsChannel2[2].sun=espEeprom.getBool("Ch2Timer3Sun",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].mon=espEeprom.getBool("Ch2Timer3Mon",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].thurs=espEeprom.getBool("Ch2Timer3Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].wend=espEeprom.getBool("Ch2Timer3Wend",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].tues=espEeprom.getBool("Ch2Timer3Tues",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].fri=espEeprom.getBool("Ch2Timer3Fri",true);
  delay(delayEprom); 
  garden.eventsChannel2[2].satu=espEeprom.getBool("Ch2Timer3Satu",true);
  delay(delayEprom); 

  garden.eventsChannel2[3].state=espEeprom.getBool("Ch2Timer4State",false); //false valor por defecto por si no encuentra la ruta
  delay(delayEprom);
  garden.eventsChannel2[3].action=espEeprom.getBool("Ch2Timer4Action",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].hour=espEeprom.getInt("Ch2Timer4Hour",6);
  delay(delayEprom); 
  garden.eventsChannel2[3].min=espEeprom.getInt("Ch2Timer4Min",0);
  delay(delayEprom); 
  garden.eventsChannel2[3].sun=espEeprom.getBool("Ch2Timer4Sun",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].mon=espEeprom.getBool("Ch2Timer4Mon",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].thurs=espEeprom.getBool("Ch2Timer4Thurs",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].wend=espEeprom.getBool("Ch2Timer4Wend",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].tues=espEeprom.getBool("Ch2Timer4Tues",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].fri=espEeprom.getBool("Ch2Timer4Fri",true);
  delay(delayEprom); 
  garden.eventsChannel2[3].satu=espEeprom.getBool("Ch2Timer4Satu",true);
  delay(delayEprom);
}

void CausaError(void){
  Serial.println("FAILED");
  Serial.println("REASON: " + fbdoStreaming.errorReason());
  Serial.println("------------------------------------");
  Serial.println();
  
}

#endif //_MYFUCTIONS_H