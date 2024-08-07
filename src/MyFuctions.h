#ifndef _MYFUCTIONS_H
#define _MYFUCTIONS_H

#include <Arduino.h>
#include <SmartGreenhouse.h>
#include <FirebaseClient.h>
#include <FirebaseJson.h>
#include <Wire.h>           // Para la comunicación I2C con los sensores
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"         // Para el sensor de temperatura y humedad DHT
#include <BluetoothSerial.h>
#include <NTPClient.h>
#include <Preferences.h>

#define CHANNELS_TOTAL 4
#define TIMERS_TOTAL 4
                       //Canal,timer           
const char* stateChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"stateCh1Timer1","stateCh1Timer2","stateCh1Timer3","stateCh1Timer4"},
    {"stateCh2Timer1","stateCh2Timer2","stateCh2Timer3","stateCh2Timer4"},
    {"stateCh3Timer1","stateCh3Timer2","stateCh3Timer3","stateCh3Timer4"},
    {"stateCh4Timer1","stateCh4Timer2","stateCh4Timer3","stateCh4Timer4"}
}; 

const char* actionChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"actionCh1Timer1","actionCh1Timer2","actionCh1Timer3","actionCh1Timer4"},
    {"actionCh2Timer1","actionCh2Timer2","actionCh2Timer3","actionCh2Timer4"},
    {"actionCh3Timer1","actionCh3Timer2","actionCh3Timer3","actionCh3Timer4"},
    {"actionCh4Timer1","actionCh4Timer2","actionCh4Timer3","actionCh4Timer4"}
}; 

const char* sunChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"sunCh1Timer1","sunCh1Timer2","sunCh1Timer3","sunCh1Timer4"},
    {"sunCh2Timer1","sunCh2Timer2","sunCh2Timer3","sunCh2Timer4"},
    {"sunCh3Timer1","sunCh3Timer2","sunCh3Timer3","sunCh3Timer4"},
    {"sunCh4Timer1","sunCh4Timer2","sunCh4Timer3","sunCh4Timer4"}
}; 

const char* monChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"monCh1Timer1","monCh1Timer2","monCh1Timer3","monCh1Timer4"},
    {"monCh2Timer1","monCh2Timer2","monCh2Timer3","monCh2Timer4"},
    {"monCh3Timer1","monCh3Timer2","monCh3Timer3","monCh3Timer4"},
    {"monCh4Timer1","monCh4Timer2","monCh4Timer3","monCh4Timer4"}
}; 

const char* tuesChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"tuesCh1Timer1","tuesCh1Timer2","tuesCh1Timer3","tuesCh1Timer4"},
    {"tuesCh2Timer1","tuesCh2Timer2","tuesCh2Timer3","tuesCh2Timer4"},
    {"tuesCh3Timer1","tuesCh3Timer2","tuesCh3Timer3","tuesCh3Timer4"},
    {"tuesCh4Timer1","tuesCh4Timer2","tuesCh4Timer3","tuesCh4Timer4"}
}; 

const char* wendChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"wendCh1Timer1","wendCh1Timer2","wendCh1Timer3","wendCh1Timer4"},
    {"wendCh2Timer1","wendCh2Timer2","wendCh2Timer3","wendCh2Timer4"},
    {"wendCh3Timer1","wendCh3Timer2","wendCh3Timer3","wendCh3Timer4"},
    {"wendCh4Timer1","wendCh4Timer2","wendCh4Timer3","wendCh4Timer4"}
}; 

const char* thursChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"thursCh1Timer1","thursCh1Timer2","thursCh1Timer3","thursCh1Timer4"},
    {"thursCh2Timer1","thursCh2Timer2","thursCh2Timer3","thursCh2Timer4"},
    {"thursCh3Timer1","thursCh3Timer2","thursCh3Timer3","thursCh3Timer4"},
    {"thursCh4Timer1","thursCh4Timer2","thursCh4Timer3","thursCh4Timer4"}
}; 

const char* friChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"friCh1Timer1","friCh1Timer2","friCh1Timer3","friCh1Timer4"},
    {"friCh2Timer1","friCh2Timer2","friCh2Timer3","friCh2Timer4"},
    {"friCh3Timer1","friCh3Timer2","friCh3Timer3","friCh3Timer4"},
    {"friCh4Timer1","friCh4Timer2","friCh4Timer3","friCh4Timer4"}
};

const char* satuChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"satuCh1Timer1","satuCh1Timer2","satuCh1Timer3","satuCh1Timer4"},
    {"satuCh2Timer1","satuCh2Timer2","satuCh2Timer3","satuCh2Timer4"},
    {"satuCh3Timer1","satuCh3Timer2","satuCh3Timer3","satuCh3Timer4"},
    {"satuCh4Timer1","satuCh4Timer2","satuCh4Timer3","satuCh4Timer4"}
};

const char* hourChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"hourCh1Timer1","hourCh1Timer2","hourCh1Timer3","hourCh1Timer4"},
    {"hourCh2Timer1","hourCh2Timer2","hourCh2Timer3","hourCh2Timer4"},
    {"hourCh3Timer1","hourCh3Timer2","hourCh3Timer3","hourCh3Timer4"},
    {"hourCh4Timer1","hourCh4Timer2","hourCh4Timer3","hourCh4Timer4"}
};

const char* minChLabels[CHANNELS_TOTAL][TIMERS_TOTAL] = {
    {"minCh1Timer1","minCh1Timer2","minCh1Timer3","minCh1Timer4"},
    {"minCh2Timer1","minCh2Timer2","minCh2Timer3","minCh2Timer4"},
    {"minCh3Timer1","minCh3Timer2","minCh3Timer3","minCh3Timer4"},
    {"minCh4Timer1","minCh4Timer2","minCh4Timer3","minCh4Timer4"}
};

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

bool stateCh1 = false;
bool stateCh2 = false;
bool stateCh3 = false;
bool stateCh4 = false;

//VARIABLES PARA LA CONFIGURACION DE LA RED WIFI
volatile bool modeConfigBT = false; 
bool modeConfigFire = true;
//bool modoConfigUID = true;
bool bandUID = true;
String ssidWifi="MILO";  //"JuanD",   REDGOINN,         Inverna/ESCOBAR_ME        Moto_AH    
String passWifi="MILOVALERO17";  //"Cata1979",  900791927G             wiracocha          12345678
//String userID="";
String uidUserFire="2k147bi5U8WDFrt3OWHOc0KMg7D3";  //2k147bi5U8WDFrt3OWHOc0KMg7D3   mSeD55cmDSeuSRx9T8yceAehvpA2

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

DefaultNetwork network; // initilize with boolean parameter to enable/disable network reconnection

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);

FirebaseApp app;

// Define Firebase Data object
//FirebaseJson jsFb;
//FirebaseJsonData jsData;
String jsStr;

WiFiClientSecure ssl_client1, ssl_client2;

using AsyncClient = AsyncClientClass;

AsyncClient aClient(ssl_client1, getNetwork(network)), aClient2(ssl_client2, getNetwork(network));

RealtimeDatabase Database;

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

//String lastCharTimer = "";
//String lastCharChannel = "";

struct channel{
  uint8_t pin = 1;
  bool state = false;
};

String userPath = "/users/"+uidUserFire;

#define HOME  1
//VARIABLES PARA EL CONTROL MAQUINA DE ESTADOS PRINCIPAL
unsigned char state = HOME;  //HOME

bool flagShedulesStart = true;
bool flagTimerCh1 = false;
hw_timer_t *timer1 = NULL; // Apuntador a variable de tipo hw_timer_t que luego usaremos en la función de configuración de Arduino.

// Prototipos de funciones
void readSensorsTask(void *pvParameters);
void controlOutputsTask(void *pvParameters);

void testHwm(char * taskName);
bool InitWiFi(String SSID, String PASS);
bool initFirebase(String email, String pass, String path);
void asyncCB(AsyncResult &aResult);
void printResult(AsyncResult &aResult);
void receiveJsonData(String jsStr, String currentPath);
void receiveBoolData(bool boolValue, String currentPath);
void lockTimerCh(uint8_t);

void printMsg(String label,int val);
void ledBlinkMillis(unsigned int velocidad);
void updateTime();
void getInfoTimerChanel(String lastCharChannel,String lastCharTimer, String jsStr);
float readLm35();
void loadEeprom();
void CausaError(void);

void IRAM_ATTR onTimer1(){
  flagShedulesStart = true;
  //printMsg("Timmer ", 1);
}

void readSensorsTask(void *pvParameters) {
  
  while (1) {
    //garden.sensorValue[DHT_PIN]=garden.readDigitalSensor(DHT_PIN);
    //garden.sensorValue[DHT_PIN]=random(25,40);
    //float temperatureDHT = dht.readTemperature();
    
    float temperatureDHT = random(25,40);
    printMsg("Temperatura: ", temperatureDHT);
    //Firebase.set(fbdo, userPath + "/sensors/0",temperatureDHT);
    vTaskDelay(pdMS_TO_TICKS(1000));
    //float humidityDHT = dht.readHumidity();
    float humidityDHT = random(25,40);
    //Firebase.set(fbdo, userPath +"/sensors/1",humidityDHT);
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
    //boolean bandStateWifi = false;
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
      Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
      Serial.println("Initializing app...");


    // with minimum receive and transmit buffer size setting as following.
    // ssl_client1.setBufferSizes(1024, 512);
    // ssl_client2.setBufferSizes(1024, 512);
    // Note that, because the receive buffer size was set to minimum safe value, 1024, the large server response may not be able to handle.
    // The WiFiClientSecure uses 1k less memory than ESP_SSLClient.

    ssl_client1.setInsecure();
    ssl_client2.setInsecure();
      
    initializeApp(aClient2, app, getAuth(user_auth), asyncCB, "authTask");
    // Binding the FirebaseApp for authentication handler.
    // To unbind, use Database.resetApp();
    app.getApp<RealtimeDatabase>(Database);

    Database.url(DATABASE_URL);

    // Since v1.2.1, in SSE mode (HTTP Streaming) task, you can filter the Stream events by using RealtimeDatabase::setSSEFilters(<keywords>),
    // which the <keywords> is the comma separated events.
    // The event keywords supported are:
    // get - To allow the http get response (first put event since stream connected).
    // put - To allow the put event.
    // patch - To allow the patch event.
    // keep-alive - To allow the keep-alive event.
    // cancel - To allow the cancel event.
    // auth_revoked - To allow the auth_revoked event.
    // To clear all prevousely set filter to allow all Stream events, use RealtimeDatabase::setSSEFilters().
    Database.setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");

    // The "unauthenticate" error can be occurred in this case because we don't wait
    // the app to be authenticated before connecting the stream.
    // This is ok as stream task will be reconnected automatically when the app is authenticated.
    Database.get(aClient, "/users/2k147bi5U8WDFrt3OWHOc0KMg7D3", asyncCB, true /* SSE mode (HTTP Streaming) */, "streamTask");

      return confirm;
}

void asyncCB(AsyncResult &aResult){
    // WARNING!
    // Do not put your codes inside the callback and printResult.
    Serial.println("Cambios en la base de datos");

    printResult(aResult);
}

void printResult(AsyncResult &aResult){

    if (aResult.isEvent()){
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.appEvent().message().c_str(), aResult.appEvent().code());
    }

    if (aResult.isDebug()){
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError()){
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }

    if (aResult.available()) {
        RealtimeDatabaseResult &RTDB = aResult.to<RealtimeDatabaseResult>();
        if (RTDB.isStream()){

            Serial.println("----------------------------");
            Firebase.printf("task: %s\n", aResult.uid().c_str());
            Firebase.printf("event: %s\n", RTDB.event().c_str());
            Firebase.printf("path: %s\n", RTDB.dataPath().c_str());
            Firebase.printf("data: %s\n", RTDB.to<const char *>());
            Firebase.printf("type: %d\n", RTDB.type());

            // The stream event from RealtimeDatabaseResult can be converted to the values as following.
            //bool v1 = RTDB.to<bool>();
            //int   v2 = RTDB.to<int>();
            //float v3 = RTDB.to<float>();
            //double v4 = RTDB.to<double>();

            switch (RTDB.type()){
                case 4:{  //BOOL
                    receiveBoolData(RTDB.to<bool>(),String(RTDB.dataPath()));
                    

                break;}
                case 6:{  //JSON
                    //receiveBoolData();
                    jsStr = RTDB.to<String>();
                    

                    //String currenPath = String(RTDB.dataPath());

                    receiveJsonData(jsStr,String(RTDB.dataPath()));

                    //String lastCharChannel = namePath.substring(17,18);  //18 posicion en el paht del canal
                    //String lastCharTimer = namePath.substring(namePath.length() - 1);
                break;}
                
                default:
                  break;
            }
            
            
        }
        else
        {
            Serial.println("----------------------------");
            Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
        }

        #if defined(ESP32) || defined(ESP8266)
                Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
        #elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
                Firebase.printf("Free Heap: %d\n", rp2040.getFreeHeap());
        #endif
    }
}

void receiveJsonData(String jsStr, String currentPath){
    
    String lastCharChannel = currentPath.substring(17,18);  //18 posicion en el paht del canal
    String lastCharTimer = currentPath.substring(currentPath.length() - 1);


    Serial.println(currentPath);
    

    if(currentPath.equals("/channels/channel"+lastCharChannel+"/timers/timer"+lastCharTimer)){
        
        getInfoTimerChanel(lastCharChannel,lastCharTimer,jsStr);
        printMsg("Horariooooooooo CH"+lastCharTimer+": ",  200);
    }
}

void receiveBoolData(bool boolValue, String currentPath){
    String lastCharChannel = currentPath.substring(17,18);  ////se recupera el texto correspondiente al numero del timer , ejemplo: "2", el valor 30 corresponde a la posición del numero del timer en el PATH
    String lastCharTimer = currentPath.substring(31,32);

    if(currentPath.equals("/channels/channel"+lastCharChannel+"/state")){  //Con la ultima letra del namePath se define cual fue el canal que se modifico
       
        //Se recibe la data modificada eb FB
        garden.channel.state=boolValue;
        garden.channel.numberChannel=lastCharChannel.toInt();

        //Se guarda en un arreglo que contiene los estados de cada canal y en la Eeprom 
        garden.enableChFlag[lastCharChannel.toInt()-1]=garden.channel.state;

        String nameEeprom = "Ch"+lastCharChannel+"State";
        espEeprom.putBool(nameEeprom.c_str(),garden.channel.state);

        //Se desactivan los eventos relacionados al canal, por si hubiese alguno activado de manera que se deje trabajar en modo manual (Con los botones)
        //Desactivar los timers del canal lastCharChannel.toInt()
        //lockTimerCh(lastCharChannel.toInt());
        
        //Se aplica el cambio
        //digitalWrite(garden.ch[lastCharChannel.toInt()-1],boolValue);
        garden.enableChannel(garden.channel);
        
        printMsg("stateChannel"+lastCharChannel+": ",  garden.enableChFlag[garden.channel.numberChannel-1]);
        
        //printMsg("stateChannelllllllllllll"+lastCharChannel+": ",  boolValue);
    
    }

    if(currentPath.equals("/channels/channel"+lastCharChannel+"/timers/timer"+lastCharTimer+"/state")){  //Se vigila si se activa o se desactiva el timer
        
        switch (lastCharChannel.toInt()){
            case 1:{
                garden.eventsChannel1[lastCharTimer.toInt()-1].state = boolValue;
                //ruta = "Ch1Timer"+lastCharTimer+"State";
                espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],boolValue);
                
              break;}
            case 2:{
                garden.eventsChannel2[lastCharTimer.toInt()-1].state = boolValue;
                //ruta = "Ch2Timer"+lastCharTimer+"State";
                espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],boolValue);
              break;}
            case 3:{
                garden.eventsChannel3[lastCharTimer.toInt()-1].state = boolValue;
                //ruta = "Ch1Timer"+lastCharTimer+"State";
                espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],boolValue);
              break;}
            case 4:{
                garden.eventsChannel4[lastCharTimer.toInt()-1].state = boolValue;
                //ruta = "Ch1Timer"+lastCharTimer+"State";
                espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],boolValue);
              break;}
          default:
            break;
        }

        printMsg("stateTimer"+lastCharTimer+": ", boolValue);
        
        
    }
}

void lockTimerCh(uint8_t ch){

    switch (ch){  
        case 1:{ //Se desactivan todos los eventos del canal 1
            for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                garden.eventsChannel1[event].state = false;
                espEeprom.putBool(stateChLabels[ch-1][event],false);
                //Firebase.set(fbdo, userPath + "/sensors/0",temperatureDHT); hace falta implementar en todos
            }
        break;}               
        case 2:{ //Se desactivan todos los eventos del canal 2
            for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                garden.eventsChannel2[event].state = false;
                espEeprom.putBool(stateChLabels[ch-1][event],false);
            }
        break;}
        case 3:{ //Se desactivan todos los eventos del canal 3
            for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                garden.eventsChannel3[event].state = false;
                espEeprom.putBool(stateChLabels[ch-1][event],false);
            }
        break;}
        case 4:{ //Se desactivan todos los eventos del canal 4
            for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                garden.eventsChannel4[event].state = false;
                espEeprom.putBool(stateChLabels[ch-1][event],false);
            }
        break;}

        default:
          break;
    }
}

void getInfoTimerChanel(String lastCharChannel,String lastCharTimer, String jsStr){

    FirebaseJsonData jsData;
    FirebaseJson jsFb;

    jsFb.setJsonData(jsStr);

    switch (lastCharChannel.toInt()){
        case 1:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel1[lastCharTimer.toInt()-1].state = jsData.boolValue;
            //Serial.print("State: "); Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].state); 
            //ruta = "Ch1Timer" + lastCharTimer + "State";
            espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);
            
            jsFb.get(jsData,"/action");
            garden.eventsChannel1[lastCharTimer.toInt()-1].action = jsData.boolValue;
            //Serial.print("Action: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].action);
            //ruta = "Ch1Timer" + lastCharTimer + "Action";
            espEeprom.putBool(actionChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/sun");
            garden.eventsChannel1[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            //Serial.print("D: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].sun);
            //ruta = "Ch1Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(sunChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/mon");
            garden.eventsChannel1[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            //Serial.print("L: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].mon);
            //ruta = "Ch1Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(monChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/tues");
            garden.eventsChannel1[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            //Serial.print("M: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].tues);
            //ruta = "Ch1Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(tuesChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/wend");
            garden.eventsChannel1[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            //Serial.print("W: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].wend);
            //ruta = "Ch1Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(wendChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            garden.eventsChannel1[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            //Serial.print("J: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].thurs);
            //ruta = "Ch1Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(thursChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/fri");
            garden.eventsChannel1[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            //Serial.print("V: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].fri);
            //ruta = "Ch1Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(friChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/satu");
            garden.eventsChannel1[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            //Serial.print("S: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].satu);
            //ruta = "Ch1Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(satuChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/hour");
            garden.eventsChannel1[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            //Serial.print("Hora: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].hour);
            //ruta = "Ch1Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(hourChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            jsFb.get(jsData,"/min");
            garden.eventsChannel1[lastCharTimer.toInt()-1].min = jsData.intValue; 
            //Serial.print("min: ");Serial.println(garden.eventsChannel1[lastCharTimer.toInt()-1].min);
            //ruta = "Ch1Timer" + lastCharTimer + "Min";
            espEeprom.putInt(minChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel1) / sizeof(garden.eventsChannel1[0]);
            garden.sortEventsByTime(garden.eventsChannel1, numEvents);
            garden.printEventTimes(garden.eventsChannel1, numEvents);
        break;}
        case 2:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel2[lastCharTimer.toInt()-1].state = jsData.boolValue;
            //Serial.print("State: "); Serial.println(jsData.boolValue); 
            //ruta = "Ch1Timer" + lastCharTimer + "State";
            espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);
            
            jsFb.get(jsData,"/action");
            //Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].action = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Action";
            espEeprom.putBool(actionChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/sun");
            //Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(sunChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/mon");
            //Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(monChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/tues");
            //Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(tuesChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/wend");
            //Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(wendChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            //Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(thursChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/fri");
            //Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(friChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/satu");
            //Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(satuChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/hour");
            //Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(hourChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            jsFb.get(jsData,"/min");
            //Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel2[lastCharTimer.toInt()-1].min = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Min";
            espEeprom.putInt(minChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel2) / sizeof(garden.eventsChannel2[0]);
            garden.sortEventsByTime(garden.eventsChannel2, numEvents);
            garden.printEventTimes(garden.eventsChannel2, numEvents);
        break;}
        case 3:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel3[lastCharTimer.toInt()-1].state = jsData.boolValue;
            //Serial.print("State: "); Serial.println(jsData.boolValue); 
            //ruta = "Ch1Timer" + lastCharTimer + "State";
            espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);
            
            jsFb.get(jsData,"/action");
            //Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].action = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Action";
            espEeprom.putBool(actionChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/sun");
           // Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(sunChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/mon");
            //Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(monChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/tues");
            //Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(tuesChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/wend");
            //Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(wendChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            //Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(thursChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/fri");
            //Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(friChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/satu");
            //Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(satuChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/hour");
            //Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(hourChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            jsFb.get(jsData,"/min");
            //Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel3[lastCharTimer.toInt()-1].min = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Min";
            espEeprom.putInt(minChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            int numEvents = sizeof(garden.eventsChannel3) / sizeof(garden.eventsChannel3[0]);
            garden.sortEventsByTime(garden.eventsChannel3, numEvents);
            garden.printEventTimes(garden.eventsChannel3, numEvents);
        break;}
        case 4:{
            jsFb.get(jsData,"/state");
            garden.eventsChannel4[lastCharTimer.toInt()-1].state = jsData.boolValue;
            //Serial.print("State: "); Serial.println(jsData.boolValue); 
            //ruta = "Ch1Timer" + lastCharTimer + "State";
            espEeprom.putBool(stateChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);
            
            jsFb.get(jsData,"/action");
            //Serial.print("Action: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].action = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Action";
            espEeprom.putBool(actionChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/sun");
            //Serial.print("D: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].sun = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Sun";
            espEeprom.putBool(sunChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/mon");
            //Serial.print("L: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].mon = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Mon";
            espEeprom.putBool(monChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/tues");
            //Serial.print("M: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].tues = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Tues";
            espEeprom.putBool(tuesChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/wend");
            //Serial.print("W: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].wend = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Wend";
            espEeprom.putBool(wendChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/thurs");
            //Serial.print("J: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].thurs = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Thurs";
            espEeprom.putBool(thursChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/fri");
            //Serial.print("V: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].fri = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Fri";
            espEeprom.putBool(friChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/satu");
            //Serial.print("S: ");Serial.println(jsData.boolValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].satu = jsData.boolValue;
            //ruta = "Ch1Timer" + lastCharTimer + "Satu";
            espEeprom.putBool(satuChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.boolValue);

            jsFb.get(jsData,"/hour");
            //Serial.print("Hora: ");Serial.println(jsData.intValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].hour = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Hour";
            espEeprom.putInt(hourChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

            jsFb.get(jsData,"/min");
            //Serial.print("min: ");Serial.println(jsData.intValue);
            garden.eventsChannel4[lastCharTimer.toInt()-1].min = jsData.intValue; 
            //ruta = "Ch1Timer" + lastCharTimer + "Min";
            espEeprom.putInt(minChLabels[lastCharChannel.toInt()-1][lastCharTimer.toInt()-1],jsData.intValue);

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
    unsigned char delayEprom = 3;

    uidUserFire = espEeprom.getString("uidUserFire",uidUserFire);
    Serial.print("uidUserFire: ");Serial.println(uidUserFire);
    delay(delayEprom);
    ssidWifi = espEeprom.getString("ssidWifi",ssidWifi);
    Serial.print("ssidWifi: ");Serial.println(ssidWifi);
    delay(delayEprom);
    passWifi = espEeprom.getString("passWifi",passWifi);
    Serial.print("passWifi: ");Serial.println(passWifi);
    delay(delayEprom);

    /*modeConfigBT = espEeprom.getBool("modeConfigBT",modeConfigBT);
    delay(delayEprom);*/

    garden.enableChFlag[0] = espEeprom.getBool("Ch1State",true);
    Serial.print("state Ch1: ");Serial.println(garden.enableChFlag[0]);
    delay(delayEprom);

    garden.enableChFlag[1] = espEeprom.getBool("Ch2State",true);
    Serial.print("state Ch2: ");Serial.println(garden.enableChFlag[1]);
    delay(delayEprom);

    garden.enableChFlag[2] = espEeprom.getBool("Ch3State",true);
    Serial.print("state Ch3: ");Serial.println(garden.enableChFlag[2]);
    delay(delayEprom);

    garden.enableChFlag[3] = espEeprom.getBool("Ch4State",true);
    Serial.print("state Ch4: ");Serial.println(garden.enableChFlag[3]);
    delay(delayEprom);

    

    for (uint8_t chn = 0; chn < CHANNELS_TOTAL; chn++){
        switch (chn){
          case 0:{
              for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                  garden.eventsChannel1[event].state=espEeprom.getBool(stateChLabels[chn][event],false); //false valor por defecto por si no encuentra la ruta
                  delay(delayEprom);
                  garden.eventsChannel1[event].action=espEeprom.getBool(actionChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].hour=espEeprom.getInt(hourChLabels[chn][event],6);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].min=espEeprom.getInt(minChLabels[chn][event],0);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].sun=espEeprom.getBool(sunChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].mon=espEeprom.getBool(monChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].thurs=espEeprom.getBool(thursChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].wend=espEeprom.getBool(wendChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].tues=espEeprom.getBool(tuesChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].fri=espEeprom.getBool(friChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel1[event].satu=espEeprom.getBool(satuChLabels[chn][event],true);
                  delay(delayEprom);
              }

              int numEvents = sizeof(garden.eventsChannel1) / sizeof(garden.eventsChannel1[0]);
              garden.sortEventsByTime(garden.eventsChannel1, numEvents);
              garden.printEventTimes(garden.eventsChannel1, numEvents);
          break;}
          case 1:{
              for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                  garden.eventsChannel2[event].state=espEeprom.getBool(stateChLabels[chn][event],false); //false valor por defecto por si no encuentra la ruta
                  delay(delayEprom);
                  garden.eventsChannel2[event].action=espEeprom.getBool(actionChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].hour=espEeprom.getInt(hourChLabels[chn][event],6);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].min=espEeprom.getInt(minChLabels[chn][event],0);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].sun=espEeprom.getBool(sunChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].mon=espEeprom.getBool(monChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].thurs=espEeprom.getBool(thursChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].wend=espEeprom.getBool(wendChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].tues=espEeprom.getBool(tuesChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].fri=espEeprom.getBool(friChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel2[event].satu=espEeprom.getBool(satuChLabels[chn][event],true);
                  delay(delayEprom);
              }
              
              int numEvents = sizeof(garden.eventsChannel2) / sizeof(garden.eventsChannel2[0]);
              garden.sortEventsByTime(garden.eventsChannel2, numEvents);
              garden.printEventTimes(garden.eventsChannel2, numEvents);
          break;}
          case 2:{
              for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                  garden.eventsChannel3[event].state=espEeprom.getBool(stateChLabels[chn][event],false); //false valor por defecto por si no encuentra la ruta
                  delay(delayEprom);
                  garden.eventsChannel3[event].action=espEeprom.getBool(actionChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].hour=espEeprom.getInt(hourChLabels[chn][event],6);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].min=espEeprom.getInt(minChLabels[chn][event],0);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].sun=espEeprom.getBool(sunChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].mon=espEeprom.getBool(monChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].thurs=espEeprom.getBool(thursChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].wend=espEeprom.getBool(wendChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].tues=espEeprom.getBool(tuesChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].fri=espEeprom.getBool(friChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel3[event].satu=espEeprom.getBool(satuChLabels[chn][event],true);
                  delay(delayEprom);
              }
              
              int numEvents = sizeof(garden.eventsChannel3) / sizeof(garden.eventsChannel3[0]);
              garden.sortEventsByTime(garden.eventsChannel3, numEvents);
              garden.printEventTimes(garden.eventsChannel3, numEvents);
          break;}
          case 3:{
              for (uint8_t event = 0; event < TIMERS_TOTAL; event++){
                  garden.eventsChannel4[event].state=espEeprom.getBool(stateChLabels[chn][event],false); //false valor por defecto por si no encuentra la ruta
                  delay(delayEprom);
                  garden.eventsChannel4[event].action=espEeprom.getBool(actionChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].hour=espEeprom.getInt(hourChLabels[chn][event],6);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].min=espEeprom.getInt(minChLabels[chn][event],0);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].sun=espEeprom.getBool(sunChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].mon=espEeprom.getBool(monChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].thurs=espEeprom.getBool(thursChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].wend=espEeprom.getBool(wendChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].tues=espEeprom.getBool(tuesChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].fri=espEeprom.getBool(friChLabels[chn][event],true);
                  delay(delayEprom); 
                  garden.eventsChannel4[event].satu=espEeprom.getBool(satuChLabels[chn][event],true);
                  delay(delayEprom);
              }
              
              int numEvents = sizeof(garden.eventsChannel4) / sizeof(garden.eventsChannel4[0]);
              garden.sortEventsByTime(garden.eventsChannel4, numEvents);
              garden.printEventTimes(garden.eventsChannel4, numEvents);
          break;}
          default:
            break;
        }

        delay(1500);
    }

    
}


#endif //_MYFUCTIONS_H