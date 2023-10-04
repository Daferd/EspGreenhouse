#include <Arduino.h>
#include <SmartGreenhouse.h>
#include <FirebaseESP32.h>  // Para la comunicación con Firebase
#include <Wire.h>           // Para la comunicación I2C con los sensores
#include <WiFi.h>
//#include <Adafruit_Sensor.h>
//#include <DHT.h>            // Para el sensor de temperatura y humedad DHT
//#include <Adafruit_ADS1015.h> // Para los sensores analógicos

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>


// Definiciones de pines
#define DHT_PIN 4        // Pin del sensor DHT
#define BUTTON_PIN 2     // Pin del pulsador
#define NUM_ANALOG_SENSORS 6 // Número de sensores analógicos
#define NUM_DIGITAL_OUTPUTS 8 // Número de salidas digitales

// Configuración del sensor DHT
//DHT dht(DHT_PIN, DHT22);

/* 1. Define the API Key */
#define API_KEY "AIzaSyCu7jNCoSoW3hLku1biL2RQ0tIvp0jzFsA"

/* 2. Define the RTDB URL */
#define DATABASE_URL "ejemploesp8266-533d5-default-rtdb.firebaseio.com"

/* 3. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "daferd19@gmail.com"
#define USER_PASSWORD "123456"
#define USER_PATH "/users/AVW8kzZI5oMe8JOwoZp49eX8FNP2"

//Variable y Estructuras para el control del Timer
/*struct ChannelServer {
  uint8_t numberChannel;
  uint8_t state;
};

ChannelServer channel;*/

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

String SSID="REDGOINN";
String PASS="900791927G";
// Variables globales
float temperature = 0.0;
float humidity = 0.0;
bool buttonState = false;
int analogSensorValues[NUM_ANALOG_SENSORS];
bool digitalOutputs[NUM_DIGITAL_OUTPUTS];

// Prototipos de funciones
void readSensorsTask(void *pvParameters);
void controlOutputsTask(void *pvParameters);


void testHwm(char * taskName);
bool InitWiFi(String SSID, String PASS);
bool initFirebase(String email, String pass, String path);
void firebaseCallback(StreamData data);
void timeoutCallback(bool timeCallback);
void printMsg(String label,int val);

void setup() {
  Serial.begin(9600);

  if(InitWiFi(SSID,PASS)){ //Se verifica que la conexión a internet es correcta

      if(initFirebase(USER_EMAIL,USER_PASSWORD,USER_PATH)){
          Serial.println("...Se establecio StreamCalback con el PATH: ");Serial.println(USER_PATH);
          delay(3000);
      }else{
          Serial.println("...No se pudo establer StreamCalback con el PATH: ");Serial.println(USER_PATH);
          while (1){/* code */}
      }

           // Crear tareas para leer sensores y controlar salidas
      xTaskCreatePinnedToCore(
        readSensorsTask,     // Función de la tarea
        "ReadSensorsTask",   // Nombre de la tarea
        5800,                // Tamaño de la pila de la tarea
        NULL,                // Parámetros de la tarea
        1,                   // Prioridad de la tarea (mayor número = mayor prioridad)
        NULL,                // Manejador de la tarea
        0                    // Núcleo de la CPU (0 o 1)

      );

      xTaskCreatePinnedToCore(
        controlOutputsTask,   // Función de la tarea
        "ControlOutputsTask", // Nombre de la tarea
        2000,                // Tamaño de la pila de la tarea
        NULL,                // Parámetros de la tarea
        1,                   // Prioridad de la tarea (mayor número = mayor prioridad)
        NULL,                // Manejador de la tarea
        0                    // Núcleo de la CPU (0 o 1)
      );

  } 

}

void loop() {


}

void readSensorsTask(void *pvParameters) {
  
  while (1) {
    //garden.sensorValue[DHT_PIN]=garden.readDigitalSensor(DHT_PIN);
    garden.sensorValue[DHT_PIN]=random(25,40);
    Firebase.set(fbdo,"/users/AVW8kzZI5oMe8JOwoZp49eX8FNP2/sensors/s1",garden.sensorValue[DHT_PIN]);
    //testHwm("ReadSensorsTask");
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

void controlOutputsTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {

    

    testHwm("ControlOutputsTask"); // Esta función consume 512 bytes aproximadamente
    vTaskDelay(pdMS_TO_TICKS(1000));
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

    while(((WiFi.status() != WL_CONNECTED) && contIntentos == 0) || (contIntentos <= MAX_INTENT)){     // Se quedata en este bucle hasta que el estado del WiFi sea diferente a desconectado.  
        contIntentos++;
        Serial.print(".");
        //fuente.ledBlinkMillis(50);
        delay(50);
    }

    if(WiFi.status() == WL_CONNECTED){        // Si el estado del WiFi es conectado entra al If
      contIntentos = 0;
      Serial.println();
      Serial.println();
      Serial.println("Conexion exitosa!!!");
      Serial.println("");
      Serial.print("Tu IP es: ");
      Serial.println(WiFi.localIP());

      //updateTime();
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

    Serial.println(namePath);

    if(namePath.equals("channels/channel1")){
        Firebase.get(fbdoStreaming, "/infoUsuarios/parametrosShows");
        jsStr = fbdoStreaming.jsonString();
        jsFb.setJsonData(jsStr);

        jsFb.get(jsData,"/xxxxxx");
        printMsg("xxxxxx: ", jsData.intValue);
        //state = jsData.intValue;
        //fuente.espEeprom.putInt("state",state);
    }

    
  }

  if(data.dataType().equals("boolean")){
      boolean valorBool = fbdoStreaming.boolData();
      namePath = fbdoStreaming.dataPath();
      
      String lastCharTimer = namePath.substring(12,13); //se recupera el texto correspondiente al numero del timer , ejemplo: "2", el valor 30 corresponde a la posición del numero del timer en el PATH
      //Serial.println("/timers/timersLuminarias/timer"+lastCharTimer);
      
      Serial.print("The change is in: "); Serial.println(namePath);
      //Serial.print("Ultima letra: "); Serial.println(lastCharTimer);

      if(namePath.equals("/channels/ch"+lastCharTimer)){  //Con la ultima letra del namePath se define cual fue el canal que se modifico
          //Firebase.get(fbdo,"users");
          garden.channel.state=valorBool;
          garden.channel.numberChannel=lastCharTimer.toInt();
          garden.enableChannel(garden.channel);
          
          printMsg("stateChanel"+lastCharTimer+": ", garden.channel.state);
      }

  }


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