#include <Arduino.h>
#include "MyFuctions.h"
#include <ChannelCtrl.h> 

#define SENSOR_READ_TIME_MS 60000 // 1min

void readSensors(void);

void setup() {
  Serial.begin(9600);
  //garden.dhtInit(DHT_PIN);
  dht.begin();

  espEeprom.begin("garden",false);

  pinMode(LED_ESP,OUTPUT); 

  pinMode(garden.CH1, OUTPUT);
  pinMode(garden.CH2, OUTPUT);
  pinMode(garden.CH3, OUTPUT);
  pinMode(garden.CH4, OUTPUT);
  pinMode(garden.CH5, OUTPUT);
  pinMode(garden.CH6, OUTPUT);
  pinMode(garden.CH7, OUTPUT);
  pinMode(garden.CH8, OUTPUT);

  digitalWrite(garden.CH1,!LOW);
  digitalWrite(garden.CH2,!LOW);
  digitalWrite(garden.CH3,!LOW);
  digitalWrite(garden.CH4,!LOW);
  digitalWrite(garden.CH5,!LOW);
  digitalWrite(garden.CH6,!LOW);
  //digitalWrite(garden.CH7,!LOW);
  digitalWrite(garden.CH8,!LOW);

  /*if(InitWiFi(ssidWifi,passWifi)){ //Se verifica que la conexión a internet es correcta

      if(initFirebase(USER_EMAIL,USER_PASSWORD,USER_PATH)){
          Serial.println("...Se establecio StreamCalback con el PATH: ");Serial.println(USER_PATH);
          delay(3000);
      }else{
          Serial.println("...No se pudo establer StreamCalback con el PATH: ");Serial.println(USER_PATH);
          while (1){/* code *///}
      //}
  //} 

  timer1 = timerBegin(1,80,true); //se escogio un preescalador de 80, es decir se divide la frecuencia normal de operación(80MHz), entre 80(preescalador), quedando frecuencia para el contador del timer de 10MHZ(periodo 1 microsegundo)
  timerAttachInterrupt(timer1, &onTimer1, true); //Esta función recibe como entrada un puntero al temporizador inicializado, que almacenamos en nuestra variable global, la dirección de la función que manejará la interrupción y una bandera que indica si la interrupción que se va a generar es de flanco (verdadero) o de nivel (falso)
  timerAlarmWrite(timer1,60000000,true);  // Con un valor de 23000 garantizamos que la 
  timerAlarmEnable(timer1);

  unsigned char delayEprom = 100;
  
  /*garden.channel.numberChannel = garden.ch[0];
  garden.channel.state=espEeprom.getBool("Ch1State",true);
  delay(delayEprom);
  garden.enableChannel(garden.channel);

  garden.channel.numberChannel = garden.ch[1];
  garden.channel.state=espEeprom.getBool("Ch2State",true);
  delay(delayEprom);
  garden.enableChannel(garden.channel);*/

  //loadEeprom();

  /*int numEvents = sizeof(garden.eventsChannel1) / sizeof(garden.eventsChannel1[0]);
  garden.sortEventsByTime(garden.eventsChannel1, numEvents);
  garden.printEventTimes(garden.eventsChannel1, numEvents);*/

  modeConfigFire = true;
  modoConfigUID = true;

}

void loop() {

    if (modeConfigBT){

        //Se apaga todos los canales por control (Se podria definir diferente)
        for (size_t i = 0; i < 4; i++){
          garden.channel.numberChannel = 1;
          garden.channel.state = false;
          garden.enableChannel(garden.channel);
        }

        SerialBT.begin("SmartGarden");     // Inicializamos la comunicación bluetooth serial.
        modeConfigFire=true;
        switch (STATE){
          case STATE_UID:{
            ledBlinkMillis(700);
            Serial.print("********** STATE: "); Serial.println(STATE);
            
            while(SerialBT.available()){  // Checamos si el buffer del bluetooth serial tiene datos   
                if(bandUID){
                    /* Si tiene datos entrara en el while */
                    if(bandBT==false){
                      uidUserFire = SerialBT.readString();            // Leemos únicamente 8bits y se almacena en “valor” 
                      delay(100);
                      bandBT = true;
                    }

                    espEeprom.putString("uidUserFire",uidUserFire);
                    delay(100);
                    bandUID = false;
                    espEeprom.putString("bandUID",uidUserFire);
                    delay(100);
                }      
                
                /* Enviamos el valor al monitor serial  */
                Serial.println(uidUserFire);  
                bandBT = false; 
                STATE = STATE_NAME;
            }
            //STATE = STATE_NAME;
          break;
          }
          case STATE_NAME:{
            ledBlinkMillis(500);
            Serial.print("********** STATE: "); Serial.println(STATE);

            while(SerialBT.available()){  // Checamos si el buffer del bluetooth serial tiene datos
                /* Si tiene datos entrara en el while */
                if(bandBT==false){
                  ssidWifi = SerialBT.readString();            // Leemos únicamente 8bits y se almacena en “valor” 
                  delay(150);
                  bandBT = true;
                }

                espEeprom.putString("ssidWifi",ssidWifi);
                delay(150);
                
                /* Enviamos el valor al monitor serial  */
                Serial.println(ssidWifi);  
                bandBT = false; 
                STATE = STATE_PASS;
            }
          
          break;
          }
          case STATE_PASS:{
            ledBlinkMillis(300);
            Serial.print("********** STATE: "); Serial.println(STATE);
            while(SerialBT.available()){          // Checamos si el buffer del bluetooth serial tiene datos
                  /* Si tiene datos entrara en el while */
                  if(bandBT==false){
                    passWifi = SerialBT.readString();            // Leemos el string y se almacena en “PASS” 
                    delay(150);
                    bandBT = true;
                  }

                  espEeprom.putString("passWifi",passWifi);
                  delay(150);
                  
                  /* Enviamos el valor al monitor serial  */
                  Serial.println(passWifi);   
                  
                  STATE = STATE_WIFINIT;
            }
          break;
          }
          case STATE_WIFINIT:{

              ledBlinkMillis(100);
              if(SerialBT.disconnect()){
                  
                  Serial.print("********** STATE: "); Serial.println(STATE);
                  Serial.print("Nombre red: "); Serial.println(ssidWifi);
                  Serial.print("passWifi red: "); Serial.println(passWifi);
                  Serial.print("userID: "); Serial.println(uidUserFire);

                  modeConfigBT = false;
                  espEeprom.putBool("bandBT",modeConfigBT);
                  STATE = STATE_UID;
                  SerialBT.end();
              } else {
                Serial.println("ERROR DE DESCONEXION BT");
              }
              
          break;
          }
          default:{
          break;}
        }

    }else{
        if (modeConfigFire){
              SerialBT.end(); //Se finaliza la comunicación Bluethoot

              //Se inicializa la configuración WiFi y se actualiza la hora desde la web 
              //ssidWifi="Moto_AH";  //"JuanD",   REDGOINN,         VORTIC
              //passWifi="12345678";  //"Cata1979",  900791927G   9876543210*
              if(InitWiFi(ssidWifi,passWifi)){ //Se verifica que la conexión a internet es correcta

                  config.api_key = API_KEY;
                  // Assign the user sign in credentials
                  auth.user.email = "daferd19@gmail.com"; 
                  auth.user.password = "123456";
                  // Assign the project host and api key (required)
                  config.database_url = DATABASE_URL;
                  /* Assign the callback function for the long running token generation task */
                  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
                  Firebase.reconnectWiFi(true);
                  Firebase.setDoubleDigits(5);     

                  // required for large file data, increase Rx size as needed.
                  fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 4096 /* Tx buffer size in bytes from 512 - 16384 */);
                  fbdoStreaming.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);
            
                  // Inicializamos Firebase, mediante el URL y secreto de la base de datos del proyecto en Firebase.
                  //Firebase.begin(DB_URL, SECRET_KEY);
                  Firebase.begin(&config, &auth);
                
                  //Firebase.setReadTimeout(myFirebaseData, 1000 * 60);
        
                  //Tamaño y  tiempo de espera de escritura, tiny (1s), small (10s), medium (30s) and large (60s).
                  //tiny, small, medium, large and unlimited.
                  //Firebase.setwriteSizeLimit(myFirebaseData, "tiny");
                  userPath = "/users/"+ uidUserFire;  //Se almacena el path personalizado del usuario, a este se le asocia el uidUserFire de Firebase
                  Serial.println("PATH DEFINIDO: "+userPath);
                  delay(1000);
                  
                  modeConfigFire=false;
                  
              } 
              
        }else{
            if (WiFi.status() == WL_CONNECTED){

              //tempLm35 = readLm35();
            
                if(modoConfigUID){
                    Serial.print("********** INFORMACIÓN OBTENIDA DE FIREBASE **********");Serial.println();
                    

                    fbdoStreaming.keepAlive(5, 5, 1);
                    
                    if(!Firebase.beginStream(fbdoStreaming, userPath)){   
                        Serial.println("...No se establecio StreamCalback con el PATH: ");Serial.println(userPath);
                        CausaError();
                        while (1){
                            ledBlinkMillis(500);
                        }
                    }

                    Firebase.setStreamCallback(fbdoStreaming, firebaseCallback, timeoutCallback);
                    Serial.println("...Se establecio StreamCalback con el PATH: ");Serial.println(userPath);
                        delay(300);
              
                    digitalWrite(LED_ESP,HIGH); //Se enciende el led indicador para saber que el sistema esta listo para recibir ordenes de la APP
                    modoConfigUID = false;
                }else{
                        
                    if(flagTimer){
                        Serial.print("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());
                        stateChannel1 = garden.stateDefine(1,garden.eventsChannel1);  // la funcion stateDefine solo modifica al canal 1, se debe cambia REBISAR!!
                        stateChannel2 = garden.stateDefine(2,garden.eventsChannel2);
                        flagTimer = false;
                    }

                    
                    if (millis() - tiempoComp >= SENSOR_READ_TIME_MS) readSensors();
                    
                }

                
            }else{
                Serial.println("SE PERDIO LA CONEXION!!");
                contIntentos = 0;
                InitWiFi(ssidWifi,passWifi);
            }
        }
    }
}

void readSensors(void){
  tiempoComp = millis();

  float temperatureDHT = random(25,40);
  //float  temperatureDHT = dht.readTemperature();
  printMsg("TemperaturaDHT: ", temperatureDHT);
  Firebase.set(fbdo, userPath + "/sensors/0",temperatureDHT);
  delay(10);

  float humidityDHT = random(25,40);
  Firebase.set(fbdo, userPath+"/sensors/1",humidityDHT);
  delay(10);

  tempLm35 = readLm35();
  Firebase.set(fbdo, userPath+"/sensors/3",int(tempLm35));
  delay(10);
}