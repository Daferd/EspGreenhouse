#include <Arduino.h>
#include "MyFuctions.h"

void setup() {
  Serial.begin(9600);
  //garden.dhtInit(DHT_PIN);
  dht.begin();

  espEeprom.begin("garden",false); 

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

  /*if(InitWiFi(SSID,PASS)){ //Se verifica que la conexión a internet es correcta

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
  
  garden.channel.numberChannel = garden.ch[0];
  garden.channel.state=espEeprom.getBool("Ch1State",true);
  delay(delayEprom);
  garden.enableChannel(garden.channel);

  garden.channel.numberChannel = garden.ch[1];
  garden.channel.state=espEeprom.getBool("Ch2State",true);
  delay(delayEprom);
  garden.enableChannel(garden.channel);

  loadEeprom();

  /*garden.eventsChannel1[0].state=false;
  garden.eventsChannel1[0].action=true;
  garden.eventsChannel1[0].hour=17;
  garden.eventsChannel1[0].min=46;
  garden.eventsChannel1[0].mon=false;
  garden.eventsChannel1[0].thurs=false;
  garden.eventsChannel1[0].wend=false;
  garden.eventsChannel1[0].tues=false;
  garden.eventsChannel1[0].fri=false;
  garden.eventsChannel1[0].satu=true;
  garden.eventsChannel1[0].sun=false;

  garden.eventsChannel1[1].state=false;
  garden.eventsChannel1[1].action=false;
  garden.eventsChannel1[1].hour=17;
  garden.eventsChannel1[1].min=47;
  garden.eventsChannel1[1].mon=false;
  garden.eventsChannel1[1].thurs=false;
  garden.eventsChannel1[1].wend=false;
  garden.eventsChannel1[1].tues=false;
  garden.eventsChannel1[1].fri=false;
  garden.eventsChannel1[1].satu=true;
  garden.eventsChannel1[1].sun=false;

  garden.eventsChannel1[2].state=false;
  garden.eventsChannel1[2].action=true;
  garden.eventsChannel1[2].hour=17;
  garden.eventsChannel1[2].min=48;
  garden.eventsChannel1[2].mon=false;
  garden.eventsChannel1[2].thurs=false;
  garden.eventsChannel1[2].wend=false;
  garden.eventsChannel1[2].tues=false;
  garden.eventsChannel1[2].fri=false;
  garden.eventsChannel1[2].satu=true;
  garden.eventsChannel1[2].sun=false;

  garden.eventsChannel1[3].state=false;
  garden.eventsChannel1[3].action=false;
  garden.eventsChannel1[3].hour=11;
  garden.eventsChannel1[3].min=54;
  garden.eventsChannel1[3].mon=false;
  garden.eventsChannel1[3].thurs=false;
  garden.eventsChannel1[3].wend=false;
  garden.eventsChannel1[3].tues=false;
  garden.eventsChannel1[3].fri=false;
  garden.eventsChannel1[3].satu=true;
  garden.eventsChannel1[3].sun=false;

  garden.eventsChannel1[4].state=false;
  garden.eventsChannel1[4].action=false;
  garden.eventsChannel1[4].hour=15;
  garden.eventsChannel1[4].min=15;
  garden.eventsChannel1[4].mon=false;
  garden.eventsChannel1[4].thurs=false;
  garden.eventsChannel1[4].wend=false;
  garden.eventsChannel1[4].tues=false;
  garden.eventsChannel1[4].fri=false;
  garden.eventsChannel1[4].satu=true;
  garden.eventsChannel1[4].sun=false;

  garden.eventsChannel1[5].state=false;
  garden.eventsChannel1[5].action=false;
  garden.eventsChannel1[5].hour=15;
  garden.eventsChannel1[5].min=15;
  garden.eventsChannel1[5].mon=false;
  garden.eventsChannel1[5].thurs=false;
  garden.eventsChannel1[5].wend=false;
  garden.eventsChannel1[5].tues=false;
  garden.eventsChannel1[5].fri=false;
  garden.eventsChannel1[5].satu=true;
  garden.eventsChannel1[5].sun=false;*/
  
  int numEvents = sizeof(garden.eventsChannel1) / sizeof(garden.eventsChannel1[0]);
  garden.sortEventsByTime(garden.eventsChannel1, numEvents);
  garden.printEventTimes(garden.eventsChannel1, numEvents);

}

void loop() {

    if (modoConfigBT){

        //Se apaga todos los canales por control (Se podria definir diferente)
        for (size_t i = 0; i < 4; i++){
          garden.channel.numberChannel = 1;
          garden.channel.state = false;
          garden.enableChannel(garden.channel);
        }
        SerialBT.begin("SmartGarden");     // Inicializamos la comunicación bluetooth serial.
        modoConfigFB=true;
        switch (STATE){
          case STATE_UID:{
            ledBlinkMillis(700);
            Serial.print("********** STATE: "); Serial.println(STATE);
            
            while(SerialBT.available()){  // Checamos si el buffer del bluetooth serial tiene datos   
                if(bandUID){
                    /* Si tiene datos entrara en el while */
                    if(bandBT==false){
                      UID = SerialBT.readString();            // Leemos únicamente 8bits y se almacena en “valor” 
                      delay(100);
                      bandBT = true;
                    }

                    espEeprom.putString("UID",UID);
                    delay(100);
                    bandUID = false;
                    espEeprom.putString("bandUID",UID);
                    delay(100);
                }      
                
                /* Enviamos el valor al monitor serial  */
                Serial.println(UID);  
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
                  SSID = SerialBT.readString();            // Leemos únicamente 8bits y se almacena en “valor” 
                  delay(150);
                  bandBT = true;
                }

                espEeprom.putString("SSID",SSID);
                delay(150);
                
                /* Enviamos el valor al monitor serial  */
                Serial.println(SSID);  
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
                    PASS = SerialBT.readString();            // Leemos el string y se almacena en “PASS” 
                    delay(150);
                    bandBT = true;
                  }

                  espEeprom.putString("PASS",PASS);
                  delay(150);
                  
                  /* Enviamos el valor al monitor serial  */
                  Serial.println(PASS);   
                  
                  STATE = STATE_WIFINIT;
            }
          break;
          }
          case STATE_WIFINIT:{

              ledBlinkMillis(100);
              if(SerialBT.disconnect()){
                  
                  Serial.print("********** STATE: "); Serial.println(STATE);
                  Serial.print("Nombre red: "); Serial.println(SSID);
                  Serial.print("Pass red: "); Serial.println(PASS);
                  Serial.print("userID: "); Serial.println(UID);

                  modoConfigBT = false;
                  espEeprom.putBool("bandBT",modoConfigBT);
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
        if (modoConfigFB){
              SerialBT.end(); //Se finaliza la comunicación Bluethoot

              //Se inicializa la configuración WiFi y se actualiza la hora desde la web 
              if(InitWiFi(SSID,PASS)){ //Se verifica que la conexión a internet es correcta

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
                  userPath = "/users/"+ UID;  //Se almacena el path personalizado del usuario, a este se le asocia el UID de Firebase
                  Serial.println("PATH DEFINIDO: "+userPath);
                  delay(1000);
                  
                  modoConfigFB=false;
                  
              } 
              
        }else{
            if (WiFi.status() == WL_CONNECTED){
            
                if(modoConfigUID){
                    Serial.print("********** INFORMACIÓN OBTENIDA DE FIREBASE **********");Serial.println();
                    

                    fbdoStreaming.keepAlive(5, 5, 1);
                    
                    if(!Firebase.beginStream(fbdoStreaming, userPath)){   
                        Serial.println("...No se establecio StreamCalback con el PATH: ");Serial.println(userPath);
                        delay(500);
                        /*Firebase.setStreamCallback(myFirebaseData, firebaseCallback, timeoutCallback);
                        delay(300);
                        Serial.println("******************************************************");Serial.println(); 
                        delay(200);*/
                    }/*else{
                      Serial.println("No se puede establecer conexión con la base de datos.");
                      CausaError();
                      delay(500);

                    }*/

                    Firebase.setStreamCallback(fbdoStreaming, firebaseCallback, timeoutCallback);
                    Serial.println("...Se establecio StreamCalback con el PATH: ");Serial.println(userPath);
                        delay(300);
              
                    digitalWrite(LED_ESP,HIGH); //Se enciende el led indicador para saber que el sistema esta listo para recibir ordenes de la APP
                    modoConfigUID = false;
                    //printMsg("AnalogEnable: ",analogPort.enable);
                } 
            
                if(flagTimer){
                    Serial.print("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());
                    stateChannel1 = garden.stateDefine(1,garden.eventsChannel1);  // la funcion stateDefine solo modifica al canal 1, se debe cambia REBISAR!!
                    stateChannel2 = garden.stateDefine(2,garden.eventsChannel2);
                    flagTimer = false;
                }

                tiempoActual = millis();
                if (tiempoActual - tiempoComp >= 30000){
                    tiempoComp = tiempoActual;
                    
                    float temperatureDHT = random(25,40);
                    //float  temperatureDHT = dht.readTemperature();
                    printMsg("Temperatura: ", temperatureDHT);
                    Firebase.set(fbdo, userPath + "/sensors/0",temperatureDHT);
                    delay(10);

                    float humidityDHT = random(25,40);
                    Firebase.set(fbdo, userPath+"/sensors/1",humidityDHT);
                    delay(10);

                    tempLm35 = readLm35();
                    Firebase.set(fbdo, userPath+"/sensors/3",tempLm35);
                    delay(10);
                }

                switch (state){
                    case 1:{
                      
                    break;}
                    case 2:{
                      
                    break;}               
                    default:
                    break;
                }
            }
        }
    }
}
