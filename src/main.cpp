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


  timer1 = timerBegin(1,80,true); //se escogio un preescalador de 80, es decir se divide la frecuencia normal de operación(80MHz), entre 80(preescalador), quedando frecuencia para el contador del timer de 10MHZ(periodo 1 microsegundo)
  timerAttachInterrupt(timer1, &onTimer1, true); //Esta función recibe como entrada un puntero al temporizador inicializado, que almacenamos en nuestra variable global, la dirección de la función que manejará la interrupción y una bandera que indica si la interrupción que se va a generar es de flanco (verdadero) o de nivel (falso)
  timerAlarmWrite(timer1,60000000,true);  // Con un valor de 23000 garantizamos que la 
  timerAlarmEnable(timer1);

  //unsigned char delayEprom = 100;
  loadEeprom();
  
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
                  espEeprom.putBool("modeConfigBT",modeConfigBT);
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
              if(InitWiFi(ssidWifi,passWifi)){ //Se verifica que la conexión a internet es correcta
                  initFirebase("daferd19@gmail.com","123456",userPath);
                  modeConfigFire = false;
                  digitalWrite(LED_ESP,HIGH);
              } 
              
        }else{
            if (WiFi.status() == WL_CONNECTED){
                 
                    if(flagTimer){
                        Serial.print("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());
                        
                        uint8_t ch = 1;
                        garden.stateDefine(ch,garden.eventsChannel1);  // la funcion stateDefine solo modifica al canal 1, se debe cambia REBISAR!!
                        printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                        Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                        
                        ch = 2;
                        bool stateChannel2 = garden.stateDefine(ch,garden.eventsChannel2);
                        printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                        Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                        
                        ch = 3;
                        garden.stateDefine(3,garden.eventsChannel3);
                        printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                        Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                        
                        ch=4;
                        garden.stateDefine(4,garden.eventsChannel4);
                        printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                        Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                        
                        flagTimer = false;
                    }
                    
                    if (millis() - tiempoComp >= SENSOR_READ_TIME_MS) 
                        readSensors();
                  
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