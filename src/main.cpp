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

  digitalWrite(garden.CH1,LOW);
  digitalWrite(garden.CH2,LOW);
  digitalWrite(garden.CH3,LOW);
  digitalWrite(garden.CH4,LOW);
  digitalWrite(garden.CH5,LOW);
  digitalWrite(garden.CH6,LOW);
  //digitalWrite(garden.CH7,!LOW);
  digitalWrite(garden.CH8,LOW);

  //unsigned char delayEprom = 100;
  loadEeprom();
  
  //HACE FALTA VERIFICAR QUE RETOME LOS ESTADOS QUE TENIA ANTES DEL REINICIO
  garden.channel.numberChannel = 1;
  garden.channel.state = garden.enableChFlag[0];
  garden.enableChannel(garden.channel);
  delay(1000); //Se pausa para que los actuadores no enciendan todos a la vez, si no de uno en uno

  garden.channel.numberChannel = 2;
  garden.channel.state = garden.enableChFlag[1];
  garden.enableChannel(garden.channel);
  delay(1000); //Se pausa para que los actuadores no enciendan todos a la vez, si no de uno en uno

  garden.channel.numberChannel = 3;
  garden.channel.state = garden.enableChFlag[2];
  garden.enableChannel(garden.channel);
  delay(1000); //Se pausa para que los actuadores no enciendan todos a la vez, si no de uno en uno

  garden.channel.numberChannel = 4;
  garden.channel.state = garden.enableChFlag[3];
  garden.enableChannel(garden.channel);
  delay(1000); //Se pausa para que los actuadores no enciendan todos a la vez, si no de uno en uno


  modeConfigFire = true;
  //modoConfigUID = true;

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
                 
                if (rtc.getSecond()==0 && flagShedulesStart==true){
                    flagShedulesStart = false;
                    // SE DEBE CORREGIR EL TEMA DE LA INSTRUCCIÓN FIREBASE.SET() QUE SE EJECUTA SIEMPRE EXISTA O NO EL TIMER HABILITADO
                    // IMLICA QUE CREA EL TIMER EN FIREBASE Y ESTO HACE QUE SE VEA EN LA APP, CUANDO NO DEBERIA SER ASI
                    // SE SUGIERE SACAR EL CONTROL DE HORARIOS DE LA CLASE SMARTGREENHOUSE PARA USARLA CON FIREBASE
                    Serial.println("Hora actualizada: "); Serial.print(rtc.getTime()); Serial.print(", Dia: "); Serial.println(rtc.getDay());
                    
                    //readSensors();

                    uint8_t ch = 1;
                    bool stateCh1 = garden.stateDefine(ch,garden.eventsChannel1,garden.enableChFlag[ch-1]);  // la funcion stateDefine solo modifica al canal 1, se debe cambia REBISAR!!
                    digitalWrite(garden.ch[ch-1],stateCh1);
                    garden.enableChFlag[ch-1] = stateCh1;
                    printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                    Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                    
                    ch = 2;
                    bool stateCh2 = garden.stateDefine(ch,garden.eventsChannel2,garden.enableChFlag[ch-1]);
                    digitalWrite(garden.ch[ch-1],stateCh2);
                    garden.enableChFlag[ch-1] = stateCh2;
                    printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                    Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                    
                    ch = 3;
                    bool stateCh3=garden.stateDefine(ch,garden.eventsChannel3,garden.enableChFlag[ch-1]);
                    digitalWrite(garden.ch[ch-1],stateCh3);
                    garden.enableChFlag[ch-1] = stateCh3;
                    printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                    Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                    
                    ch=4;
                    bool stateCh4=garden.stateDefine(ch,garden.eventsChannel4,garden.enableChFlag[ch-1]);
                    digitalWrite(garden.ch[ch-1],stateCh4);
                    garden.enableChFlag[ch-1] = stateCh4;
                    printMsg("ESTADO CANAL "+String(ch)+": ", garden.enableChFlag[ch-1]);
                    Firebase.set(fbdo, userPath + "/channels/channel"+String(ch)+"/state",garden.enableChFlag[ch-1]);
                }
                
                if(rtc.getSecond()==30) flagShedulesStart = true;

                    
                //if (millis() - tiempoComp >= SENSOR_READ_TIME_MS) readSensors();
                  
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