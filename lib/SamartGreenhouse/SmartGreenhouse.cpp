#include <SmartGreenhouse.h>

SmartGreenhouse::SmartGreenhouse(/* args */)
{
}

SmartGreenhouse::~SmartGreenhouse()
{
}

float SmartGreenhouse::readDigitalSensor(uint8_t sensorPin){
    float sensorValue = digitalRead(sensorPin);
    return sensorValue;
    
}

void SmartGreenhouse::enableChannel(ChannelServer channel){
    digitalWrite(channel.numberChannel,channel.state);
}

bool SmartGreenhouse::timerAction(Event eventAux){
  bool action=false; 
  bool listDays[]={false,false,false,false,false,false,false};
  
  listDays[0] = eventAux.sun;
  listDays[1] = eventAux.mon;
  listDays[2] = eventAux.tues;
  listDays[3] = eventAux.wend;
  listDays[4] = eventAux.thurs;
  listDays[5] = eventAux.fri;
  listDays[6] = eventAux.satu;

  if(listDays[rtc.getDayofWeek()]){
      if(rtc.getHour(true) == eventAux.hour){
        if(rtc.getMinute() >= eventAux.min){
            action = true; 
        }else {
            action = false;
        }
      }else if(rtc.getHour(true) > eventAux.hour){
        action = true;
      }else{
        action = false;
      }
  }else{
    action = false;
  }
  return action;
}

bool SmartGreenhouse::stateDefine(Event events[]){
    bool stateDef = 1;

        for (int i = 0; i < 7; i++) {
          if (events[i].state){                              
              if(timerAction(events[i])){
                  //events[i].state = false;
                  if(events[i].action){
                      Serial.println("ENCENDIDO!!");
                      stateDef = true;
                    //fuente.encenderZona(bandLuces,zonaAux,colorR,colorG,colorB);
                  }else{
                      Serial.println("APAGADO!!");
                      stateDef = false;
                  }
              }
          }
        }
    /*}else{
        stateDef = true;
    }*/
    return stateDef;
}

void SmartGreenhouse::swapEvents(Event &event1, Event &event2) {
    Event temp = event1;
    event1 = event2;
    event2 = temp;
}

void SmartGreenhouse::sortEventsByTime(Event events[], int size) {
    for (int i = 0; i < size - 1; i++) {
        int minIndex = i;
        for (int j = i + 1; j < size; j++) {
            if (events[j].hour < events[minIndex].hour ||
                (events[j].hour == events[minIndex].hour && events[j].min < events[minIndex].min)) {
                minIndex = j;
            }
        }
        if (minIndex != i) {
            swapEvents(events[i], events[minIndex]);
        }
    }
}


