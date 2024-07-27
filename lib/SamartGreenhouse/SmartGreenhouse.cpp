#include <SmartGreenhouse.h>

#define CH1 25
#define CH2 13
#define CH3 12
#define CH4 14
#define CH5 27
#define CH6 26
#define CH7 2
#define CH8 4

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

void SmartGreenhouse::enableInvChannel(ChannelServer channel){
    digitalWrite(ch[channel.numberChannel-1],channel.state);
}

void SmartGreenhouse::enableChannel(ChannelServer channel){
    digitalWrite(ch[channel.numberChannel-1],channel.state);
}

bool SmartGreenhouse::timerAction(Event eventAux, bool stateCurrent){
  bool enableAction=stateCurrent; 
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
            enableAction = eventAux.action; 
        }
      }else if(rtc.getHour(true) > eventAux.hour){
        enableAction = eventAux.action;
      }
  }
  return enableAction;
}

bool SmartGreenhouse::stateDefine(int chn, Event events[],bool currentState){
    bool stateDef = currentState;

    for (int i = 0; i < 4; i++) {
        if (events[i].state){                              
            stateDef = timerAction(events[i],stateDef);
        }
    }
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

void SmartGreenhouse::printEventTimes(Event events[], int size) {
    //int numEvents = size;
    //sortEventsByTime(events, numEvents); //Se organizan de menor a mayor respecto a la hora del dia
    Serial.println("Event Times:");
    for (int i = 0; i < size; i++) {
        Serial.print("Event ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(events[i].hour);
        Serial.print(":");
        if (events[i].min < 10) {
            Serial.print("0");
        }
        Serial.print(events[i].min);
        Serial.print(", State: "); 
        Serial.print(events[i].state); 
        Serial.print(", Action: "); 
        Serial.print(events[i].action); 
        if(events[i].mon) Serial.print(", L: X"); 
        if(events[i].tues) Serial.print(", M: X"); 
        if(events[i].wend) Serial.print(", W: X"); 
        if(events[i].thurs) Serial.print(", J: X"); 
        if(events[i].fri) Serial.print(", F: X"); 
        if(events[i].satu) Serial.print(", S: X"); 
        if(events[i].sun) Serial.print(", D: X"); 

        Serial.println();
        
        

    }
}

bool SmartGreenhouse::dhtInit(uint8_t dhtPin){
    //dht.begin();
    return true;
}

/*float readDhtTemperature(){
    //return dht.readHumidity();
}
float readDhtHumidity(){
    //return dht.readTemperature();
}*/


