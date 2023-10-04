#ifndef _SMARTGREENHOUSE_H
#define _SMARTGREENHOUSE_H

#include <Arduino.h>
#include <ESP32Time.h>

 



    //Variable y Estructuras para el control del Timer


class SmartGreenhouse {



public:
    //Variable y Estructuras para el control del Timer
    struct Event {
        uint8_t min;
        uint8_t hour;
        bool sun;
        bool mon;
        bool thurs;
        bool wend;
        bool tues;
        bool fri;
        bool satu;
        //bool listDays[7];
        bool state;
        bool action;
    };

    struct ChannelServer {
        uint8_t numberChannel;
        uint8_t state;
    };

    ChannelServer channel;
    Event event;
   
   
    ESP32Time rtc;

    float sensorValue[4];

    SmartGreenhouse(/* args */);
    ~SmartGreenhouse();

    void  enableChannel(ChannelServer channel);
    bool  stateDefine(Event events[]);
    float readDigitalSensor(uint8_t sensorPin);

private:
    bool timerAction(Event eventAux);
    void sortEventsByTime(Event events[], int size);
    void swapEvents(Event &event1, Event &event2);
};




#endif //_ANALOGSENSOR_H