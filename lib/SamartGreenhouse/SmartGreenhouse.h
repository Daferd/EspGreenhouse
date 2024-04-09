#ifndef _SMARTGREENHOUSE_H
#define _SMARTGREENHOUSE_H

#include <Arduino.h>
#include <ESP32Time.h>



class SmartGreenhouse {



public:
    //Variable y Estructuras para el control del Timer
    const uint8_t CH1 = 25;
    const uint8_t CH2 = 13;
    const uint8_t CH3 = 12;
    const uint8_t CH4 = 14;
    const uint8_t CH5 = 27;
    const uint8_t CH6 = 26;
    const uint8_t CH7 = 2;
    const uint8_t CH8 = 4;

    bool enableChFlag[8] = {false,false,false,false,false,false,false,false};

    int ch[8] = {CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8};

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
        bool state;
    };

    ChannelServer channel;
    Event eventsChannel1[6];
    Event eventsChannel2[6];
    Event eventsChannel3[6];
    Event eventsChannel4[6];
    Event eventsChannel5[6];
    Event eventsChannel6[6];
   
    ESP32Time rtc;

    float sensorValue[4];

    SmartGreenhouse(/* args */);
    ~SmartGreenhouse();

    void  enableChannel(ChannelServer channel);
    bool  stateDefine(int channel,Event events[]);
    float readDigitalSensor(uint8_t sensorPin);
    bool dhtInit(uint8_t dhtPin);
    float readDhtTemperature();
    float readDhtHumidity();
    void printEventTimes(Event events[], int size);
    bool timerAction(Event eventAux);
    void sortEventsByTime(Event events[], int size);
    void swapEvents(Event &event1, Event &event2);
    

private:
    
};




#endif //_ANALOGSENSOR_H