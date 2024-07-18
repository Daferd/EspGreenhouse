#include <ChannelCtrl.h> 

ChannelCtrl::ChannelCtrl(){}

ChannelCtrl::~ChannelCtrl(){}

void ChannelCtrl::chInit(uint8_t pin, String name){
    chPin = pin;
    chName = name;
    pinMode(chPin,OUTPUT);
}

void ChannelCtrl::chOnHw(void){
    digitalWrite(chPin, HIGH);
}

void ChannelCtrl::chOffHw(void){
    digitalWrite(chPin, LOW);
}