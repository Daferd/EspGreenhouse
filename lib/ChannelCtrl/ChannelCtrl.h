#ifndef _CHANNELCTRL_H
#define _CHANNELCTRL_H

#include <Arduino.h>

class ChannelCtrl{
private:
    String chName = "channel";
    
    
public:
    uint8_t chPin = 0;

    ChannelCtrl();
    ~ChannelCtrl();

    void chInit(uint8_t pin, String name);
    void chOnHw(void);
    void chOffHw(void);
};




#endif //_CHANNELCTRL_H