#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#include <Arduino.h>

class ArduinoHardware
{
  public:
    ArduinoHardware()
    {
      iostream = &Serial;
      baud_ = 500000;
    }
  
    void setBaud(long baud)
    {
      this->baud_= baud;
    }
  
    int getBaud()
    {
      return baud_;
    }

    void init()
    {
      iostream->begin(baud_);
    }

    int read()
    {
      return iostream->read();
    };

    void write(uint8_t* data, int length)
    {
      for(int i=0; i<length; i++)
      {
        iostream->write(data[i]);
      }
    }

    unsigned long time()
    {
      return millis();
    }

  protected:
    Serial_* iostream;
    long baud_;
};

#endif
