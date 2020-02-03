#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 512, 512  
  typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif