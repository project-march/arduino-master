#include <Arduino.h>
#include <ros.h>

namespace pins
{
}

ros::NodeHandle nh;

void setup()
{
  nh.initNode();
}

void loop()
{
  nh.spinOnce();
}
