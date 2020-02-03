#include <Arduino.h>
#include <ros.h>

namespace pins
{
}

ros::NodeHandle nh;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  nh.initNode();
}

void loop()
{
  nh.spinOnce();
}
