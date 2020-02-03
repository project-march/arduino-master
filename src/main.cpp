#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>

namespace pins
{
const uint8_t rgb_led = 5;
const uint8_t emergency_button = 0;
}  // namespace pins

ros::NodeHandle nh;

std_msgs::Bool button_msg;
ros::Publisher button("/march/emergency_button/pressed", &button_msg);

// High voltage is enabled when HIGH and disabled when LOW
int hv_enabled = LOW;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pins::rgb_led, OUTPUT);
  pinMode(pins::emergency_button, INPUT);

  hv_enabled = digitalRead(pins::emergency_button);
  digitalWrite(LED_BUILTIN, hv_enabled);

  nh.initNode();
  nh.advertise(button);
}

void loop()
{
  if (digitalRead(pins::emergency_button) != hv_enabled)
  {
    button_msg.data = hv_enabled == LOW;
    button.publish(&button_msg);

    hv_enabled = digitalRead(pins::emergency_button);
    digitalWrite(LED_BUILTIN, hv_enabled);
  }

  nh.spinOnce();
}
