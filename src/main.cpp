#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>

namespace pins
{
const uint8_t RED_LED = 9;
const uint8_t GREEN_LED = 10;
const uint8_t BLUE_LED = 11;
const uint8_t E_BUTTON = 0;
}  // namespace pins

ros::NodeHandle nh;

std_msgs::Bool button_msg;
ros::Publisher button("/march/emergency_button/pressed", &button_msg);

// High voltage is enabled when HIGH and disabled when LOW
int hv_enabled = LOW;

void writeColor(int red, int green, int blue)
{
  analogWrite(pins::RED_LED, red);
  analogWrite(pins::GREEN_LED, green);
  analogWrite(pins::BLUE_LED, blue);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pins::RED_LED, OUTPUT);
  pinMode(pins::GREEN_LED, OUTPUT);
  pinMode(pins::BLUE_LED, OUTPUT);
  pinMode(pins::E_BUTTON, INPUT);

  hv_enabled = digitalRead(pins::E_BUTTON);
  digitalWrite(LED_BUILTIN, hv_enabled);

  nh.initNode();
  nh.advertise(button);
}

void loop()
{
  if (digitalRead(pins::E_BUTTON) != hv_enabled)
  {
    hv_enabled = !hv_enabled;
    button_msg.data = hv_enabled == HIGH;
    button.publish(&button_msg);

    digitalWrite(LED_BUILTIN, hv_enabled);
  }

  nh.spinOnce();
}
