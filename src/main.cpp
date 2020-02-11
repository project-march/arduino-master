#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

namespace pins
{
const uint8_t RED_LED = 9;
const uint8_t GREEN_LED = 10;
const uint8_t BLUE_LED = 11;
const uint8_t E_BUTTON = 0;
}  // namespace pins

void setColorCallback(const std_msgs::ColorRGBA& color);
void writeColor(uint8_t red, uint8_t green, uint8_t blue);

// High voltage is enabled when HIGH and disabled when LOW
int hv_enabled = LOW;

ros::NodeHandle nh;

std_msgs::Bool button_msg;
ros::Publisher button_pub("/march/emergency_button/pressed", &button_msg);

ros::Subscriber<std_msgs::ColorRGBA> color_sub("/march/rgb_led/set_color", &setColorCallback);

void setColorCallback(const std_msgs::ColorRGBA& color)
{
  writeColor(color.r, color.g, color.b);
}

void writeColor(uint8_t red, uint8_t green, uint8_t blue)
{
  analogWrite(pins::RED_LED, 255 - red);
  analogWrite(pins::GREEN_LED, 255 - green);
  analogWrite(pins::BLUE_LED, 255 - blue);
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
  nh.advertise(button_pub);
  nh.subscribe(color_sub);
}

void loop()
{
  if (digitalRead(pins::E_BUTTON) != hv_enabled)
  {
    hv_enabled = !hv_enabled;
    button_msg.data = hv_enabled == HIGH;
    button_pub.publish(&button_msg);

    digitalWrite(LED_BUILTIN, hv_enabled);
  }

  nh.spinOnce();
}
