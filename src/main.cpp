#include "color.h"

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

#include <march_shared_resources/Error.h>
#include <march_shared_resources/GaitInstructionResponse.h>

namespace pins
{
const uint8_t RGB_LED_OUTPUT = 8;
const uint8_t RED_LED = 9;
const uint8_t GREEN_LED = 10;
const uint8_t BLUE_LED = 11;
const uint8_t E_BUTTON = 0;
}  // namespace pins

namespace colors
{
const Color OFF(0, 0, 0);
const Color ERROR(255, 0, 0);
const Color IDLE(0, 0, 255);
const Color ACTIVE(0, 255, 0);
}  // namespace colors

void setColorCallback(const std_msgs::ColorRGBA& color);
void errorCallback(const march_shared_resources::Error& error);
void instructionResponseCallback(const march_shared_resources::GaitInstructionResponse& response);
void currentGaitCallback(const std_msgs::String& gait);
void writeColor(Color color);

// High voltage is enabled when HIGH and disabled when LOW
int hv_enabled = LOW;
// True when a gait is being executed, false otherwise
bool active = false;

ros::NodeHandle nh;

std_msgs::Bool button_msg;
ros::Publisher button_pub("/march/emergency_button/pressed", &button_msg);

ros::Subscriber<std_msgs::ColorRGBA> color_sub("/march/rgb_led/set_color", &setColorCallback);
ros::Subscriber<march_shared_resources::Error> error_sub("/march/error", &errorCallback);
ros::Subscriber<march_shared_resources::GaitInstructionResponse> response_sub("/march/input_device/"
                                                                              "instruction_response",
                                                                              &instructionResponseCallback);
ros::Subscriber<std_msgs::String> current_gait_sub("/march/gait/current", &currentGaitCallback);

void setColorCallback(const std_msgs::ColorRGBA& color)
{
  writeColor(Color(color.r, color.g, color.b));
}

void errorCallback(const march_shared_resources::Error& /* error */)
{
  active = false;
  writeColor(colors::ERROR);
}

void instructionResponseCallback(const march_shared_resources::GaitInstructionResponse& response)
{
  if (response.result == response.GAIT_FINISHED)
  {
    active = false;
    writeColor(colors::IDLE);
  }
}

void currentGaitCallback(const std_msgs::String& /* gait */)
{
  active = true;
  writeColor(colors::ACTIVE);
}

void writeColor(Color color)
{
  analogWrite(pins::RED_LED, 255 - color.r);
  analogWrite(pins::GREEN_LED, 255 - color.g);
  analogWrite(pins::BLUE_LED, 255 - color.b);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pins::RGB_LED_OUTPUT, OUTPUT);
  pinMode(pins::RED_LED, OUTPUT);
  pinMode(pins::GREEN_LED, OUTPUT);
  pinMode(pins::BLUE_LED, OUTPUT);
  pinMode(pins::E_BUTTON, INPUT);

  digitalWrite(pins::RGB_LED_OUTPUT, HIGH);

  hv_enabled = digitalRead(pins::E_BUTTON);
  digitalWrite(LED_BUILTIN, hv_enabled);

  writeColor(colors::OFF);

  nh.initNode();
  nh.advertise(button_pub);
  nh.subscribe(color_sub);
  nh.subscribe(error_sub);
  nh.subscribe(response_sub);
  nh.subscribe(current_gait_sub);
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

  if (!nh.connected())
  {
    writeColor(colors::OFF);
  }
  else if (!active)
  {
    writeColor(colors::IDLE);
  }
  nh.spinOnce();
}
