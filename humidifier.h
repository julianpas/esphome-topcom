#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/climate/climate_mode.h"
#include "esphome/components/climate/climate_traits.h"
#include "esphome/components/custom/climate/custom_climate.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace topcom {

static const char *TAG = "topcom_1901w";

struct Status {
  bool standby;
  bool ion;
  bool heater;
  bool no_water;
  int level;
  int humidity;
  int temperature;
  int timer;
};
Status status;

namespace {
    
const int CS_PIN = 22;
const int CLK_PIN = 19;
const int DIO_PIN = 23;
const int CONTROL_PIN = 15;

int bit = 0;
uint32_t data;
uint16_t final_data[64];
uint32_t final_size = 0;
bool reading = true;

// Each digit as combination of its segments split over two addresses.
// Here is the map of the 7 segments (byte,bit)
//      1,1 
//     +---+
// 0,4 |0,3| 1,0
//     +---+ 
// 0,5 |   | 0,7
//     +---+
//      0,6
static constexpr byte dig[10][2] = {
  {0xf0, 0x03}, // b11110000 b00000011
  {0x80, 0x01}, // b10000000 b00000001
  {0x68, 0x03}, // b01101000 b00000011
  {0xc8, 0x03}, // b11001000 b00000011
  {0x98, 0x01}, // b10011000 b00000001
  {0xd8, 0x02}, // b11011000 b00000010
  {0xf8, 0x02}, // b11111000 b00000010
  {0x80, 0x03}, // b10000000 b00000011
  {0xf8, 0x03}, // b11111000 b00000011
  {0xd8, 0x03}  // b11011000 b00000011
};

// Decodes memory which has the following contents:
// 0x00 : thermomether in bits 0,1; 1st byte of temp. 1st digit.
// 0x01 : 2nd byte of temp. 1st digit.
// 0x02 : clock in bits 0,1; C in bit 2; 1st byte of temp. 2nd digit.
// 0x03 : 2nd byte of temp. 2nd digit.
// 0x04 : standby in bit 0; 1st byte of time 1st digit.
// 0x05 : 2nd byte of time 1st digit.
// 0x06 : ion in bit 0; H in bit 2; 1st byte of time 2nd digit.
// 0x07 : 2nd byte of time 2nd digit.
// 0x08 : water in bit 0;RH% in bit 2; 1st byte of hum. 1st digit.
// 0x09 : 2nd byte of hum. 1st digit.
// 0x0A : heat in bit 0; 1st byte of hum. 2nd digit.
// 0x0B : 2nd byte of hum. 2nd digit.
// 0x0C : mid in bit 6; high in bit 4;
// 0x0D : none
// The location of the low symbol is missing from that chart.
void IRAM_ATTR DecodeMemory(byte* buf, struct Status* status) {
  status->temperature = 0;
  status->timer = 0;
  status->humidity = 0;
  for (int i = 0; i < 10; i++) {
    if ((buf[0] & 0xf8) == dig[i][0] && (buf[2] & 0x03) == dig[i][1])
      status->temperature += 10 * i;
    if ((buf[4] & 0xf8) == dig[i][0] && (buf[6] & 0x03) == dig[i][1])
      status->temperature += i;

    if ((buf[8] & 0xf8) == dig[i][0] && (buf[10] & 0x03) == dig[i][1])
      status->timer += 10 * i;
    if ((buf[12] & 0xf8) == dig[i][0] && (buf[14] & 0x03) == dig[i][1])
      status->timer += i;
    
    if ((buf[16] & 0xf8) == dig[i][0] && (buf[18] & 0x03) == dig[i][1])
      status->humidity += 10 * i;
    if ((buf[20] & 0xf8) == dig[i][0] && (buf[22] & 0x03) == dig[i][1])
      status->humidity += i;
  }

  status->standby = buf[8] & 0x01 != 0;
  status->ion = buf[12] & 0x01 != 0;
  status->heater = buf[20] & 0x01 != 0;
  status->no_water = buf[16] & 0x01 != 0;
  status->level = ((buf[24] & 0x10) ? 3 : 0) + ((buf[24] & 0x40) ? 2 : 0);
  if (status->level == 0)
    status->level = 1;  
}

// Interrupt for end of command transmition when CS line resets.
void IRAM_ATTR isr() {
  if (bit == 8) {
    // There are 3 8-bit commands:
    //   CMD1 (0x0X) - This device only ever sends 0x03 (7x11 mode)
    //   CMD2 (0x4X) - This device only ever sends 0x44 (single-address writes)
    //   CMD4 (0x8X) - This device sends either 0x88 (1/16 brithness in idle
    //                 mode) or 0x8A (4/16 brightness when changing settings)
    if ((data & 0x0080) == 0x0080)
      // Only respect the readings when the display is in idle mode.
      reading = (data == 0x0088);
  }
  if (!reading) {
    bit = 0;
    data = 0;    
    return;  
  }
  if (bit == 16) {
    // The only type of 16-bit command is 
    //  CMD3 (0xCX 0xYY) - Set the segment data in memory address X. 
    //                     0xD is the last of 14 addresses. So if we have 14
    //                     writes in a row we have correctly received all bytes.
    final_data[final_size++] = data;
    if ((data & 0x00CD) == 0x00CD || final_size > 32) {
      reading = false;  
      if (final_size == 14)
        DecodeMemory((byte*)final_data + 1, &status);
      final_size = 0;
      reading = true;
    }
  }
  bit = 0;
  data = 0;
}

// Interrupt for gathering individual bits from the DIO line on CLK pulses.
void IRAM_ATTR isr2() {
  if (GPIO_REG_READ(GPIO_IN_REG) & (1 << DIO_PIN))
    bitSet(data, bit);
  bit++;
}

}  // namespace

static constexpr uint8_t CMD_POWER = 0xA1;
static constexpr uint8_t CMD_HUMIDITY = 0xA2;
static constexpr uint8_t CMD_TIMER = 0xA3;
static constexpr uint8_t CMD_ION = 0xA4;
static constexpr uint8_t CMD_LEVEL = 0xA5;
static constexpr uint8_t CMD_HEATING = 0xA6;

// Sends a command as if pressed on the front panel.
// Every command is a series of 8 pulses of length 6ms.
// 0 is encoded as 2ms low + 4ms high
// 1 is encoded as 4ms low + 2ms high
// The command ends with a 2ms pulse.
void SendCommand(const uint8_t command) {
  // Only set to output durring commands to avoid blocking the keys on device.
  pinMode(CONTROL_PIN, OUTPUT);
  delay(6);
  for(int i = 7; i >= 0;i--) {
    digitalWrite(CONTROL_PIN, LOW);
    delay((command & (1 << i)) == 0 ? 2 : 4);
    digitalWrite(CONTROL_PIN, HIGH);
    delay((command & (1 << i)) == 0 ? 4 : 2);
  }
  digitalWrite(CONTROL_PIN, LOW);
  delay(2);
  digitalWrite(CONTROL_PIN, HIGH);
  delay(4);
  pinMode(CONTROL_PIN, INPUT);
}

void GpioSetup() {
  pinMode(CS_PIN, INPUT_PULLUP);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DIO_PIN, INPUT);
  pinMode(CONTROL_PIN, INPUT);
  attachInterrupt(CS_PIN, isr, RISING);
  attachInterrupt(CLK_PIN, isr2, RISING);
}

class IonSwitch;
class HeatSwitch;
class TimerButton;
class HumidityButton;

class TopcomHumidfierComponent : public climate::Climate, PollingComponent {
 public:

  TopcomHumidfierComponent() : Climate(TAG), PollingComponent(5000) {}

  /// Return the traits of this controller.
  climate::ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({climate::CLIMATE_MODE_FAN_ONLY});
    traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
      climate::CLIMATE_FAN_OFF,
    }); 
    return traits;
  }

  /// Override control to change settings of the climate device.
  void control(const climate::ClimateCall &call) override {
    if (call.get_fan_mode().has_value()) {
      int level;
      fan_mode = *call.get_fan_mode();
      switch (fan_mode.value()) {
        case climate::CLIMATE_FAN_HIGH:
          level = 0x03;
          break;
        case climate::CLIMATE_FAN_MEDIUM:
          level = 0x02;
          break;
        case climate::CLIMATE_FAN_LOW:
          level = 0x01;
          break;
        case climate::CLIMATE_FAN_OFF:
          level = 0x0;
          break;
        case climate::CLIMATE_FAN_AUTO:
        case climate::CLIMATE_FAN_FOCUS:
        case climate::CLIMATE_FAN_ON:
        case climate::CLIMATE_FAN_MIDDLE:
        case climate::CLIMATE_FAN_DIFFUSE:
        default:
          level = -1;
          break;
      }

      if (level >= 0) {
        set_level(level);
      }

    }

    publish_state();
    ESP_LOGV(TAG, "State.", status.standby);      
  }

  void dump_config() override {
    uint8_t *p;
    ESP_LOGCONFIG(TAG, "Topcom 1901W Humidifer:");
    LOG_UPDATE_INTERVAL(this);
  }
  
  void update() override {
    is_water_present->publish_state(!status.no_water);
    if (heat_button != nullptr)
      ((switch_::Switch*)heat_button)->publish_state(status.heater);
    if (ion_button != nullptr)
      ((switch_::Switch*)ion_button)->publish_state(status.ion);
    air_temperature->publish_state(status.temperature);
    air_humdity->publish_state(status.humidity);
    timer_rest_time->publish_state(status.timer);
    if (status.standby) {
      mode = climate::CLIMATE_MODE_OFF;
      fan_mode = climate::CLIMATE_FAN_OFF;
    } else {
      mode = climate::CLIMATE_MODE_FAN_ONLY;
      switch(status.level) {
        case 1:
          fan_mode = climate::CLIMATE_FAN_LOW;
          break;
        case 2:
          fan_mode = climate::CLIMATE_FAN_MEDIUM;
          break;
        case 3:
          fan_mode = climate::CLIMATE_FAN_HIGH;
          break;
      }
    }
    current_temperature = status.temperature;
    publish_state();
  }
  
  void setup() override {
    ESP_LOGV(TAG, "Beginning GPIO setup.", data_index_);      
    GpioSetup();
    ESP_LOGV(TAG, "GPIO Setup done.", data_index_);
  }

  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_ion_state(bool state) {
    if (status.ion != state)
      SendCommand(CMD_ION);        
  }

  void set_heat_state(bool state) {
    if (status.heater != state)
      SendCommand(CMD_HEATING);        
  }
  
  void set_timer() {
    SendCommand(CMD_TIMER);
  }

  void set_humidity() {
    SendCommand(CMD_HUMIDITY);
  }

  void set_level(int level) {
    // If the device standby mode is not aligned with desired one send a power
    // command first.
    if (status.standby != (level == 0))
      SendCommand(CMD_POWER);
    if (level > 0 && level != status.level) {
      int cur_level = status.level;
      // Send as many level commands as needed to align modes. With a second
      // between commands.
      while (level != cur_level) {
        delay(1000);  
        SendCommand(CMD_LEVEL);
        level = (level + 1 > 3) ? 1 : level + 1;
      }
    }
  }

public:
  binary_sensor::BinarySensor *is_water_present{nullptr};
  sensor::Sensor *air_temperature{nullptr};
  sensor::Sensor *air_humdity{nullptr};
  sensor::Sensor *timer_rest_time{nullptr};
  IonSwitch *ion_button{nullptr};
  HeatSwitch *heat_button{nullptr};
  TimerButton *timer_button{nullptr};
  HumidityButton *humidity_button{nullptr};
};

class IonSwitch : public Switch {
 public:
  IonSwitch(const std::string& name, TopcomHumidfierComponent* humdifier)
    : switch_::Switch(name),
      humidifier_(humdifier) {}
 protected: 
  void write_state(bool state) override {
    humidifier_->set_ion_state(state);
    publish_state(state);
  }
 private:
  TopcomHumidfierComponent* humidifier_;  
};

class HeatSwitch : public Switch {
 public:
  HeatSwitch(const std::string& name, TopcomHumidfierComponent* humdifier)
    : switch_::Switch(name),
      humidifier_(humdifier) {}
 protected: 
  void write_state(bool state) override {
    humidifier_->set_heat_state(state);
    publish_state(state);
  }
 private:
  TopcomHumidfierComponent* humidifier_;  
};

class TimerButton : public button::Button {
  public:
    TimerButton(const std::string& name, TopcomHumidfierComponent* humdifier)
      : button::Button(name),
        humidifier_(humdifier) {}
    
  protected:
    void press_action() override {
      humidifier_->set_timer();
    }
  private:
    TopcomHumidfierComponent* humidifier_;  
};

class HumidityButton : public button::Button {
  public:
    HumidityButton(const std::string& name, TopcomHumidfierComponent* humdifier)
      : button::Button(name), 
        humidifier_(humdifier) {}
    
  protected:
    void press_action() override {
      humidifier_->set_humidity();
    }
  private:
    TopcomHumidfierComponent* humidifier_;  
};

}  // namespace topcom
}  // namespace esphome
