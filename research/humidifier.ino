#include <ESP32SPISlave.h>

ESP32SPISlave slave;

static constexpr uint32_t ESP_BUF_SIZE = 32;
uint8_t spi_tx_buf[ESP_BUF_SIZE];
uint8_t spi_rx_buf[ESP_BUF_SIZE];

static constexpr uint8_t CONTROL_PIN = 25;

enum ErrorCode { NONE = 0, BAD_PACKET = 1 };
ErrorCode error = NONE;

static constexpr uint8_t CMD_POWER = 0xA1;
static constexpr uint8_t CMD_HUMIDITY = 0xA2;
static constexpr uint8_t CMD_TIMER = 0xA3;
static constexpr uint8_t CMD_ION = 0xA4;
static constexpr uint8_t CMD_LEVEL = 0xA5;
static constexpr uint8_t CMD_HEATING = 0xA6;

static constexpr byte dig[10][2] = {
  {0xf0, 0x03},
  {0x80, 0x01},
  {0x68, 0x03},
  {0xc8, 0x03},
  {0x98, 0x01},
  {0xd8, 0x02},
  {0xf8, 0x02},
  {0x80, 0x03},
  {0xf8, 0x03},
  {0xd8, 0x03}
};

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
bool printOutput = false;

void DecodeMemory(const byte* buf, struct Status* status) {
  if (buf[0] != 0x44 || buf[2] != 0x03) {
    error = BAD_PACKET;
    return;
  }
  error = NONE;
  status->temperature = 0;
  status->timer = 0;
  status->humidity = 0;
  for (int i = 0; i < 10; i++) {
    if ((buf[4] & 0xf8) == dig[i][0] && (buf[6] & 0x03) == dig[i][1])
      status->temperature += 10 * i;
    if ((buf[8] & 0xf8) == dig[i][0] && (buf[10] & 0x03) == dig[i][1])
      status->temperature += i;

    if ((buf[12] & 0xf8) == dig[i][0] && (buf[14] & 0x03) == dig[i][1])
      status->timer += 10 * i;
    if ((buf[16] & 0xf8) == dig[i][0] && (buf[18] & 0x03) == dig[i][1])
      status->timer += i;
    
    if ((buf[20] & 0xf8) == dig[i][0] && (buf[22] & 0x03) == dig[i][1])
      status->humidity += 10 * i;
    if ((buf[24] & 0xf8) == dig[i][0] && (buf[26] & 0x03) == dig[i][1])
      status->humidity += i;
  }

  status->standby = buf[12] & 0x01 != 0;
  status->ion = buf[16] & 0x01 != 0;
  status->heater = buf[24] & 0x01 != 0;
  status->no_water = buf[20] & 0x01 != 0;
  status->level = ((buf[28] & 0x10) >> 4) + ((buf[28] & 0x40) >> 6);
  if (status->level == 0)
    status->level = 1;  
}


void SendCommand(const uint8_t command) {
  for(int i = 0; i < 8;i++) {
    if (command & (1 << i) == 0) {}
    digitalWrite(CONTROL_PIN, LOW);
    delay(command & (1 << i) == 0 ? 2 : 4);
    digitalWrite(CONTROL_PIN, HIGH);
    delay(command & (1 << i) == 0 ? 4 : 2);
  }
  digitalWrite(CONTROL_PIN, LOW);
  delay(2);
  digitalWrite(CONTROL_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.print("Let's go!");
  Serial.println();
  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
  // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
  slave.setDataMode(SPI_MODE3);
  slave.setSlaveFlags(SPI_SLAVE_BIT_LSBFIRST);
  slave.begin(HSPI);
  Serial.println("SPI Initialized");
}

void loop() {
  if (slave.remained() == 0)
    slave.queue(spi_rx_buf, spi_tx_buf, ESP_BUF_SIZE);

  while (slave.available()) {
    DecodeMemory(spi_rx_buf, &status);
    if (error == 0) {
      if (printOutput) {
        Serial.print(status.humidity);
        Serial.print(' ');
        Serial.print(status.temperature);
        Serial.print(' ');
        Serial.print(status.timer);
        Serial.print(' ');
        Serial.print(status.level);
        Serial.print(' ');
        Serial.print(status.standby ? '1' : '0');
        Serial.print(' ');
        Serial.print(status.ion ? '1' : '0');
        Serial.print(' ');
        Serial.print(status.heater ? '1' : '0');
        Serial.print(' ');
        Serial.print(status.no_water ? '1' : '0');
        Serial.println();
      }
    }
    slave.pop();
  }

  if (Serial.available() > 0) {
    int cmd = Serial.read();
    switch(cmd) {
      case '*':
        printOutput = !printOutput;
        break;
      case '0':
        SendCommand(CMD_POWER);
        break;
      case '1':
        SendCommand(CMD_HUMIDITY);
        break;
      case '2':
        SendCommand(CMD_TIMER);
        break;
      case '3':
        SendCommand(CMD_ION);
        break;
      case '4':
        SendCommand(CMD_LEVEL);
        break;
      case '5':
        SendCommand(CMD_HEATING);
        break;
      case '?':
        Serial.println("* - enable/disable output, 0 - PWR, 1 - HUM, 2 - TIM, 3 - ION, 4 - LEVEL, 5 - HEAT");
    }
    delay(1000);
  }
}
