#ifndef DEVICEDRIVERS_ANALOG_MCP4728_H
#define DEVICEDRIVERS_ANALOG_MCP4728_H

#include <functional>
#include <stdint.h>

namespace DeviceDrivers {
namespace Analog {

namespace MCP4728 {
  constexpr uint8_t I2C_BASE_ADDR = 0x60;

  class MCP4728 {
    public:
      MCP4728(std::function<bool(uint8_t*, uint8_t, uint8_t*, uint8_t)> i2c_txn,
              std::function<void(bool)> write_ldac, std::function<bool()> read_rdy,
              uint8_t addr);

    private:
  }
}

}
}

#endif DEVICE_DRIVERS_ANALOG_MCP4728_H
