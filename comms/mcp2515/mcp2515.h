#ifndef DEVICEDRIVERS_COMMS_MCP2515_H
#define DEVICEDRIVERS_COMMS_MCP2515_H

#include <functional>
#include <optional>
#include <stdint.h>

namespace DeviceDrivers {
namespace CAN {

enum IDType {
  StandardID,
  ExtendedID,
};

// TODO: move this definition out of this file
// TODO: add FD support
/// @brief Basic (non-FD) CAN frame class
template <enum IDType id_type>
class Frame {
  public:
    uint32_t id;
    bool remote;
    std::array<uint8_t, 8> data;
    uint8_t data_len;

    // How do I change the type based on id_type?
    Frame(uint32_t id, bool remote, uint8_t* data, uint8_t data_len);
};

namespace MCP2515 {
  // Register declarations
  constexpr uint8_t TXB0CTRL = 0x30;
  constexpr uint8_t TXB1CTRL = 0x30;
  constexpr uint8_t TXB2CTRL = 0x30;

  enum SPICommands : uint8_t {
    RESET = 0xC0,
    READ = 0x03,
    READ_RX_BUFFER = 0x90, // Must be bit-masked
    WRITE = 0x02,
    LOAD_TX_BUFFER = 0x40, // Must be bit-masked
    RTS = 0x80, // Must be bit-masked
    READ_STATUS = 0xA0,
    RX_STATUS = 0xB0,
    BIT_MODIFY = 0x05,
  };

  // OR these with LOAD_TX_BUFFER instruction to select a TX buffer
  constexpr uint8_t TX_BUF_TXB0SIDH = 0b000;
  constexpr uint8_t TX_BUF_TXB0D0 = 0b001;
  constexpr uint8_t TX_BUF_TXB1SIDH = 0b010;
  constexpr uint8_t TX_BUF_TXB1D0 = 0b011;
  constexpr uint8_t TX_BUF_TXB2SIDH = 0b100;
  constexpr uint8_t TX_BUF_TXB2D0 = 0b101;

  // possible transmit errors
  enum TxError : uint8_t {
    TX_ERR_OK,  // No error
    TX_ERR_ARB, // Arbitration error
    TX_ERR_BUS, // Bus error
    TX_ERR_FULL, // Buffers are full
  };

  enum RxBuffer {
    RXB0,
    RXB1
  };

  enum ReadStatusFlags {
    RX0IF = (1 << 0),
    RX1IF = (1 << 1),
    TX0REQ = (1 << 2),
    TX0IF = (1 << 3),
    TX1REQ = (1 << 4),
    TX1IF = (1 << 5),
    TX2REQ = (1 << 6),
    TX2IF = (1 << 7),
  };

  class MCP2515 {
    public:
      MCP2515(std::function<void()> spi_assert, std::function<void()> spi_deassert,
              std::function<void (uint8_t*, uint8_t)> spi_tx,
              std::function<void (uint8_t*, uint8_t)> spi_rx,
              std::function<void (uint8_t, uint8_t*, uint8_t*)> spi_transfer);

      // Resets via SPI
      void reset();

      // returns 0 if buffer is full
      template <enum IDType id_type>
      bool enque_frame(Frame<id_type> out_frame);
      // Blocking call to enque_frame
      template <enum IDType id_type>
      enum TxError send_frame(Frame<id_type> out_frame);

      template <enum IDType id_type>
      std::optional<Frame<id_type>> receive_frame();
      template <enum IDType id_type>
      std::optional<Frame<id_type>> receive_frame(RxBuffer);

      void configure_interrupts();
      // Reads status registers to determine interrupts and acknowledges it.
      void decode_interrupt();

      template <enum RxBuffer buf>
      void configure_mask(uint32_t mask);
      template <enum RxBuffer buf>
      void configure_filter(uint32_t filter);

    private:
      std::function<void()> spi_assert;
      std::function<void()> spi_deassert;
      std::function<void (uint8_t*, uint8_t)> spi_tx;
      std::function<void (uint8_t*, uint8_t)> spi_rx;
      std::function<void (uint8_t, uint8_t*, uint8_t*)> spi_transfer;

      /// @brief returns bit mask to OR with READ_TX_BUFFER SPI command
      constexpr uint8_t use_rxbuf1(bool n) { return n & (1 << 2); }
      /// @brief returns bit mask to OR with READ_TX_BUFFER SPI command
      constexpr uint8_t rx_start_at_data(bool m) { return m & (1 << 2); }

      uint8_t read_status();
      void rts(uint8_t buf);
      uint8_t read_reg(uint8_t reg);
      void write_reg(uint8_t reg, uint8_t value);

      template <enum IDType id_type>
      constexpr void write_buf_id(Frame<id_type> &frame, uint8_t* id_buf);
  };

} // Namespace MCP2515

} // Namespace CAN
} // Namespace DeviceDrivers

#endif // DEVICEDRIVERS_COMMS_MCP2515_H