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

  constexpr uint8_t CNF3 = 0x28;
  constexpr uint8_t CNF2 = 0x29;
  constexpr uint8_t CNF1 = 0x2A;

  constexpr uint8_t CANINTF = 0x2C;
  constexpr uint8_t CANSTAT = 0xE0;
  constexpr uint8_t CANCTRL = 0xF0;

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

  enum OperationalMode : uint8_t {
    ConfigurationMode = 0,
    NormalMode,
    SleepMode,
    ListenOnlyMode,
    LoopBackMode,
  };

  // OR these with READ_RX_BUFFER to select an RX buffer
  constexpr uint8_t RX_BUF_RXB0SIDH = 0x00;
  constexpr uint8_t RX_BUF_RXB0DO = 0x02;
  constexpr uint8_t RX_BUF_RXB1SIDH = 0x04;
  constexpr uint8_t RX_BUF_RXB1DO = 0x06;

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
    RXB0 = 0,
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

      void reset();
      void enter_mode(enum OperationalMode mode, bool verify);
      void set_baudrate(uint8_t brp, uint8_t sync, uint8_t prop,
                        uint8_t ps1, uint8_t ps2, uint8_t sjw);

      template <enum IDType id_type>
      bool enque_frame(Frame<id_type> out_frame);
      template <enum IDType id_type>
      enum TxError send_frame(Frame<id_type> out_frame);

      template <enum IDType id_type>
      std::optional<Frame<id_type>> receive_frame();
      template <enum IDType id_type>
      std::optional<Frame<id_type>> receive_frame(RxBuffer buffer);

      void configure_interrupts();
      // Reads status registers to determine interrupts and acknowledges it.
      void decode_interrupt();

      void configure_mask(uint32_t mask);
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
      uint8_t rx_status();
      void rts(uint8_t buf);
      uint8_t read_reg(uint8_t reg);
      void write_reg(uint8_t reg, uint8_t value);
      void read(uint8_t start_addr, uint8_t len, uint8_t* buffer);
      void write(uint8_t start_addr, uint8_t len, uint8_t* buffer);

      template <enum IDType id_type>
      constexpr void write_buf_id(Frame<id_type> &frame, uint8_t* id_buf);
  };

} // Namespace MCP2515

} // Namespace CAN
} // Namespace DeviceDrivers

#endif // DEVICEDRIVERS_COMMS_MCP2515_H