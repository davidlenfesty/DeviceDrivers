#include "mcp2515.h"

namespace DeviceDrivers {
namespace CAN {

// TODO assert or something data_len < 8
// TODO manage remotes better
template <enum IDType id_type>
Frame<id_type>::Frame(uint32_t id, bool remote, uint8_t* data, uint8_t data_len) :
  id(id),
  remote(remote),
  data_len(data_len)
{
  for (uint8_t i = 0; i < data_len; ++i)
    this->data[i] = data[i];
}

// TODO implement filtering
namespace MCP2515 {
  /// @brief Initializes the class, and resets the device, entering configuration mode.
  MCP2515::MCP2515(std::function<void()> spi_assert, std::function<void()> spi_deassert,
                   std::function<void (uint8_t*, uint8_t)> spi_tx,
                   std::function<void (uint8_t*, uint8_t)> spi_rx,
                   std::function<void (uint8_t, uint8_t*, uint8_t*)> spi_transfer) :
    spi_assert(spi_assert),
    spi_deassert(spi_deassert),
    spi_tx(spi_tx),
    spi_rx(spi_rx),
    spi_transfer(spi_transfer)
  {
    reset();
  }

  /// @brief Resets the device, entering configuration mode
  void MCP2515::reset()
  {
    uint8_t ins = RESET;
    spi_assert();
    spi_tx(&ins, 1);
    spi_deassert();
  }

  /// @brief Sends a request for the device to enter an operational mode
  void MCP2515::enter_mode(enum OperationalMode mode, bool verify = false)
  {
    uint8_t canctrl = read_reg(CANCTRL);
    canctrl = (canctrl & ~0x07) | mode;
    write_reg(CANCTRL, canctrl);

    // Hold until mode is verified, if requested
    if (verify)
      while ((read_reg(CANSTAT) & ~0x07) != mode);
  }

  /// @brief Sets the required parameters to set a baud rate with the given clock frequency
  /// Refer to the datasheet, section 5.0 for how to set these constants.
  /// It's fairly application-dependant and I'm too lazy to come up with a
  /// clean formula for setting baudrate.
  void MCP2515::set_baudrate(uint8_t brp, uint8_t /*sync*/, uint8_t prop,
                             uint8_t ps1, uint8_t ps2, uint8_t sjw)
  {
    uint8_t CNF[3];
    // for some reason CNF1 - CNF3 is reversed...
    // TODO make WAKFIL it's own thing
    CNF[2] = sjw << 6 | brp;
    CNF[1] = ps1 << 3 | prop;
    CNF[0] = (1 << 6) | ps2;
    write(CNF3, 3, CNF);
  }

  /// @brief Write frame to first available TX buf and send RTS
  /// @retval 1   Success
  /// @retval 0   Buffers are full, frame not sent
  template <enum IDType id_type>
  bool MCP2515::enque_frame (Frame<id_type> out_frame)
  {
    std::array<uint8_t, 6> frame_buf; // 1 byte cmd, 4 bytes ID, 1 byte DLC
    uint8_t tx_mbox;
    uint8_t status;

    // Check if we have any buffers to use
    status = read_status();
    if (status ^ TX0REQ)
      tx_mbox = TX_BUF_TXB0SIDH;
    else if (status ^ TX1REQ)
      tx_mbox = TX_BUF_TXB1SIDH;
    else if (status ^ TX2REQ)
      tx_mbox = TX_BUF_TXB2SIDH;
    else
      return 0;

    // Instruction
    frame_buf[0] = LOAD_TX_BUFFER | tx_mbox;
    // ID will get written first
    write_buf_id<id_type>(&out_frame, &frame_buf[1]);

    // Begin transferring data
    spi_assert();
    spi_tx(frame_buf.data(), 6);
    // Transmit data only if not RTR frame
    if (!out_frame.remote) {
      spi_tx(out_frame.data.data(), out_frame.data_len);
    }
    spi_deassert();
    return 1;
  }

  /// @brief Blocking write frame to first available TX buf and send RTS
  /// @returns enum 
  template <enum IDType id_type>
  enum TxError MCP2515::send_frame(Frame<id_type> out_frame)
  {
    std::array<uint8_t, 6> frame_buf; // 1 byte cmd, 4 bytes Id, 1 byte DLC
    uint8_t tx_mbox;
    uint8_t status;

    // Check if we have free buffers
    status = read_status();
    if (status ^ TX0REQ)
      tx_mbox = 0;
    else if (status ^ TX1REQ)
      tx_mbox = 1;
    else if (status ^ TX2REQ)
      tx_mbox = 2;
    else
      return TX_ERR_FULL;

    frame_buf[0] = LOAD_TX_BUFFER | (TX_BUF_TXB0SIDH + 2 * tx_mbox);
    write_buf_id(&out_frame, &frame_buf[1]);

    // Begin transferring data
    spi_assert();
    spi_tx(frame_buf.data(), 6);
    // Transmit data only if not RTR frame
    if (!out_frame.remote) {
      spi_tx(out_frame.data.data(), out_frame.data_len);
    }
    spi_deassert();

    // Wait for mailbox to finish sending
    do {
      status = read_status();
    } while (status ^ (2 + 2 * tx_mbox));

    // Check errors
    status = read_reg(TXB0CTRL + 0x10 * tx_mbox);
    if (status & (1 << 5))
      return TX_ERR_ARB;
    if (status & (1 << 4))
      return TX_ERR_BUS;
    
    return TX_ERR_OK;
  }

  template <enum IDType id_type>
  std::optional<Frame<id_type>> MCP2515::receive_frame()
  {
    // Determine which buffer has a frame
    uint8_t status = read_status();

    // Pulls from RXB0 first
    if (status & RX0IF) {
      receive_frame<id_type>(RX0IF);
    } else if (status & RX0IF) {
      receive_frame<id_type>(RX1IF);
    }
  }

  template <enum IDType id_type>
  std::optional<Frame<id_type>> MCP2515::receive_frame(RxBuffer buffer)
  {
    std::array<uint8_t, 5> frame_ctrl_buf; // Frame "control" data registers
    std::array<uint8_t, 8> frame_data_buf; // Data bytes
    uint8_t status;
    uint32_t id;
    uint8_t cmd;
    uint8_t dlc;
    bool remote;
    
    status = rx_status();

    // Check if message was received in appropriate buffer
    if (!(status & (1 << (6 + buffer))))
      return std::nullopt;

    // Avoid reading from buffer if not of the appropriate frame type
    // We can check the ID type iff message not in both buffers xor we are using buf 0
    if (!((status & 0x18) >> 3 == 3) && buffer == RXB1) {
      if (id_type == StandardID) {
        if (status & (1 << 4))
          return std::nullopt;
      } else if (id_type == ExtendedID) {
        if (status ^ (1 << 4))
          return std::nullopt;
      }
    }

    // Read SPI buffer
    cmd = READ_RX_BUFFER | (buffer ? RX_BUF_RXB1DO : 0);
    spi_assert();
    spi_tx(&cmd, 1);
    spi_rx(frame_ctrl_buf.data(), frame_ctrl_buf.size()); // read "control" data registers
    dlc = frame_ctrl_buf[4] & 0x0F;
    if (dlc)
      spi_rx(frame_data_buf.data(), dlc);
    spi_deassert();

    // Error handling, don't decode frame if we don't have the right ID type
    if (id_type == StandardID) {
      if (frame_ctrl_buf[1] & (1 << 3))
        return std::nullopt;
    } else if (id_type == ExtendedID) {
      if (frame_ctrl_buf[1] ^ (1 << 3))
        return std::nullopt;
    }

    // Compose into Frame type
    if (id_type == StandardID) {
      id = frame_ctrl_buf[0] << 3;  // SID[10-3]
      id |= frame_ctrl_buf[1] >> 5; // SID[2-0]
    } else if (id_type == ExtendedID) {
      id = frame_ctrl_buf[0] << 21;           // EID[28-21]
      id |= (frame_ctrl_buf[1] & 0xE0) << 18; // EID[20-18]
      id |= (frame_ctrl_buf[1] & 0x03) << 16; // EID[17-16]
      id |= frame_ctrl_buf[2] << 8;           // EID[15-8]
      id |= frame_ctrl_buf[3];                // EID[7-0]
    }

    // TODO maybe remote frames don't work, I don't use them so idk and idc
    remote = (id_type == StandardID) ? frame_ctrl_buf[1] & (1 << 4) : false;
    Frame<id_type> frame(id, remote, frame_data_buf.data, dlc);

    // Clear RXnIF flag
    status = read_reg(CANINTF);
    status ^= (1 << buffer);
    write_reg(CANINTF, status);

    return std::optional<Frame<id_type>>(frame);
  }


  /// @brief Sends a READ_STATUS command
  /// @returns Bit-field using ReadStatusFlags (see fig. 12-8 of datasheet)
  uint8_t MCP2515::read_status()
  {
    uint8_t buf;

    spi_assert();
    // Send READ_STATUS command
    buf = READ_STATUS;
    spi_tx(&buf, 1);
    spi_rx(&buf, 1);
    spi_deassert();

    return buf;
  }

  /// @brief Sends an RX_STATUS command
  /// @returns Bit-field of receive status (see fig. 12-9 of datasheet)
  uint8_t MCP2515::rx_status()
  {
    uint8_t buf;

    spi_assert();
    // Send RX_STATUS command
    buf = RX_STATUS;
    spi_tx(&buf, 1);
    spi_rx(&buf, 1);
    spi_deassert();

    return buf;
  }

  /// @brief Sends a Request To Send (RTS) command to send a TX buffer
  /// @note buf must be <= 2
  void MCP2515::rts(uint8_t buf)
  {
    uint8_t cmd = RTS | ((1 << buf) & 0x07);
    spi_assert();
    spi_tx(&cmd, 1);
    spi_deassert();
  }

  /// @brief Reads the value of the provided register
  /// @returns Value of the register
  uint8_t MCP2515::read_reg(uint8_t reg)
  {
    uint8_t data;

    spi_assert();
    spi_tx(&reg, 1);
    spi_rx(&data, 1);
    spi_deassert();

    return data;
  }

  /// @brief Writes a given value to a register
  void MCP2515::write_reg(uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = {reg, value};

    spi_assert();
    spi_tx(buf, 2);
    spi_deassert();
  }

  /// @brief Reads a continuous chunk.
  void MCP2515::read(uint8_t start_addr, uint8_t len, uint8_t* buffer)
  {
    uint8_t buf_[2] = {READ, start_addr};
    spi_assert();
    spi_tx(buf_, 2);
    spi_rx(buffer, len);
    spi_deassert();
  }

  /// @brief Writes a continuous chunk.
  void MCP2515::write(uint8_t start_addr, uint8_t len, uint8_t* buffer)
  {
    uint8_t buf_[2] = {WRITE, start_addr};
    spi_assert();
    spi_tx(buf_, 2);
    spi_tx(buffer, len);
    spi_deassert();
  }

  template <enum IDType id_type>
  constexpr void MCP2515::write_buf_id(Frame<id_type> &frame, uint8_t* id_buf)
  {
    if (id_type == StandardID) {
      id_buf[0] = (frame.id & 0x000007F8) >> 3; // SID[10-3]
      id_buf[1] = (frame.id & 0x00000007) << 5; // SID[2-0]
    } else if (id_type == ExtendedID) {
      id_buf[0] = ((frame.id & 0x1FE00000) >> 22);             // EID[28-21]
      id_buf[1] = ((frame.id & 0x001C0000) >> 14) | (1 << 3) | // EID[20-18]
                  (frame.id & 0x00030000);                     // EID[17-16]
      id_buf[2] = (frame.id & 0x0000FF00) >> 8;                // EID[15-8]
      id_buf[3] = (frame.id & 0x000000FF);                     // EID[7-0]
    }

    // Encode RTR and DLC
    id_buf[4] = (frame.remote & (1 << 6)) | (frame.data_len & 0x0F);
  }
} // Namespace MCP2515

} // Namespace CAN
} // Namespace DeviceDrivers
