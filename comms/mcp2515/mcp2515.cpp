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

namespace MCP2515 {
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

  void MCP2515::reset()
  {
    uint8_t ins = RESET;
    spi_assert();
    spi_tx(&ins, 1);
    spi_deassert();
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
  /// @returns Nothing
  void MCP2515::write_reg(uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = {reg, value};

    spi_assert();
    spi_tx(buf, 2);
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
