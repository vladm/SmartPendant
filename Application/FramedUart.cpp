//******************************************************************************
//  @file FramedUart.cpp
//  @author Nicolai Shlapunov
//
//  @details FramedUart: grblHAL MPG framed transport, implementation
//
//  @copyright Copyright (c) 2016-2026, Devtronic & Nicolai Shlapunov
//             All rights reserved.
//
//  @section SUPPORT
//
//   Devtronic invests time and resources providing this open source code,
//   please support Devtronic and open-source hardware/software by
//   donations and/or purchasing products from Devtronic.
//
//******************************************************************************

// *****************************************************************************
// ***   Includes   ************************************************************
// *****************************************************************************
#include "FramedUart.h"

#include <cstring> // For memcpy()

// *****************************************************************************
// ***   Public: Init   ********************************************************
// *****************************************************************************
Result FramedUart::Init()
{
  // Initialize the underlying UART first
  Result result = uart.Init();

  // Reset all protocol state
  ResetSession();

  // Return result
  return result;
}

// *****************************************************************************
// ***   Public: DeInit   ******************************************************
// *****************************************************************************
Result FramedUart::DeInit()
{
  // Link is down until Init() is called again
  link_up = false;

  // Return result
  return uart.DeInit();
}

// *****************************************************************************
// ***   Public: SetAttempts   *************************************************
// *****************************************************************************
void FramedUart::SetAttempts(uint32_t attempts)
{
  // Clamp instead of refusing: a value from settings that has gone out of
  // range should not leave the link with no retransmissions at all
  if(attempts < MIN_ATTEMPTS)
  {
    attempts = MIN_ATTEMPTS;
  }
  else if(attempts > MAX_ATTEMPTS)
  {
    attempts = MAX_ATTEMPTS;
  }
  else
  {
    ; // Do nothing - MISRA rule
  }

  attempts_limit = attempts;
}

// *****************************************************************************
// ***   Public: SetAckTimeoutFloor   ******************************************
// *****************************************************************************
void FramedUart::SetAckTimeoutFloor(uint32_t floor_ms)
{
  // Clamp instead of refusing: a value from settings that has gone out of
  // range should not stall the link behind an absurd timeout. There is no
  // lower clamp - zero is the documented "no floor" value, and the floor can
  // only raise the derived timeout, never lower it.
  if(floor_ms > MAX_ACK_MIN_MS)
  {
    floor_ms = MAX_ACK_MIN_MS;
  }

  ack_min_ms = floor_ms;
  ApplyAckTimeout();
}

// *****************************************************************************
// ***   Public: Read   ********************************************************
// *****************************************************************************
Result FramedUart::Read(uint8_t& rx_byte)
{
  // Receive frames and retransmit unacknowledged ones. GrblComm calls Read()
  // every task tick, so this is the protocol heartbeat.
  Service();

  // Give out one byte of the unframed stream
  return ReadByte(rx_byte);
}

// *****************************************************************************
// ***   Public: Read   ********************************************************
// *****************************************************************************
Result FramedUart::Read(uint8_t* rx_buf_ptr, uint32_t& size)
{
  Result result = Result::ERR_NULL_PTR;

  // Service once for the whole buffer, not once per byte
  Service();

  // Check pointer before use
  if(rx_buf_ptr != nullptr)
  {
    uint32_t cnt = 0u;
    // Read as much as requested or as much as available
    while(cnt < size)
    {
      if(ReadByte(rx_buf_ptr[cnt]).IsBad()) break;
      cnt++;
    }
    // Return number of bytes read
    size = cnt;
    result = (cnt != 0u) ? Result::RESULT_OK : Result::ERR_UART_EMPTY;
  }

  // Return result
  return result;
}

// *****************************************************************************
// ***   Public: Write   *******************************************************
// *****************************************************************************
Result FramedUart::Write(uint8_t tx_byte)
{
  // Single byte is a real time command
  return Write(&tx_byte, 1u);
}

// *****************************************************************************
// ***   Public: Write   *******************************************************
// *****************************************************************************
Result FramedUart::Write(uint8_t* tx_buf_ptr, uint32_t size)
{
  Result result = Result::ERR_NULL_PTR;

  // Service the protocol before deciding if the channel is free
  Service();

  // Check parameters before use
  if((tx_buf_ptr != nullptr) && (size != 0u) && (size <= MPG_TRANSPORT_MAX_PAYLOAD))
  {
    // GrblComm sends every real time command as a single byte and every
    // g-code line with its terminator, so the size selects the channel.
    if(size == 1u)
    {
      result = SendOnChannel(rt_ch, (uint8_t)MpgFrame_Rlt, tx_buf_ptr, (uint8_t)size);
    }
    else
    {
      result = SendOnChannel(cmd_ch, (uint8_t)MpgFrame_Dat, tx_buf_ptr, (uint8_t)size);
    }
  }
  else if(tx_buf_ptr != nullptr)
  {
    // Payload doesn't fit into a frame - the caller has to split it
    result = Result::ERR_BAD_PARAMETER;
  }
  else
  {
    ; // Do nothing - MISRA rule
  }

  // Return result
  return result;
}

// *****************************************************************************
// ***   Public: IsTxComplete   ************************************************
// *****************************************************************************
bool FramedUart::IsTxComplete(void)
{
  // Reports whether a write can be accepted, not the state of the hardware:
  // a frame is assembled into the buffer of its channel and pushed into the
  // UART later, so there is no reason to hold the caller back while the line
  // is busy.
  //
  // It also can't be gated on acknowledges alone: this is checked before the
  // message is looked at, so a channel waiting for an acknowledge would hold
  // back a real time character queued for the other one. Only when BOTH
  // channels are occupied there is really nothing to accept. A write into a
  // channel that is busy is refused by Write() with ERR_UART_BUSY instead.
  return (!(cmd_ch.in_flight && rt_ch.in_flight));
}

// *****************************************************************************
// ***   Private: ReadByte   ***************************************************
// *****************************************************************************
Result FramedUart::ReadByte(uint8_t& rx_byte)
{
  Result result = Result::ERR_UART_EMPTY;

  // Give out one byte of the unframed stream
  if(rx_data_tail != rx_data_head)
  {
    rx_byte = rx_data_buf[rx_data_tail];
    rx_data_tail = (rx_data_tail + 1u) % RX_DATA_BUF_SIZE;
    result = Result::RESULT_OK;
  }

  // Return result
  return result;
}

// *****************************************************************************
// ***   Private: Service   ****************************************************
// *****************************************************************************
void FramedUart::Service(void)
{
  uint8_t rx_byte = 0u;

  // Receive everything the UART has
  while(uart.Read(rx_byte).IsGood())
  {
    ProcessRxByte(rx_byte);
  }

  // Retransmit unacknowledged frames
  ServiceChannel(cmd_ch);
  ServiceChannel(rt_ch);

  // Report a given up command as soon as the buffer has room for the byte.
  // Retried rather than dropped: if it doesn't fit the reader is stalled, and
  // that is exactly the case where losing it costs a skipped line.
  if(cmd_loss_pending)
  {
    uint8_t marker = CMD_LOST_MARKER;
    if(PushRxData(&marker, 1u, false))
    {
      cmd_loss_pending = false;
    }
  }

  // Push queued frames into the UART
  PumpTx();
}

// *****************************************************************************
// ***   Private: ProcessRxByte   **********************************************
// *****************************************************************************
void FramedUart::ProcessRxByte(uint8_t rx_byte)
{
  switch(rx_state)
  {
    case RX_SOF:
      // Anything before the start of frame byte is noise and is dropped
      if(rx_byte == MPG_TRANSPORT_SOF)
      {
        rx_crc_calc = 0xFFFFu;
        rx_state = RX_SEQ;
      }
      break;

    case RX_SEQ:
      rx_seq = rx_byte;
      rx_crc_calc = mpg_frame_crc16_add(rx_crc_calc, rx_byte);
      rx_state = RX_TYPE;
      break;

    case RX_TYPE:
      rx_type = rx_byte;
      rx_crc_calc = mpg_frame_crc16_add(rx_crc_calc, rx_byte);
      rx_state = RX_LEN;
      break;

    case RX_LEN:
      rx_len = rx_byte;
      rx_crc_calc = mpg_frame_crc16_add(rx_crc_calc, rx_byte);
      rx_payload_idx = 0u;
      // A length that can't fit can't be a valid frame - drop it and resync
      if(rx_len > MPG_TRANSPORT_MAX_PAYLOAD)
      {
        rx_state = RX_SOF;
      }
      else
      {
        rx_state = (rx_len != 0u) ? RX_PAYLOAD : RX_CRC_LO;
      }
      break;

    case RX_PAYLOAD:
      rx_payload[rx_payload_idx] = rx_byte;
      rx_payload_idx++;
      rx_crc_calc = mpg_frame_crc16_add(rx_crc_calc, rx_byte);
      if(rx_payload_idx >= rx_len)
      {
        rx_state = RX_CRC_LO;
      }
      break;

    case RX_CRC_LO:
      rx_crc_frame = rx_byte;
      rx_state = RX_CRC_HI;
      break;

    case RX_CRC_HI:
      rx_crc_frame |= (uint16_t)rx_byte << 8;
      ProcessRxFrame();
      rx_state = RX_SOF;
      break;

    default:
      rx_state = RX_SOF;
      break;
  }
}

// *****************************************************************************
// ***   Private: ProcessRxFrame   *********************************************
// *****************************************************************************
void FramedUart::ProcessRxFrame(void)
{
  // Corrupted frame: the type is unknown, so ask for whatever the controller
  // has outstanding instead of waiting for its timeout
  if(rx_crc_frame != rx_crc_calc)
  {
    uint8_t payload = (uint8_t)MpgFrame_Unk;
    stats.rx_crc_err++;
    SendControlFrame((uint8_t)MpgFrame_Nak, 0u, &payload, 1u);
  }
  else
  {
    // Any CRC valid frame proves the controller speaks the protocol
    link_up = true;

    switch(rx_type)
    {
      case MpgFrame_Dat:
        // Only an exact repeat of the last accepted sequence number is a
        // duplicate: it is acknowledged again but not delivered again. Any
        // other value is accepted and resynchronizes the stream.
        if(rx_last_seq_valid && (rx_seq == rx_last_seq))
        {
          uint8_t payload = (uint8_t)MpgFrame_Dat;
          stats.rx_dup++;
          SendControlFrame((uint8_t)MpgFrame_Ack, rx_seq, &payload, 1u);
        }
        else if(PushRxData(rx_payload, rx_len, ResyncNeeded(rx_seq)))
        {
          uint8_t payload = (uint8_t)MpgFrame_Dat;
          rx_last_seq = rx_seq;
          rx_last_seq_valid = true;
          rx_resync_pending = false;
          stats.rx_data++;
          SendControlFrame((uint8_t)MpgFrame_Ack, rx_seq, &payload, 1u);
        }
        else
        {
          // No room to buffer - Nak carrying that sequence number
          uint8_t payload = (uint8_t)MpgFrame_Dat;
          stats.rx_no_room++;
          SendControlFrame((uint8_t)MpgFrame_Nak, rx_seq, &payload, 1u);
        }
        break;

      case MpgFrame_Ack: // intentional fall-trough
      case MpgFrame_Nak:
      {
        // payload[0] names the channel this refers to
        uint8_t acked_type = (rx_len == 1u) ? rx_payload[0u] : (uint8_t)MpgFrame_Unk;
        bool is_ack = (rx_type == (uint8_t)MpgFrame_Ack);
        // Counted here rather than in HandleAckNak, which is about what the
        // frame does to a channel and not about the frame: a Nak naming an
        // unidentifiable frame reaches both channels but is still one Nak,
        // and one that arrives with nothing in flight reaches neither.
        if(is_ack)
        {
          stats.rx_ack++;
        }
        else
        {
          stats.rx_nak++;
        }
        if(acked_type == (uint8_t)MpgFrame_Rlt)
        {
          HandleAckNak(rt_ch, is_ack, rx_seq);
        }
        else if(acked_type == (uint8_t)MpgFrame_Dat)
        {
          HandleAckNak(cmd_ch, is_ack, rx_seq);
        }
        else if(!is_ack)
        {
          // Nak for an unidentifiable frame - retransmit whatever is in flight
          HandleAckNak(cmd_ch, false, cmd_ch.seq);
          HandleAckNak(rt_ch, false, rt_ch.seq);
        }
        else
        {
          ; // Do nothing - MISRA rule
        }
        break;
      }

      default:
        // Nothing else is expected from the controller
        break;
    }
  }
}

// *****************************************************************************
// ***   Private: SendOnChannel   **********************************************
// *****************************************************************************
Result FramedUart::SendOnChannel(tx_channel_t& ch, uint8_t type, const uint8_t* payload, uint8_t len)
{
  Result result = Result::ERR_UART_BUSY;

  // One frame in flight per channel
  if(!ch.in_flight)
  {
    // Payload has to fit into the buffer of this channel
    if(((uint32_t)len + MPG_TRANSPORT_OVERHEAD) <= ch.capacity)
    {
      // Assemble the frame. It stays in the buffer: the UART transmits from
      // it directly and it is reused for every retransmission.
      ch.size = (uint32_t)mpg_frame_build(ch.frame, ch.seq, (mpg_frame_type_t)type, payload, len);
      ch.type = type;
      ch.attempts = 1u;
      ch.tx_timestamp = RtosTick::GetTimeMs();
      ch.in_flight = true;
      ch.pending_tx = true;
      stats.tx_data++;
      // Try to push it out right away
      PumpTx();
      result = Result::RESULT_OK;
    }
    else
    {
      result = Result::ERR_BAD_PARAMETER;
    }
  }

  // Return result
  return result;
}

// *****************************************************************************
// ***   Private: ServiceChannel   *********************************************
// *****************************************************************************
void FramedUart::ServiceChannel(tx_channel_t& ch)
{
  // Only a frame that waits for an acknowledge needs attention
  if(ch.in_flight && (RtosTick::GetTimeMs() - ch.tx_timestamp >= ack_timeout_ms))
  {
    RetransmitOrDrop(ch);
  }
}

// *****************************************************************************
// ***   Private: RetransmitOrDrop   *******************************************
// *****************************************************************************
void FramedUart::RetransmitOrDrop(tx_channel_t& ch)
{
  if(ch.attempts < attempts_limit)
  {
    // Retransmit the identical frame
    ch.attempts++;
    ch.tx_timestamp = RtosTick::GetTimeMs();
    ch.pending_tx = true;
    stats.tx_retry++;
  }
  else
  {
    // Link is down. The frame is dropped, which frees the channel so the
    // caller isn't stuck retrying forever, and the sequence number is
    // ADVANCED rather than reset: the controller may have received and
    // delivered the frame and only lost the acknowledges, so reusing the
    // same number would make the next command look like a duplicate and
    // be silently discarded.
    ch.in_flight = false;
    ch.pending_tx = false;
    ch.seq = NextSeq(ch.seq);
    stats.tx_dropped++;
    // Tell the reader its command never arrived, so it fails that command
    // instead of waiting for a response that can't come
    if(ch.report_loss)
    {
      cmd_loss_pending = true;
    }
    // We should count link downs when it actually goes down, not when the
    // retry limit is hit, but there is no link already
    if(link_up)
    {
      link_up = false;
      stats.link_downs++;
      // Whatever the controller was sending when the link failed, the reader
      // may be holding half of it - make sure it is told before anything new
      // is appended
      rx_resync_pending = true;
    }
  }
}

// *****************************************************************************
// ***   Private: HandleAckNak   ***********************************************
// *****************************************************************************
void FramedUart::HandleAckNak(tx_channel_t& ch, bool is_ack, uint8_t seq)
{
  // Only interesting while a frame waits for an acknowledge
  if(ch.in_flight)
  {
    if(is_ack)
    {
      // Match on the sequence number: the type is checked by the caller
      if(seq == ch.seq)
      {
        ch.in_flight = false;
        ch.seq = NextSeq(ch.seq);
      }
    }
    else
    {
      // Retransmit immediately, same sequence number, same bytes, instead of
      // waiting out the acknowledge timeout. It costs an attempt exactly like
      // a timeout does - the controller shares one budget between the two for
      // the same reason, so that a peer stuck sending Nak can't keep a
      // channel busy indefinitely. The Nak itself is counted by the caller:
      // this can be reached twice for one Nak, when it doesn't name a channel.
      RetransmitOrDrop(ch);
      PumpTx();
    }
  }
}

// *****************************************************************************
// ***   Private: SendControlFrame   *******************************************
// *****************************************************************************
void FramedUart::SendControlFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len)
{
  // Free space in the queue, one entry is always kept free to tell an empty
  // queue from a full one
  uint32_t used = (ctrl_head - ctrl_tail + CTRL_QUEUE_SIZE) % CTRL_QUEUE_SIZE;

  // A control frame that doesn't fit is dropped: it is never retransmitted
  // anyway, and the peer recovers by its own timeout
  if((used + 1u) < CTRL_QUEUE_SIZE)
  {
    ctrl_queue[ctrl_head].seq = seq;
    ctrl_queue[ctrl_head].type = type;
    ctrl_queue[ctrl_head].payload = (len != 0u) ? payload[0u] : (uint8_t)MpgFrame_Unk;
    ctrl_head = (ctrl_head + 1u) % CTRL_QUEUE_SIZE;
    // Counted on queueing: a frame that doesn't fit is dropped below and was
    // never sent, so counting it here would overstate what left the pendant
    if(type == (uint8_t)MpgFrame_Ack)
    {
      stats.tx_ack++;
    }
    else
    {
      stats.tx_nak++;
    }
  }

  // Try to push it out right away
  PumpTx();
}

// *****************************************************************************
// ***   Private: ResetSession   ***********************************************
// *****************************************************************************
void FramedUart::ResetSession(void)
{
  // Receiver
  rx_state = RX_SOF;
  rx_payload_idx = 0u;
  rx_last_seq_valid = false;
  rx_resync_pending = false;
  cmd_loss_pending = false;
  rx_data_head = 0u;
  rx_data_tail = 0u;

  // Transmitter
  ctrl_head = 0u;
  ctrl_tail = 0u;
  // Both channels start at the reserved zero. This is the only place that
  // sets it, and it runs from Init() only, so zero really does mean "first
  // frame after a reset" - see NextSeq().
  cmd_ch.in_flight = false;
  cmd_ch.pending_tx = false;
  cmd_ch.seq = 0u;
  rt_ch.in_flight = false;
  rt_ch.pending_tx = false;
  rt_ch.seq = 0u;

  // Link is down until a CRC valid frame arrives
  link_up = false;
}

// *****************************************************************************
// ***   Private: PumpTx   *****************************************************
// *****************************************************************************
void FramedUart::PumpTx(void)
{
  // The UART transmits straight from the buffers below, so nothing new can be
  // given to it until the previous transfer is complete
  if(uart.IsTxComplete())
  {
    // Acknowledges go first: they unblock the controller, they are tiny, and
    // they are the only frames that are never retransmitted if lost
    if(ctrl_tail != ctrl_head)
    {
      uint8_t payload = ctrl_queue[ctrl_tail].payload;
      uint32_t size = (uint32_t)mpg_frame_build(ctrl_frame, ctrl_queue[ctrl_tail].seq,
                                                 (mpg_frame_type_t)ctrl_queue[ctrl_tail].type,
                                                 &payload, 1u);
      ctrl_tail = (ctrl_tail + 1u) % CTRL_QUEUE_SIZE;
      uart.Write(ctrl_frame, size);
    }
    // Then the real time channel - it exists to keep latency low
    else if(rt_ch.pending_tx)
    {
      rt_ch.pending_tx = false;
      uart.Write(rt_ch.frame, rt_ch.size);
    }
    // And the command channel last
    else if(cmd_ch.pending_tx)
    {
      cmd_ch.pending_tx = false;
      uart.Write(cmd_ch.frame, cmd_ch.size);
    }
    else
    {
      ; // Do nothing - MISRA rule
    }
  }
}

// *****************************************************************************
// ***   Private: PushRxData   *************************************************
// *****************************************************************************
bool FramedUart::PushRxData(const uint8_t* data, uint32_t size, bool resync)
{
  bool result = false;

  // The whole payload has to fit: delivering a part of it would corrupt the
  // stream, and the sender retransmits the frame anyway. The resync marker
  // costs one more byte and is written in the same step - a payload that
  // arrived after a gap must never reach the reader without it, so either
  // both go in or the frame is refused and gets retransmitted.
  if((size + (resync ? 1u : 0u)) <= GetRxDataFree())
  {
    if(resync)
    {
      rx_data_buf[rx_data_head] = RESYNC_MARKER;
      rx_data_head = (rx_data_head + 1u) % RX_DATA_BUF_SIZE;
    }

    for(uint32_t i = 0u; i < size; i++)
    {
      rx_data_buf[rx_data_head] = data[i];
      rx_data_head = (rx_data_head + 1u) % RX_DATA_BUF_SIZE;
    }
    result = true;
  }

  // Return result
  return result;
}

// *****************************************************************************
// ***   Private: GetRxDataFree   **********************************************
// *****************************************************************************
uint32_t FramedUart::GetRxDataFree(void)
{
  // One byte is always kept free to tell an empty buffer from a full one
  uint32_t used = (rx_data_head - rx_data_tail + RX_DATA_BUF_SIZE) % RX_DATA_BUF_SIZE;
  return (RX_DATA_BUF_SIZE - used - 1u);
}
