//******************************************************************************
//  @file FramedUart.h
//  @author Nicolai Shlapunov
//
//  @details FramedUart: grblHAL MPG framed transport, header
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

#ifndef FramedUart_h
#define FramedUart_h

// *****************************************************************************
// ***   Includes   ************************************************************
// *****************************************************************************
#include "DevCore.h"

// *****************************************************************************
// ***   Wire format, copied from the plugin's mpg_transport.h   ***************
// *****************************************************************************

#define MPG_TRANSPORT_SOF 0x02 // ASCII STX

// Max payload per frame. Keep small enough that a frame time is short
// (128 bytes + 6 overhead = ~11.6 ms @ 115200 baud) but large enough that
// framing overhead on status reports stays low. Must be <= 250.
#ifndef MPG_TRANSPORT_MAX_PAYLOAD
#define MPG_TRANSPORT_MAX_PAYLOAD 128
#endif

#if MPG_TRANSPORT_MAX_PAYLOAD > 250 || MPG_TRANSPORT_MAX_PAYLOAD < 8
#error "MPG_TRANSPORT_MAX_PAYLOAD must be between 8 and 250 - len is a single byte and the frame adds 6"
#endif

#define MPG_TRANSPORT_OVERHEAD 6 // SOF + seq + type + len + crc16

// Largest frame that can appear on the wire. Both ends use this to size
// buffers and to derive the acknowledgement timeout from the baud rate.
#define MPG_TRANSPORT_MAX_FRAME (MPG_TRANSPORT_MAX_PAYLOAD + MPG_TRANSPORT_OVERHEAD)

typedef enum
{
  MpgFrame_Unk = 0x00, // only valid in a Nak payload: "frame unidentifiable"
  MpgFrame_Dat = 0x01, // byte stream, either direction
  MpgFrame_Rlt = 0x02, // MPG/DRO->controller: expedited byte stream, own sequence space
  MpgFrame_Ack = 0x03, // seq = acknowledged seq, payload[0] = type acknowledged
  MpgFrame_Nak = 0x04  // payload[0] = type to retransmit, or MpgFrame_Unk for "anything outstanding"
} mpg_frame_type_t;

// *****************************************************************************
// ***   Global: CRC-16/CCITT-FALSE, shared reference implementation   *********
// *****************************************************************************
static inline uint16_t mpg_frame_crc16_add(uint16_t crc, uint8_t c)
{
  uint_fast8_t bits = 8;
  crc ^= (uint16_t)c << 8;
  do {crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;} while(--bits);
  return crc;
}

// *****************************************************************************
// ***   Global: CRC-16/CCITT-FALSE, shared reference implementation   *********
// *****************************************************************************
static inline uint16_t mpg_frame_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;
  while(length--) crc = mpg_frame_crc16_add(crc, *data++);
  return crc;
}

// *****************************************************************************
// ***   Global: mpg_frame_build   *********************************************
// *****************************************************************************
// Assemble a frame into buf (must hold len + MPG_TRANSPORT_OVERHEAD bytes),
// returns total frame size. This is the reference encoder - it must stay
// byte identical to the one in the plugin's mpg_transport.h.
static inline size_t mpg_frame_build(uint8_t *buf, uint8_t seq, mpg_frame_type_t type, const uint8_t *payload, uint8_t len)
{
  size_t i;
  uint16_t crc = 0xFFFF;

  buf[0] = MPG_TRANSPORT_SOF;
  buf[1] = seq;
  buf[2] = (uint8_t)type;
  buf[3] = len;

  for(i = 0; i < len; i++) buf[4 + i] = payload[i];

  for(i = 1; i < (size_t)(4 + len); i++) crc = mpg_frame_crc16_add(crc, buf[i]);

  buf[4 + len] = (uint8_t)(crc & 0xFF);
  buf[5 + len] = (uint8_t)(crc >> 8);

  return (size_t)len + MPG_TRANSPORT_OVERHEAD;
}

// *****************************************************************************
// ***   FramedUart   **********************************************************
// *****************************************************************************
// * Reliable transport for the grblHAL MPG/DRO pendant link. It is an IUart
// * itself and wraps another IUart, so it can be placed between the hardware
// * UART and GrblComm without any changes above it:
// *
// *   StHalUart uart1(huart1);
// *   FramedUart framed_uart(uart1);
// *   // Plain or encapsulated GRBL is selected by what is passed in - after
// *   // the settings are read from the EEPROM and before the task is created
// *   if(framed) GrblComm::GetInstance().InitTask(framed_uart);
// *   else       GrblComm::GetInstance().InitTask(uart1);
// *
// * Everything written is framed, CRC protected and retransmitted until the
// * controller acknowledges it; everything received is unframed and handed
// * out as a plain byte stream. The layer doesn't interpret the data at all:
// * real time characters, g-code and $-commands are all just bytes, and the
// * controller's own real time filter decides what is what. There is no
// * session, handshake or capability layer - MPG mode is claimed by sending
// * the ordinary CMD_MPG_MODE_TOGGLE(0x8B) character like in ASCII mode.
// *
// * Channel selection: the protocol has a separate real time channel so that
// * a g-code frame waiting for an acknowledge can't delay a feed hold. Since
// * GrblComm writes real time commands as a single byte and every g-code line
// * with its terminator(two bytes or more), the size of the write selects the
// * channel: one byte goes to the real time channel, anything longer to the
// * command channel.
// *
// * Back pressure: IsTxComplete() reports "can accept something". A frame is
// * assembled into the buffer of its channel and pushed into the UART later,
// * so the state of the hardware doesn't matter - it is false only when BOTH
// * channels wait for an acknowledge. It can't be gated on acknowledges alone, because it is
// * checked before the message is looked at, so that would hold a feed hold
// * behind a g-code frame that is being retransmitted(300 ms instead of 2 ms
// * in a measurement). A write into a channel that still waits for an
// * acknowledge returns
// * ERR_UART_BUSY, which GrblComm::ProcessMessage() handles by putting the
// * message back to the front of its queue and trying again a tick later.
// * Nothing is lost, and real time commands - which are sent as priority
// * messages - overtake a g-code command that waits for its acknowledge.
// *
// * Servicing(acknowledge timeouts, retransmissions) is done inside Read()
// * and Write(). GrblComm calls Read() every task tick, so no separate task
// * or timer is needed.
// *
// * Both ends have to be set to framed mode deliberately - there is no
// * negotiation and no automatic upgrade. A mismatch means no communication
// * at all: a controller in framed mode discards everything that isn't a
// * valid frame, and a controller in legacy mode sees frames as garbage.
// *****************************************************************************
class FramedUart : public IUart
{
  public:
    // *************************************************************************
    // ***   Public: Constructor   *********************************************
    // *************************************************************************
    explicit FramedUart(IUart& uart_in) : uart(uart_in)
    {
      // Give every channel its own buffer - the UART transmits straight from
      // it, so no additional copy of a frame is ever made
      cmd_ch.frame = cmd_frame;
      cmd_ch.capacity = sizeof(cmd_frame);
      cmd_ch.report_loss = true;
      rt_ch.frame = rt_frame;
      rt_ch.capacity = sizeof(rt_frame);
    };

    // *************************************************************************
    // ***   Public: Init   ****************************************************
    // *************************************************************************
    // * Initializes the underlying UART and resets all protocol state.
    virtual Result Init();

    // *************************************************************************
    // ***   Public: DeInit   **************************************************
    // *************************************************************************
    virtual Result DeInit();

    // *************************************************************************
    // ***   Public: SetBaudRate   *********************************************
    // *************************************************************************
    // * Passed straight to the hardware, and used to scale the acknowledge
    // * timeout: a worst case acknowledge takes about one frame time, which
    // * is 12 ms at 115200 and 24 ms at 57600. Set it before communication
    // * starts. The controller's own timeout has to fit the same budget.
    virtual Result SetBaudRate(uint32_t baud_rate)
    {
      // Acknowledge timeout scales with the line rate
      applied_baud_rate = baud_rate;
      ApplyAckTimeout();
      return uart.SetBaudRate(baud_rate);
    }

    // *************************************************************************
    // ***   Public: Read   ****************************************************
    // *************************************************************************
    // * Returns one byte of the received stream. Also services the protocol,
    // * so it have to be called periodically even if the caller expects no
    // * data - GrblComm does that on every task tick.
    virtual Result Read(uint8_t& rx_byte);

    // *************************************************************************
    // ***   Public: Read   ****************************************************
    // *************************************************************************
    virtual Result Read(uint8_t* rx_buf_ptr, uint32_t& size);

    // *************************************************************************
    // ***   Public: Write   ***************************************************
    // *************************************************************************
    virtual Result Write(uint8_t tx_byte);

    // *************************************************************************
    // ***   Public: Write   ***************************************************
    // *************************************************************************
    // * One byte is sent as a real time frame, anything longer as a command
    // * frame. Returns ERR_UART_BUSY if the selected channel still waits for
    // * an acknowledge - the caller should try again later.
    virtual Result Write(uint8_t* tx_buf_ptr, uint32_t size);

    // *************************************************************************
    // ***   Public: IsTxComplete   ********************************************
    // *************************************************************************
    // * True unless both channels wait for an acknowledge. Doesn't depend on
    // * the hardware: a frame is buffered per channel and transmitted later.
    virtual bool IsTxComplete(void);

    // *************************************************************************
    // ***   Public: Link statistics   *****************************************
    // *************************************************************************
    // * Counters for diagnostics: a cable that works but is marginal shows up
    // * here as retransmissions and CRC errors long before it starts losing
    // * the link. All of them are free running and wrap around.
    // *
    // * Data frames are counted together regardless of the channel they took:
    // * Dat and Rlt are the same event to the link, and one number per event
    // * is easier to read than a pair.
    // *
    // * Acknowledges are counted per frame received or queued, not per channel
    // * they act on: one Nak naming an unidentifiable frame retransmits both
    // * channels but is still one Nak, and one that arrives when nothing is in
    // * flight acts on neither but was still received.
    typedef struct
    {
      // Dat and Rlt frames handed to the transmitter, retransmissions
      // not counted
      uint32_t tx_data = 0u;
      // Retransmissions of those, by timeout or because of a negative
      // acknowledge. Not counted in tx_data above - a frame sent once and
      // retransmitted twice is tx_data = 1, tx_retry = 2.
      uint32_t tx_retry = 0u;
      // Acknowledges and negative acknowledges received for our frames
      uint32_t rx_ack = 0u;
      uint32_t rx_nak = 0u;
      // Dat and Rlt frames accepted and put into the receive buffer
      uint32_t rx_data = 0u;
      // Repeats of the last accepted frame: our acknowledge was lost. Those
      // are acknowledged again but not delivered a second time.
      uint32_t rx_dup = 0u;
      // Acknowledges and negative acknowledges sent for received frames
      uint32_t tx_ack = 0u;
      uint32_t tx_nak = 0u;
      // Frames that failed the CRC check - noise or a bad cable
      uint32_t rx_crc_err = 0u;
      // Frames refused because the receive buffer was full
      uint32_t rx_no_room = 0u;
      // Transmitted frame dropped due retransmission limit reached
      uint32_t tx_dropped = 0u;
      // Times the retry limit was reached and the link was declared down
      uint32_t link_downs = 0u;
    } stats_t;

    // Transmissions of one frame - the first plus its retransmissions - before
    // the link is declared down. Counts TRANSMISSIONS, not retries, to match
    // the controller's "frame attempts" setting, which shares these limits and
    // this default. Timeouts and negative acknowledges share the one budget,
    // so a controller that keeps sending Nak can't hold a channel forever.
    // Public because the settings screen builds its editor from this range.
    static const uint32_t DEFAULT_ATTEMPTS = 4u;
    static const uint32_t MIN_ATTEMPTS = 1u;
    static const uint32_t MAX_ATTEMPTS = 20u;

    // Floor under the baud derived acknowledge timeout, milliseconds. Zero
    // means "use the derived value unchanged", which is right for a wired
    // link. The ceiling matches the controller's: attempts x timeout is how
    // long a dead link takes to be declared down, and 20 x 2000 ms is where
    // this stops being a pendant link in any useful sense.
    // Public because the settings screen builds its editor from this range.
    static const uint32_t DEFAULT_ACK_MIN_MS = 0u;
    static const uint32_t MAX_ACK_MIN_MS = 2000u;

    // *************************************************************************
    // ***   Public: SetAttempts   *********************************************
    // *************************************************************************
    // * Transmissions of one frame before the link is declared down. Set it to
    // * the controller's "frame attempts" setting: the two budgets are
    // * independent, so if they differ the tighter end gives up first and keeps
    // * dropping frames while the other is still patiently retrying.
    // * Out of range values are clamped rather than refused - a link that
    // * retries a sensible number of times beats no link at all.
    void SetAttempts(uint32_t attempts);

    // *************************************************************************
    // ***   Public: SetAckTimeoutFloor   **************************************
    // *************************************************************************
    // * Raises the acknowledge timeout to at least this many milliseconds.
    // *
    // * The timing invariant is one sided: neither end may wait LESS than the
    // * value both derive from the baud rate, and either may wait more. So
    // * this can only ever lengthen the wait, which is the safe direction - a
    // * longer timeout costs slower recovery from real loss, while a shorter
    // * one expires before an acknowledge that is merely late, burns an
    // * attempt on every frame and eventually declares a healthy link down.
    // * That asymmetry is why this is a floor and not an override, and why
    // * zero means "leave the derived value alone" rather than "no timeout".
    // *
    // * It is for links carrying latency the baud rate does not describe - a
    // * Bluetooth, ESP-NOW or radio serial bridge, where the derived timeout
    // * expires while the acknowledge is still in the air and every frame goes
    // * out twice. Set it past the bridge's round trip and the duplication
    // * stops, visible as Data retransmits here and rxdup at the controller.
    // *
    // * Matches the controller's own floor setting; raising only one end fixes
    // * only that end's duplication.
    void SetAckTimeoutFloor(uint32_t floor_ms);

    // *************************************************************************
    // ***   Public: GetStats   ************************************************
    // *************************************************************************
    // * The settings screen reads these from its own task while the comm task
    // * is writing them, with no lock. That is safe only because every counter
    // * is a single 32 bit word, which this core loads and stores atomically -
    // * a reader sees the old value or the new one, never half of either. Keep
    // * them uint32_t: widening one to 64 bits would make it tear.
    inline const stats_t& GetStats(void) {return stats;}

    // *************************************************************************
    // ***   Public: ClearStats   **********************************************
    // *************************************************************************
    // * Same cross task situation as GetStats(). The assignment is not atomic
    // * as a whole, so a count arriving during it can be lost - which is the
    // * right trade for a counter reset.
    inline void ClearStats(void) {stats = stats_t();}

    // *************************************************************************
    // ***   Public: IsLinkUp   ************************************************
    // *************************************************************************
    // * True after any CRC valid frame was received and until the retry limit
    // * declares the link down. Mirrors the controller's own link presence.
    inline bool IsLinkUp(void) {return link_up;}

  private:
    // Acknowledge timeout is derived from the line rate instead of being
    // fixed: the worst case is an incoming frame landing while a maximum
    // sized frame is being transmitted, so the acknowledge has to wait for
    // the line. That is one frame time, and it scales with the baud rate -
    // a fixed 100 ms is 8 times more than needed at 115200, and since a lost
    // link takes attempts x timeout to notice, that padding is paid on every
    // attempt.
    // Two frame times plus a fixed margin covers the wait, the task tick and
    // the acknowledge itself, with room for task jitter.
    static const uint32_t ACK_TIMEOUT_MARGIN_MS = 10u;
    // Never go below this, no matter how fast the line is
    static const uint32_t ACK_TIMEOUT_MIN_MS = 20u;
    // Line rate assumed until SetBaudRate() says otherwise
    static const uint32_t DEFAULT_BAUD_RATE = 115200u;
    // Maximum frame size: payload plus framing
    static const uint32_t MAX_FRAME_SIZE = MPG_TRANSPORT_MAX_FRAME;
    // Real time frames carry a single byte, so they need much less
    static const uint32_t RT_FRAME_SIZE = 1u + MPG_TRANSPORT_OVERHEAD;
    // Control frames(Ack, Nak) carry a single byte too
    static const uint32_t CTRL_FRAME_SIZE = 1u + MPG_TRANSPORT_OVERHEAD;
    // Number of control frames that can wait for the UART. The controller
    // keeps one frame in flight, so one acknowledge is pending at a time -
    // the rest is margin.
    static const uint32_t CTRL_QUEUE_SIZE = 4u;
    // Size of the buffer for the received and unframed data. Has to hold at
    // least one maximum payload, the rest is margin for a slow reader.
    static const uint32_t RX_DATA_BUF_SIZE = 256u;

    // *************************************************************************
    // ***   Private: Receiver state machine   *********************************
    // *************************************************************************
    typedef enum
    {
      RX_SOF,     // Waiting for the start of frame byte
      RX_SEQ,     // Sequence number
      RX_TYPE,    // Frame type
      RX_LEN,     // Payload length
      RX_PAYLOAD, // Payload bytes
      RX_CRC_LO,  // CRC low byte
      RX_CRC_HI   // CRC high byte
    } rx_state_t;

    // *************************************************************************
    // ***   Private: One outgoing stop-and-wait channel   *********************
    // *************************************************************************
    typedef struct
    {
      // Framed bytes kept for retransmission. The buffer is provided by the
      // owner so that the two channels can be sized differently - it is also
      // what the UART transmits from, no second copy is made.
      uint8_t* frame = nullptr;
      // Size of the buffer above
      uint32_t capacity = 0u;
      // Size of the frame
      uint32_t size = 0u;
      // Sequence number of the frame in flight. Starts at zero and never
      // comes back to it - NextSeq() skips zero on wrap - so zero marks the
      // first frame a channel sends after a reset, and nothing else.
      uint8_t seq = 0u;
      // Type of the frame in flight, for matching the acknowledge
      uint8_t type = 0u;
      // Whether losing a frame on this channel has to be reported to the
      // reader. Wired up by the constructor - see CMD_LOST_MARKER.
      bool report_loss = false;
      // Time when the frame was transmitted last time
      uint32_t tx_timestamp = 0u;
      // Transmissions of this frame so far, the first one included
      uint32_t attempts = 0u;
      // True while the frame waits for an acknowledge
      bool in_flight = false;
      // True while the frame still has to be pushed into the UART
      bool pending_tx = false;
    } tx_channel_t;

    // Underlying UART
    IUart& uart;

    // Command channel: g-code lines
    tx_channel_t cmd_ch;
    uint8_t cmd_frame[MAX_FRAME_SIZE];
    // Real time channel: single expedited characters
    tx_channel_t rt_ch;
    uint8_t rt_frame[RT_FRAME_SIZE];

    // Receiver state
    rx_state_t rx_state = RX_SOF;
    // Header of the frame being received
    uint8_t rx_seq = 0u;
    uint8_t rx_type = 0u;
    uint8_t rx_len = 0u;
    // Payload of the frame being received
    uint8_t rx_payload[MPG_TRANSPORT_MAX_PAYLOAD];
    // Number of payload bytes received so far
    uint32_t rx_payload_idx = 0u;
    // CRC calculated over the received frame and the one from the frame
    uint16_t rx_crc_calc = 0u;
    uint16_t rx_crc_frame = 0u;

    // Sequence number of the last accepted incoming frame and the flag that
    // shows that there was one - only an exact repeat of it is a duplicate.
    uint8_t rx_last_seq = 0u;
    bool rx_last_seq_valid = false;
    // Transmissions allowed per frame, see SetAttempts()
    uint32_t attempts_limit = DEFAULT_ATTEMPTS;
    // Set when a command frame was given up on and the reader has not been told
    // yet. Kept until it fits, because a full receive buffer means the reader
    // is stalled - exactly when losing this would be worst. See CMD_LOST_MARKER.
    bool cmd_loss_pending = false;
    // Set when bytes are known to be missing from the middle of the received
    // stream, cleared once the reader has been told about it. See RESYNC_MARKER.
    bool rx_resync_pending = false;

    // Buffer for the received and unframed data
    uint8_t rx_data_buf[RX_DATA_BUF_SIZE];
    uint32_t rx_data_head = 0u;
    uint32_t rx_data_tail = 0u;

    // Acknowledges and negative acknowledges waiting for the UART. They are
    // never retransmitted, so only the three values they carry are kept and
    // the frame is assembled when the UART is free.
    typedef struct
    {
      uint8_t seq = 0u;
      uint8_t type = 0u;
      uint8_t payload = 0u;
    } ctrl_frame_t;
    ctrl_frame_t ctrl_queue[CTRL_QUEUE_SIZE];
    uint32_t ctrl_head = 0u;
    uint32_t ctrl_tail = 0u;
    // Frame given to the UART - has to stay valid until the transfer is
    // complete, so it can't be a local variable
    uint8_t ctrl_frame[CTRL_FRAME_SIZE];

    // Link state: set by any CRC valid frame, cleared by the retry limit
    bool link_up = false;
    // Line rate the timeout below was derived from
    uint32_t applied_baud_rate = DEFAULT_BAUD_RATE;
    // User floor under it, see SetAckTimeoutFloor()
    uint32_t ack_min_ms = DEFAULT_ACK_MIN_MS;
    // Acknowledge timeout actually used: derived, then raised to the floor
    uint32_t ack_timeout_ms = CalcAckTimeout(DEFAULT_BAUD_RATE);
    // Diagnostic counters, see GetStats()
    stats_t stats;

    // Byte handed to the reader ahead of a payload when frames were lost, to
    // make it throw away the partial line it is holding.
    //
    // Must stay equal to GrblComm::ASCII_CAN - not included from here, since
    // the transport knows nothing about its reader. The reader already gave
    // this byte that meaning before the frame layer existed, so no new code
    // path is needed at that end.
    //
    // This is only ever written INTO the receive buffer, never transmitted.
    // The same value sent the other way is CMD_RESET and soft resets the
    // controller; nothing here can put it on the wire.
    //
    // It doesn't have to be a value the controller can never emit. A real one
    // arriving in its output costs a discarded status report and nothing
    // else, and reports come five to ten times a second. What matters is that
    // a gap is never stitched over: the reader's field splitting stops at the
    // first missing separator and leaves the rest at whatever it held, so
    // "<Idle|MPos:1" followed later by "|WCO:0.000,0.000,0.000>" is a clean
    // parse with a zero work offset, and a wrong offset moves the machine to
    // the wrong place.
    static const uint8_t RESYNC_MARKER = 0x18u;

    // Byte handed to the reader when a command frame is given up on, to say it
    // never reached the controller.
    //
    // Must stay equal to GrblComm::ASCII_NAK. Without it the reader waits for
    // a response that can never come, and its own timeout eventually decides
    // the command's status is merely "lost" - which a caller streaming a
    // program reads as permission to send the next line. A skipped g-code line
    // moves the machine somewhere nobody asked for, so a dropped command has
    // to be reported, not waited out.
    //
    // Only the command channel sends this. A dropped real time frame has no
    // response outstanding, so there is nothing to report.
    static const uint8_t CMD_LOST_MARKER = 0x15u;

    // *************************************************************************
    // ***   Private: NextSeq   ************************************************
    // *************************************************************************
    // * The sequence number that follows the given one, skipping zero.
    // *
    // * Zero is reserved for the first frame a channel sends after a reset.
    // * The controller identifies a duplicate by sequence number AND CRC, so a
    // * pendant that reboots and restarts its numbering is normally picked up
    // * without trouble; the one case it can't tell apart is a frame that is
    // * byte identical to the last one it accepted at the same number. Keeping
    // * zero out of the normal rotation confines that case to the first frame
    // * after a reboot, which GrblComm always sends as a real time status
    // * request - idempotent, so losing it costs nothing. Without the skip the
    // * collision could land on any frame, a g-code line included, and there
    // * suppression would silently drop a real command.
    // *
    // * So 255 is followed by 1, not by 0.
    inline static uint8_t NextSeq(uint8_t seq) {seq++; if(!seq) seq++; return seq;}

    // *************************************************************************
    // ***   Private: ResyncNeeded   *******************************************
    // *************************************************************************
    // * Whether the reader has to be told that bytes are missing before the
    // * frame carrying this sequence number is handed over.
    // *
    // * Both ends number frames the same way - NextSeq(), which skips zero -
    // * so anything other than the successor of the last accepted number means
    // * frames went missing. The controller gave up on one after its attempt
    // * budget ran out, or it discarded queued output and deliberately burned
    // * a number to say so. Either way bytes are gone from the middle of the
    // * stream, and one skip stands for any amount of loss.
    // *
    // * A frame at the reserved zero lands here too: it means the controller
    // * restarted, which is a discontinuity in its own right.
    // *
    // * The flag covers what a sequence number can't - our own link loss.
    inline bool ResyncNeeded(uint8_t seq) const {return rx_resync_pending || (rx_last_seq_valid && (seq != NextSeq(rx_last_seq)));}

    // *************************************************************************
    // ***   Private: CalcAckTimeout   *****************************************
    // *************************************************************************
    // * Acknowledge timeout for the given line rate - see the constants above.
    static uint32_t CalcAckTimeout(uint32_t baud_rate)
    {
      // Time to put a maximum sized frame on the wire, 10 bits per byte
      uint32_t frame_ms = ((MAX_FRAME_SIZE * 10u * 1000u) / baud_rate) + 1u;
      // Two of those plus the margin
      uint32_t timeout_ms = (frame_ms * 2u) + ACK_TIMEOUT_MARGIN_MS;
      // Never go below the minimum
      if(timeout_ms < ACK_TIMEOUT_MIN_MS) timeout_ms = ACK_TIMEOUT_MIN_MS;
      // Return result
      return timeout_ms;
    }

    // *************************************************************************
    // ***   Private: ApplyAckTimeout   ****************************************
    // *************************************************************************
    // * Recomputes the effective timeout from its two inputs. Called whenever
    // * either changes, so the derived value is never left applied on its own
    // * once a floor has been set - and so the floor can only ever raise it.
    void ApplyAckTimeout(void)
    {
      ack_timeout_ms = CalcAckTimeout(applied_baud_rate);
      // A floor, never an override - see SetAckTimeoutFloor()
      if(ack_timeout_ms < ack_min_ms)
      {
        ack_timeout_ms = ack_min_ms;
      }
    }

    // *************************************************************************
    // ***   Private: ReadByte   ***********************************************
    // *************************************************************************
    // * Gives out one byte of the unframed stream without servicing the
    // * protocol, so that a caller can take many bytes for one Service() call.
    Result ReadByte(uint8_t& rx_byte);

    // *************************************************************************
    // ***   Private: Service   ************************************************
    // *************************************************************************
    // * Receives bytes and retransmits unacknowledged frames.
    void Service(void);

    // *************************************************************************
    // ***   Private: ProcessRxByte   ******************************************
    // *************************************************************************
    void ProcessRxByte(uint8_t rx_byte);

    // *************************************************************************
    // ***   Private: ProcessRxFrame   *****************************************
    // *************************************************************************
    void ProcessRxFrame(void);

    // *************************************************************************
    // ***   Private: SendOnChannel   ******************************************
    // *************************************************************************
    Result SendOnChannel(tx_channel_t& ch, uint8_t type, const uint8_t* payload, uint8_t len);

    // *************************************************************************
    // ***   Private: ServiceChannel   *****************************************
    // *************************************************************************
    void ServiceChannel(tx_channel_t& ch);

    // *************************************************************************
    // ***   Private: HandleAckNak   *******************************************
    // *************************************************************************
    void HandleAckNak(tx_channel_t& ch, bool is_ack, uint8_t seq);

    // *************************************************************************
    // ***   Private: RetransmitOrDrop   ***************************************
    // *************************************************************************
    // * Spends one attempt on the frame in flight: retransmits it, or gives
    // * up and declares the link down when the budget is used up. A timeout
    // * and a negative acknowledge are the same event here - both cost an
    // * attempt, which is what bounds a Nak storm.
    void RetransmitOrDrop(tx_channel_t& ch);

    // *************************************************************************
    // ***   Private: SendControlFrame   ***************************************
    // *************************************************************************
    // * Queues an Ack or a Nak - those are never acknowledged themselves.
    void SendControlFrame(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len);

    // *************************************************************************
    // ***   Private: ResetSession   *******************************************
    // *************************************************************************
    void ResetSession(void);

    // *************************************************************************
    // ***   Private: PumpTx   *************************************************
    // *************************************************************************
    void PumpTx(void);

    // *************************************************************************
    // ***   Private: PushRxData   *********************************************
    // *************************************************************************
    // * Puts unframed payload into the receive buffer. Returns false if it
    // * doesn't fit - the frame is then Nak'ed instead of accepted.
    bool PushRxData(const uint8_t* data, uint32_t size, bool resync);

    // *************************************************************************
    // ***   Private: GetRxDataFree   ******************************************
    // *************************************************************************
    uint32_t GetRxDataFree(void);
};

#endif
