#include <xc.h>
#include "RFM69.h"
#include "RFM69registers.h"

#define false   0
#define true    1

void RFM69(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW){
  //_slaveSelectPin = slaveSelectPin;     // TODO: NEEDS LOOKING INTO
  //_interruptPin = interruptPin;         // TODO: NEEDS LOOKING INTO
    _mode = RF69_MODE_STANDBY;
    _promiscuousMode = false;
    _powerLevel = 31;
    isRFM69HW = isRFM69HW;
#if defined(RF69_LISTENMODE_ENABLE)
    _isHighSpeed = true;
    _haveEncryptKey = false;
    uint32_t rxDuration = DEFAULT_LISTEN_RX_US;
    uint32_t idleDuration = DEFAULT_LISTEN_IDLE_US;
    listenModeSetDurations(rxDuration, idleDuration);
#endif
}

void RFM69BLANK(){
  //_slaveSelectPin = 0;     // TODO: NEEDS LOOKING INTO
  //_interruptPin = 0;         // TODO: NEEDS LOOKING INTO
  _mode = RF69_MODE_STANDBY;
  _promiscuousMode = false;
  _powerLevel = 31;
  isRFM69HW = 0;
#if defined(RF69_LISTENMODE_ENABLE)
  _isHighSpeed = true;
  _haveEncryptKey = false;
  uint32_t rxDuration = DEFAULT_LISTEN_RX_US;
  uint32_t idleDuration = DEFAULT_LISTEN_IDLE_US;
  listenModeSetDurations(rxDuration, idleDuration);
#endif
}

bool RFM69Initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
    _LATB14 = 1;
  //_interruptNum = digitalPinToInterrupt(_interruptPin);
  //if (_interruptNum == NOT_AN_INTERRUPT) return false;
#ifdef RF69_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    _interruptNum = _interruptPin;
#endif
    const uint8_t CONFIG[][2] =
    {
      /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
      /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
      /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
      /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
      /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
      /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

      /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
      /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
      /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

      // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
      // +17dBm and +20dBm are possible on RFM69HW
      // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
      // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
      // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
      ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
      ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

      // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
      /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
      //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
      /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
      /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
      /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
      /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
      ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
      /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
      /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
      /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
      /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
      /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
      ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
      /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
      /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
      //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
      /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
      {255, 0}
    };

     CSEE = 1;                             // REMOVED: digitalWrite(_slaveSelectPin, HIGH);
                                        // REMOVED: pinMode(_slaveSelectPin, OUTPUT);
                                        // REMOVED: SPI.begin();
  //unsigned long start = millis();
  //uint8_t timeout = 50;
  
   //putU1S("RFM69 initialised aa.\n\r");     
  //do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  //do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa);
    while(readReg(REG_SYNCVALUE1) != 0xaa){
        writeReg(REG_SYNCVALUE1, 0xaa);
    }

    while (readReg(REG_SYNCVALUE1) != 0x55){
        writeReg(REG_SYNCVALUE1, 0x55);
    }
  
   //putU1S("RFM69 initialised a.\n\r");     
  
  //start = millis();
  //do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);
  //do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55);

    uint8_t i;
    for (i = 0; CONFIG[i][0] != 255; i++)
        writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
    encrypt(0);

    setHighPower(isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
    setMode(RF69_MODE_STANDBY);
    
    //start = millis();
    //while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
    while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)){} // wait for ModeReady
    //if (millis()-start >= timeout)
   //   return false;
    _inISR = false;
    //attachInterrupt(_interruptNum, RFM69::isr0, RISING);          // TODO: Do something about this

    //selfPointer = this;                 // REMOVED
    address = nodeID;
  #if defined(RF69_LISTENMODE_ENABLE)
    _freqBand = freqBand;
    _networkID = networkID;
  #endif
    return true;
}

void writeReg(uint8_t addr, uint8_t value)
{
    select();
   // putU1S("RFM69 initialised F.\n\r");     
    WriteSPI2(addr | 0x80);           // REMOVED: SPI.transfer(addr | 0x80);
    WriteSPI2(value);                 // REMOVED: SPI.transfer(value);
    unselect();
}

uint8_t readReg(uint8_t addr)
{
    select();
    WriteSPI2(addr & 0x7F);           // REMOVED: SPI.transfer(addr & 0x7F);
    uint8_t regval = WriteSPI2(0);    // REMOVED: uint8_t regval = SPI.transfer(0);
    unselect();
    return regval;
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void select() {
  //noInterrupts();                       // TODO: Do something about this!
#if defined (SPCR) && defined (SPSR)
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
#endif
  // set RFM69 SPI settings
  //SPI.setDataMode(SPI_MODE0);
  //SPI.setBitOrder(MSBFIRST);
#ifdef __arm__
	SPI.setClockDivider(SPI_CLOCK_DIV16);
#else
  //SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
#endif
  //digitalWrite(_slaveSelectPin, LOW);
    CSEE = 0;
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void unselect() {
    CSEE = 1;                     // REMOVED: digitalWrite(_slaveSelectPin, HIGH);
  // restore SPI settings to what they were before talking to RFM69
#if defined (SPCR) && defined (SPSR)
  SPCR = _SPCR;
  SPSR = _SPSR;
#endif
  //maybeInterrupts();          // TODO: Check if needed
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: radio.encrypt(null) or radio.encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void encrypt(const char* key) {
#if defined(RF69_LISTENMODE_ENABLE)
    _haveEncryptKey = key;
#endif
    setMode(RF69_MODE_STANDBY);
    if (key != 0)
    {
#if defined(RF69_LISTENMODE_ENABLE)
        memcpy(_encryptKey, key, 16);
#endif
        select();
        WriteSPI2(REG_AESKEY1 | 0x80);
        uint8_t i;
        for (i = 0; i < 16; i++)
            WriteSPI2(key[i]);
        unselect();
    }
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}


void setMode(uint8_t newMode)
{
    if (newMode == _mode)
        return;

    switch (newMode) {
        case RF69_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            if (isRFM69HW)
                setHighPowerRegs(true);
            break;
        case RF69_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            if (isRFM69HW)
                setHighPowerRegs(false);
            break;
        case RF69_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default:
            return;
    }

    // we are using packet mode, so this check is not really needed
    // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00){} // wait for ModeReady

    _mode = newMode;
}

// internal function
void setHighPowerRegs(bool onOff) {
    writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
    writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

// for RFM69HW only: you must call setHighPower(true) after initialize() or else transmission won't work
void setHighPower(bool onOff) {
    isRFM69HW = onOff;
    writeReg(REG_OCP, isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
    if (isRFM69HW) // turning ON
        writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    else
        writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

// get the received signal strength indicator (RSSI)
int16_t readRSSI(bool forceTrigger) {
    int16_t rssi = 0;
    
    if (forceTrigger)
    {
      // RSSI trigger not needed if DAGC is in continuous mode
        writeReg(REG_RSSICONFIG, RF_RSSI_START);
        while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rssi = -readReg(REG_RSSIVALUE);
    rssi >>= 1;
    return rssi;
}


bool canSend()
{
    if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
    {
        setMode(RF69_MODE_STANDBY);
        return true;
    }
    return false;
}

// internal function
void sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
    setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
    while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00){} // wait for ModeReady
    //putU1S("RFM69 initialised D.\n\r");  
    writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    if (bufferSize > RF69_MAX_DATA_LEN)
        bufferSize = RF69_MAX_DATA_LEN;

    // control byte
    uint8_t CTLbyte = 0x00;
    if (sendACK)
        CTLbyte = RFM69_CTL_SENDACK;
    else if (requestACK)
        CTLbyte = RFM69_CTL_REQACK;

    // write to FIFO
    select();
    WriteSPI2(REG_FIFO | 0x80);
    WriteSPI2(bufferSize + 3);
    WriteSPI2(toAddress);
    WriteSPI2(address);
    WriteSPI2(CTLbyte);

    uint8_t i;
    for (i = 0; i < bufferSize; i++)
        WriteSPI2(((uint8_t*) buffer)[i]);
    unselect();

    // no need to wait for transmit mode to be ready since its handled by the radio
    setMode(RF69_MODE_TX);
    //uint32_t txStart = millis();
    //putU1S("RFM69 initialised DD.\n\r");  
    while (_interruptPin == 0){} // wait for DIO0 to turn HIGH signalling transmission finish
    //putU1S("RFM69 initialised DDD.\n\r");  
    //while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
    setMode(RF69_MODE_STANDBY);
}

void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    //uint32_t now = millis();
    //putU1S("RFM69 initialised SS.\n\r");  
    while (!canSend())
        receiveDone();
    //putU1S("RFM69 initialised S.\n\r");  
    sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// internal function
void receiveBegin() {
    DATALEN = 0;
    SENDERID = 0;
    TARGETID = 0;
    PAYLOADLEN = 0;
    ACK_REQUESTED = 0;
    ACK_RECEIVED = 0;
#if defined(RF69_LISTENMODE_ENABLE)
    RF69_LISTEN_BURST_REMAINING_MS = 0;
#endif
    RSSI = 0;
    if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
        writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
    setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool receiveDone() {
//ATOMIC_BLOCK(ATOMIC_FORCEON)
//{
  //noInterrupts(); // re-enabled in unselect() via setMode() or via receiveBegin()
    if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
    {
        setMode(RF69_MODE_STANDBY); // enables interrupts
        return true;
    }
    else if (_mode == RF69_MODE_RX) // already in RX no payload yet
    {
        //interrupts(); // explicitly re-enable interrupts
        return false;
    }
    receiveBegin();
    return false;
//}
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void RFM69sleep() {
    setMode(RF69_MODE_SLEEP);
}

// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
void setPowerLevel(uint8_t powerLevel)
{
    _powerLevel = (powerLevel > 31 ? 31 : powerLevel);
    if (isRFM69HW)
        _powerLevel /= 2;
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}