/*
   Description:
   This is a example code for Sandbox Electronics' I2C/SPI to UART bridge module.
   You can get one of those products on
   http://sandboxelectronics.com

   Version:
   V0.1

   Release Date:
   2014-02-16

   Author:
   Tiequan Shao          info@sandboxelectronics.com

   Lisence:
   CC BY-NC-SA 3.0

   Please keep the above information when you use this code in your project.
 */


// #define SC16IS750_DEBUG_PRINT
#include <SC16IS752HPMA.h>

#include <SPI.h>
#include <Wire.h>

#ifdef __AVR__
 # define WIRE Wire
#elif defined(ESP8266) || defined(ESP32) // ESP8266/ESP32
 # define WIRE Wire
#elif ESP32 // ESP8266
 # define WIRE Wire
#else // Arduino Due
 # define WIRE Wire1
#endif // ifdef __AVR__


SC16IS752HPMA::SC16IS752HPMA(uint8_t prtcl, uint8_t addr_sspin) : initialized(false)
{
  protocol = prtcl;

  if (protocol == SC16IS750_PROTOCOL_I2C) {
    // Datasheet uses extra read/write bit to describe I2C address.
    // Actual address in communication has one bit shifted.
    if ((addr_sspin >= 0x48) && (addr_sspin <= 0x57)) {
      device_address_sspin = addr_sspin;
    } else {
      device_address_sspin = (addr_sspin >> 1);
    }
  } else {
    device_address_sspin = addr_sspin;
  }
  peek_flag[SC16IS752_CHANNEL_A] = 0;
  peek_flag[SC16IS752_CHANNEL_B] = 0;

  //	timeout = 1000;
}

void SC16IS752HPMA::begin(uint32_t baud_A, uint32_t baud_B)
{
  Initialize(); // Force initialize, since we're initializing both channels at once
  beginA(baud_A);
  beginB(baud_B);
}

void SC16IS752HPMA::beginA(uint32_t baud_A)
{
  if (!initialized) {
    Initialize();
  }
  FIFOEnable(SC16IS752_CHANNEL_A, 1);
  SetBaudrate(SC16IS752_CHANNEL_A, baud_A);
  SetLine(SC16IS752_CHANNEL_A, 8, 0, 1);
}

void SC16IS752HPMA::beginB(uint32_t baud_B)
{
  if (!initialized) {
    Initialize();
  }
  FIFOEnable(SC16IS752_CHANNEL_B, 1);
  SetBaudrate(SC16IS752_CHANNEL_B, baud_B);
  SetLine(SC16IS752_CHANNEL_B, 8, 0, 1);
}

int SC16IS752HPMA::available(uint8_t channel)
{
  return FIFOAvailableData(channel);
}

int SC16IS752HPMA::read(uint8_t channel)
{
  if (peek_flag[channel] == 0) {
    return ReadByte(channel);
  }
  peek_flag[channel] = 0;
  return peek_buf[channel];
}

size_t SC16IS752HPMA::write(uint8_t channel, uint8_t val)
{
  WriteByte(channel, val);
  return 1;
}

void SC16IS752HPMA::pinMode(uint8_t pin, uint8_t i_o)
{
  GPIOSetPinMode(pin, i_o);
}

void SC16IS752HPMA::digitalWrite(uint8_t pin, uint8_t value)
{
  GPIOSetPinState(pin, value);
}

uint8_t SC16IS752HPMA::digitalRead(uint8_t pin)
{
  return GPIOGetPinState(pin);
}

uint8_t SC16IS752HPMA::ReadRegister(uint8_t channel, uint8_t reg_addr)
{
  uint8_t result = 0;

  if (protocol == SC16IS750_PROTOCOL_I2C) { // register read operation via I2C
    WIRE.beginTransmission(device_address_sspin);
    WIRE.write((reg_addr << 3 | channel << 1));
    WIRE.endTransmission(0);
    WIRE.requestFrom(device_address_sspin, (uint8_t)1);
    result = WIRE.read();
  } else if (protocol == SC16IS750_PROTOCOL_SPI) { // register read operation via SPI
    ::digitalWrite(device_address_sspin, LOW);
    delayMicroseconds(10);
    SPI.transfer(0x80 | ((reg_addr << 3 | channel << 1)));
    result = SPI.transfer(0xff);
    delayMicroseconds(10);
    ::digitalWrite(device_address_sspin, HIGH);
  }

#ifdef  SC16IS750_DEBUG_PRINT
  Serial.print("ReadRegister channel=");
  Serial.print(channel,                        HEX);
  Serial.print(" reg_addr=");
  Serial.print((reg_addr << 3 | channel << 1), HEX);
  Serial.print(" result=");
  Serial.println(result, HEX);
#endif // ifdef  SC16IS750_DEBUG_PRINT
  return result;
}

void SC16IS752HPMA::WriteRegister(uint8_t channel, uint8_t reg_addr, uint8_t val)
{
#ifdef  SC16IS750_DEBUG_PRINT
  Serial.print("WriteRegister channel=");
  Serial.print(channel,                        HEX);
  Serial.print(" reg_addr=");
  Serial.print((reg_addr << 3 | channel << 1), HEX);
  Serial.print(" val=");
  Serial.println(val, HEX);
#endif // ifdef  SC16IS750_DEBUG_PRINT

  if (protocol == SC16IS750_PROTOCOL_I2C) { // register read operation via I2C
    WIRE.beginTransmission(device_address_sspin);
    WIRE.write((reg_addr << 3 | channel << 1));
    WIRE.write(val);
    WIRE.endTransmission(1);
  } else {
    ::digitalWrite(device_address_sspin, LOW);
    delayMicroseconds(10);
    SPI.transfer((reg_addr << 3 | channel << 1));
    SPI.transfer(val);
    delayMicroseconds(10);
    ::digitalWrite(device_address_sspin, HIGH);
  }
}

void SC16IS752HPMA::Initialize()
{
  if (protocol == SC16IS750_PROTOCOL_I2C) {
    WIRE.begin();
  } else {
    ::pinMode(device_address_sspin, OUTPUT);
    ::digitalWrite(device_address_sspin, HIGH);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.setBitOrder(MSBFIRST);
    SPI.begin();

    // SPI.setClockDivider(32);
  }
  ResetDevice();
  initialized = true;
}

int16_t SC16IS752HPMA::SetBaudrate(uint8_t channel, uint32_t baudrate) // return error of baudrate parts per thousand
{
  uint16_t divisor;
  uint8_t  prescaler;
  uint32_t actual_baudrate;
  int16_t  error;
  uint8_t  temp_lcr;

  if ((ReadRegister(channel, SC16IS750_REG_MCR) & 0x80) == 0) { // if prescaler==1
    prescaler = 1;
  } else {
    prescaler = 4;
  }

  divisor = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (baudrate * 16);

  temp_lcr  = ReadRegister(channel, SC16IS750_REG_LCR);
  temp_lcr |= 0x80;
  WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);

  // write to DLL
  WriteRegister(channel, SC16IS750_REG_DLL,        (uint8_t)divisor);

  // write to DLH
  WriteRegister(channel, SC16IS750_REG_DLH,        (uint8_t)(divisor >> 8));
  temp_lcr &= 0x7F;
  WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);


  actual_baudrate = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (16 * divisor);
  error           = ((float)actual_baudrate - baudrate) * 1000 / baudrate;
#ifdef  SC16IS750_DEBUG_PRINT
  Serial.print("Desired baudrate: ");
  Serial.println(baudrate, DEC);
  Serial.print("Calculated divisor: ");
  Serial.println(divisor, DEC);
  Serial.print("Actual baudrate: ");
  Serial.println(actual_baudrate, DEC);
  Serial.print("Baudrate error: ");
  Serial.println(error, DEC);
#endif // ifdef  SC16IS750_DEBUG_PRINT

  return error;
}

void SC16IS752HPMA::SetLine(uint8_t channel, uint8_t data_length, uint8_t parity_select, uint8_t stop_length)
{
  uint8_t temp_lcr;

  temp_lcr  = ReadRegister(channel, SC16IS750_REG_LCR);
  temp_lcr &= 0xC0; // Clear the lower six bit of LCR (LCR[0] to LCR[5]
#ifdef  SC16IS750_DEBUG_PRINT
  Serial.print("LCR Register:0x");
  Serial.println(temp_lcr, DEC);
#endif // ifdef  SC16IS750_DEBUG_PRINT

  switch (data_length) { // data length settings
    case 5:
      break;
    case 6:
      temp_lcr |= 0x01;
      break;
    case 7:
      temp_lcr |= 0x02;
      break;
    case 8:
      temp_lcr |= 0x03;
      break;
    default:
      temp_lcr |= 0x03;
      break;
  }

  if (stop_length == 2) {
    temp_lcr |= 0x04;
  }

  switch (parity_select) { // parity selection length settings
    case 0:                // no parity
      break;
    case 1:                // odd parity
      temp_lcr |= 0x08;
      break;
    case 2:                // even parity
      temp_lcr |= 0x18;
      break;
    case 3:                // force '1' parity
      temp_lcr |= 0x03;
      break;
    case 4:                // force '0' parity
      break;
    default:
      break;
  }

  WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);
}

void SC16IS752HPMA::GPIOSetPinMode(uint8_t pin_number, uint8_t i_o)
{
  uint8_t temp_iodir;

  temp_iodir = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IODIR);

  if (i_o == OUTPUT) {
    temp_iodir |= (0x01 << pin_number);
  } else {
    temp_iodir &= (uint8_t) ~(0x01 << pin_number);
  }

  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IODIR, temp_iodir);
}

void SC16IS752HPMA::GPIOSetPinState(uint8_t pin_number, uint8_t pin_state)
{
  uint8_t temp_iostate;

  temp_iostate = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);

  if (pin_state == 1) {
    temp_iostate |= (0x01 << pin_number);
  } else {
    temp_iostate &= (uint8_t) ~(0x01 << pin_number);
  }

  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, temp_iostate);
}

uint8_t SC16IS752HPMA::GPIOGetPinState(uint8_t pin_number)
{
  uint8_t temp_iostate;

  temp_iostate = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);

  if ((temp_iostate & (0x01 << pin_number)) == 0) {
    return 0;
  }
  return 1;
}

uint8_t SC16IS752HPMA::GPIOGetPortState(void)
{
  return ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
}

void SC16IS752HPMA::GPIOSetPortMode(uint8_t port_io)
{
  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IODIR, port_io);
}

void SC16IS752HPMA::GPIOSetPortState(uint8_t port_state)
{
  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, port_state);
}

void SC16IS752HPMA::SetPinInterrupt(uint8_t pin_number, bool int_ena)
{ 
  uint8_t temp_iostate;

  temp_iostate = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOINTENA);

  if (int_ena == 1) {
    temp_iostate |= (0x01 << pin_number);
  } else {
    temp_iostate &= (uint8_t) ~(0x01 << pin_number);
  }

  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOINTENA, temp_iostate);
}

uint8_t SC16IS752HPMA::GetPinInterrupt(uint8_t pin_number)
{
  uint8_t temp_iostate;

  temp_iostate = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOINTENA);

  if ((temp_iostate & (0x01 << pin_number)) == 0) {
    return 0;
  }
  return 1;
}

void SC16IS752HPMA::ResetDevice()
{
  uint8_t reg;

  reg  = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
  reg |= 0x08;
  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, reg);
}

void SC16IS752HPMA::ModemPin(uint8_t gpio) // gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
  uint8_t temp_iocontrol;

  temp_iocontrol = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);

  if (gpio == 0) {
    temp_iocontrol |= 0x02;
  } else {
    temp_iocontrol &= 0xFD;
  }
  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);
}

void SC16IS752HPMA::GPIOLatch(uint8_t latch)
{
  uint8_t temp_iocontrol;

  temp_iocontrol = ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);

  if (latch == 0) {
    temp_iocontrol &= 0xFE;
  } else {
    temp_iocontrol |= 0x01;
  }
  WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);
}

void SC16IS752HPMA::InterruptControl(uint8_t channel, uint8_t int_ena)
{
  WriteRegister(channel, SC16IS750_REG_IER, int_ena);
}

uint8_t SC16IS752HPMA::InterruptPendingTest(uint8_t channel)
{
  return ReadRegister(channel, SC16IS750_REG_IIR) & 0x01;
}

void SC16IS752HPMA::__isr(uint8_t channel)
{
  uint8_t irq_src;

  irq_src  = ReadRegister(channel, SC16IS750_REG_IIR);
  irq_src  = (irq_src >> 1);
  irq_src &= 0x3F;

  switch (irq_src) {
    case 0x06: // Receiver Line Status Error
      break;
    case 0x0c: // Receiver time-out interrupt
      break;
    case 0x04: // RHR interrupt
      break;
    case 0x02: // THR interrupt
      break;
    case 0x00: // modem interrupt;
      break;
    case 0x30: // input pin change of state
      break;
    case 0x10: // XOFF
      break;
    case 0x20: // CTS,RTS
      break;
    default:
      break;
  }
}

void SC16IS752HPMA::FIFOEnable(uint8_t channel, uint8_t fifo_enable)
{
  uint8_t temp_fcr;

  temp_fcr = ReadRegister(channel, SC16IS750_REG_FCR);

  if (fifo_enable == 0) {
    temp_fcr &= 0xFE;
  } else {
    temp_fcr |= 0x01;
  }
  WriteRegister(channel, SC16IS750_REG_FCR, temp_fcr);
}

void SC16IS752HPMA::FIFOReset(uint8_t channel, uint8_t rx_fifo)
{
  uint8_t temp_fcr;

  temp_fcr = ReadRegister(channel, SC16IS750_REG_FCR);

  if (rx_fifo == 0) {
    temp_fcr |= 0x04;
  } else {
    temp_fcr |= 0x02;
  }
  WriteRegister(channel, SC16IS750_REG_FCR, temp_fcr);
}

void SC16IS752HPMA::FIFOSetTriggerLevel(uint8_t channel, uint8_t rx_fifo, uint8_t length)
{
  uint8_t temp_reg;

  temp_reg  = ReadRegister(channel, SC16IS750_REG_MCR);
  temp_reg |= 0x04;
  WriteRegister(channel, SC16IS750_REG_MCR, temp_reg);        // SET MCR[2] to '1' to use TLR register or trigger level control in FCR
                                                              // register

  temp_reg = ReadRegister(channel, SC16IS750_REG_EFR);
  WriteRegister(channel, SC16IS750_REG_EFR, temp_reg | 0x10); // set ERF[4] to '1' to use the  enhanced features

  if (rx_fifo == 0) {
    WriteRegister(channel, SC16IS750_REG_TLR, length << 4);   // Tx FIFO trigger level setting
  } else {
    WriteRegister(channel, SC16IS750_REG_TLR, length);        // Rx FIFO Trigger level setting
  }
  WriteRegister(channel, SC16IS750_REG_EFR, temp_reg);        // restore EFR register
}

uint8_t SC16IS752HPMA::FIFOAvailableData(uint8_t channel)
{
#ifdef  SC16IS750_DEBUG_PRINT
  Serial.print("=====Available data:");
  Serial.println(ReadRegister(channel, SC16IS750_REG_RXLVL), DEC);
#endif // ifdef  SC16IS750_DEBUG_PRINT
  if (fifo_available[channel] == 0) {
    fifo_available[channel] = ReadRegister(channel, SC16IS750_REG_RXLVL);
  }
  return fifo_available[channel];
  //    return ReadRegister(channel, SC16IS750_REG_LSR) & 0x01;
}

uint8_t SC16IS752HPMA::FIFOAvailableSpace(uint8_t channel)
{
  return ReadRegister(channel, SC16IS750_REG_TXLVL);
}

void SC16IS752HPMA::WriteByte(uint8_t channel, uint8_t val)
{
  uint8_t tmp_lsr;

  /*   while ( FIFOAvailableSpace(channel) == 0 ){
   #ifdef  SC16IS750_DEBUG_PRINT
                 Serial.println("No available space");
   #endif

         };

   #ifdef  SC16IS750_DEBUG_PRINT
     Serial.println("++++++++++++Data sent");
   #endif
     WriteRegister(SC16IS750_REG_THR,val);
   */
  do {
    tmp_lsr = ReadRegister(channel, SC16IS750_REG_LSR);
  } while ((tmp_lsr & 0x20) == 0);

  WriteRegister(channel, SC16IS750_REG_THR, val);
}

int SC16IS752HPMA::ReadByte(uint8_t channel)
{
  volatile uint8_t val;

  if (FIFOAvailableData(channel) == 0) {
#ifdef  SC16IS750_DEBUG_PRINT
    Serial.println("No data available");
#endif // ifdef  SC16IS750_DEBUG_PRINT
    return -1;
  } else {
#ifdef  SC16IS750_DEBUG_PRINT
    Serial.println("***********Data available***********");
#endif // ifdef  SC16IS750_DEBUG_PRINT
    if (fifo_available[channel] > 0) {
      --fifo_available[channel];
    }
    val = ReadRegister(channel, SC16IS750_REG_RHR);
    return val;
  }
}

void SC16IS752HPMA::EnableTransmit(uint8_t channel, uint8_t tx_enable)
{
  uint8_t temp_efcr;

  temp_efcr = ReadRegister(channel, SC16IS750_REG_EFCR);

  if (tx_enable == 0) {
    temp_efcr |= 0x04;
  } else {
    temp_efcr &= 0xFB;
  }
  WriteRegister(channel, SC16IS750_REG_EFCR, temp_efcr);
}

uint8_t SC16IS752HPMA::ping()
{
  WriteRegister(SC16IS752_CHANNEL_A, SC16IS750_REG_SPR, 0x55);

  if (ReadRegister(SC16IS752_CHANNEL_A, SC16IS750_REG_SPR) != 0x55) {
    return 0;
  }

  WriteRegister(SC16IS752_CHANNEL_A, SC16IS750_REG_SPR, 0xAA);

  if (ReadRegister(SC16IS752_CHANNEL_A, SC16IS750_REG_SPR) != 0xAA) {
    return 0;
  }

  WriteRegister(SC16IS752_CHANNEL_B, SC16IS750_REG_SPR, 0x55);

  if (ReadRegister(SC16IS752_CHANNEL_B, SC16IS750_REG_SPR) != 0x55) {
    return 0;
  }

  WriteRegister(SC16IS752_CHANNEL_B, SC16IS750_REG_SPR, 0xAA);

  if (ReadRegister(SC16IS752_CHANNEL_B, SC16IS750_REG_SPR) != 0xAA) {
    return 0;
  }

  return 1;
}

/*
   void SC16IS752HPMA::setTimeout(uint32_t time_out)
   {
        timeout = time_out;
   }
 */
 
size_t SC16IS752HPMA::readBytes(uint8_t channel, uint8_t *buffer, size_t length)
{
  size_t count=0;
  int16_t tmp;

  while (count < length) {
	tmp = ReadByte(channel);
	if (tmp < 0) {
			break;
	}
	*buffer++ = tmp;
	count++;
  }
  return count;
}
   
String SC16IS752HPMA::readStringUntil(uint8_t channel, char terminator)
{
  String ret;
  int c = ReadByte(channel);
  while(c >= 0 && c != terminator) {
	ret += (char) c;
	c = ReadByte(channel);
  }
  return ret;
}
   
/*
   int16_t SC16IS752HPMA::readwithtimeout()
   {
   int16_t tmp;
   uint32_t time_stamp;
   time_stamp = millis();
   do {
    tmp = read();
    if (tmp >= 0) return tmp;
   } while(millis() - time_stamp < timeout);
   return -1;     // -1 indicates timeout
   }
 */
void SC16IS752HPMA::flush(uint8_t channel)
{
  uint8_t tmp_lsr;

  do {
    tmp_lsr = ReadRegister(channel, SC16IS750_REG_LSR);
  } while ((tmp_lsr & 0x20) == 0);
}

int SC16IS752HPMA::peek(uint8_t channel)
{
  if (peek_flag[channel] == 0) {
    peek_buf[channel] = ReadByte(channel);

    if (peek_buf[channel] >= 0) {
      peek_flag[channel] = 1;
    }
  }

  return peek_buf[channel];
}

void SC16IS752HPMA::readUntilUnavailable(uint8_t channel) {
  while(available(channel)) {
    read(channel);
  }
}

void SC16IS752HPMA::SendHPMACommand(uint8_t channel, unsigned char * cmdBuf, unsigned int cmdSize) {
  readUntilUnavailable(channel);

  for(int i = 0; i < cmdSize; i++) {
    write(channel, cmdBuf[i]);
  }
  delay(10);
}

void SC16IS752HPMA::initHPMA(uint8_t channel) {
  delay(100);
  startHPMAParticleMeasurement(channel);
}

void SC16IS752HPMA::startHPMAParticleMeasurement(uint8_t channel) {
	unsigned char startCmd[] = {0x68, 0x01, 0x01, 0x96};
  SendHPMACommand(channel, startCmd, 4);
}

int SC16IS752HPMA::readHPMAResp(uint8_t channel, unsigned char * dataBuf, unsigned int dataBufSize, unsigned int cmdType) {
  static unsigned char respBuf[HPM_MAX_RESP_SIZE];
  static unsigned int respIdx = 0;
  static unsigned int calChecksum = 0;

  //Read response
  respIdx = 0;
  calChecksum = 0;
  memset(respBuf, 0, sizeof(respBuf));

  delay(100);

  Serial.println("PS- Waiting for cmd resp...");
  if (readStringUntil(channel, HPM_CMD_RESP_HEAD)) {
    delay(1); //wait for the rest of the bytes to arrive
    respBuf[HPM_HEAD_IDX] = HPM_CMD_RESP_HEAD;
    respBuf[HPM_LEN_IDX] = read(channel); //Read the command length

    //Ensure buffers are big enough
    if (respBuf[HPM_LEN_IDX] && ((respBuf[HPM_LEN_IDX] + 1) <=  sizeof(respBuf) - 2) && (respBuf[HPM_LEN_IDX] - 1) <= dataBufSize ) {
      if (readBytes(channel, &respBuf[HPM_CMD_IDX], respBuf[HPM_LEN_IDX] + 1) == (respBuf[HPM_LEN_IDX] + 1)) { //read respBuf[HPM_LEN_IDX] num of bytes + calChecksum byte
        if (respBuf[HPM_CMD_IDX] == cmdType) { //check if CMD type matches

          //Calculate and validate checksum
          for (respIdx = 0; respIdx < (2 + respBuf[HPM_LEN_IDX]); respIdx++) {
            calChecksum += respBuf[respIdx];
          }
          calChecksum = (65536 - calChecksum) % 256;
          if (calChecksum == respBuf[2 + respBuf[HPM_LEN_IDX]]) {
            Serial.println("PS- Received valid data!!!");
            memset(dataBuf, 0, dataBufSize);
            memcpy(dataBuf, &respBuf[HPM_DATA_START_IDX], respBuf[HPM_LEN_IDX] - 1);
            return (respBuf[HPM_LEN_IDX] - 1);
          }
        }
      }
    }
  }
  return false;
}


boolean SC16IS752HPMA::readHPMAParticleMeasurement(uint8_t channel, unsigned int * pm2_5, unsigned int * pm10) {
  static unsigned char dataBuf[HPM_READ_PARTICLE_MEASURMENT_LEN - 1];
  unsigned char readCmd[] = {0x68, 0x01, 0x04, 0x93};
  SendHPMACommand(channel, readCmd, 4);
  if (readHPMAResp(channel, dataBuf, sizeof(dataBuf), READ_PARTICLE_MEASURMENT) == (HPM_READ_PARTICLE_MEASURMENT_LEN - 1)) {
    _pm2_5 = dataBuf[0] * 256 + dataBuf[1];
    _pm10 = dataBuf[2] * 256 + dataBuf[3];
    *pm2_5 = _pm2_5;
    *pm10 = _pm10;
    // Serial.println("PS- PM 2.5: " + String(_pm2_5) + " ug/m3" );
    // Serial.println("PS- PM 10: " + String(_pm10) + " ug/m3" );
    return true;
  }
  return false;
}

void SC16IS752HPMA::startCOParticleMeasurement(uint8_t channel) {
  delay(100);
  unsigned char startCmd[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
  SendHPMACommand(channel, startCmd, 9);
}

void SC16IS752HPMA::startReadCOParticleMeasurement(uint8_t channel) {
  unsigned char startReadCmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  SendHPMACommand(channel, startReadCmd, 9);
  delay(100);
}

int* SC16IS752HPMA::readCOParticleMeasurement(uint8_t channel) {
  for(int count = 0; count < 9; count++) {
    _co_result[count] = ReadByte(channel);
  }
  return _co_result;
}

unsigned char SC16IS752HPMA::COCheckSum(unsigned char *i, unsigned char ln)
{
  unsigned char j, tempq = 0;
  i += 1;
  for (j = 0; j < (ln - 2); j++)
  {
    tempq += *i;
    i++;
  }
  tempq = (~tempq) + 1;
  return (tempq);
}

void SC16IS752HPMA::stopHPMAParticleMeasurement(uint8_t channel) {
	unsigned char stopCmd[] = {0x68, 0x01, 0x02, 0x95};
  SendHPMACommand(channel, stopCmd, 4);
}
void SC16IS752HPMA::enableHPMAAutoSend(uint8_t channel) {
	unsigned char enableCmd[] = {0x68, 0x01, 0x40, 0x57};
  SendHPMACommand(channel, enableCmd, 4);
}
void SC16IS752HPMA::disableHPMAAutoSend(uint8_t channel) {
	unsigned char disableCmd[] = {0x68, 0x01, 0x20, 0x77};
  SendHPMACommand(channel, disableCmd, 4);
}
unsigned int SC16IS752HPMA::GetHPMAPM2_5(uint8_t channel) {
  return _pm2_5;
}
unsigned int SC16IS752HPMA::GetHPMAPM10(uint8_t channel) {
  return _pm10;
}
int* SC16IS752HPMA::GetCOResult(uint8_t channel) {
  return _co_result;
}