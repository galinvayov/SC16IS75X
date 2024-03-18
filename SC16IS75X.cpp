/*
Description:
Library for SC16IS750, SC16IS752, SC16IS762 I2C/SPI to UART bridge IC.

Based on library for SC16IS750 from SandboxElectronix
http://sandboxelectronics.com
Version:
V0.1
Release Date:
2014-02-16
Author:
Tiequan Shao          info@sandboxelectronics.com
Lisence:
CC BY-NC-SA 3.0

Used some parts from library for SC16IS752 from TD-er
https://github.com/TD-er/SC16IS752

Version:
V1.0
Release Date:
2024-03-18
Author:
Galin Vayov          galinvayov@gmail.com
Lisence:
CC BY-NC-SA 4.0

Please keep the above information when you use this code in your project.
*/


//#define SC16IS75X_DEBUG_PRINT
#include <SC16IS75X.h>
#include <SPI.h>
#include <Wire.h>


#ifdef __AVR__
 #define WIRE Wire
#else // Arduino Due
 #define WIRE Wire1
#endif


SC16IS75X::SC16IS75X(uint8_t prtcl, uint8_t addr_sspin, uint8_t channel)
{
    protocol = prtcl;
    if ( protocol == SC16IS75X_PROTOCOL_I2C ) {
		device_address_sspin = (addr_sspin>>1);
	} else {
		device_address_sspin = addr_sspin;
	}
	serial_device_channel = channel;
	peek_flag = 0;
//	timeout = 1000;
}


void SC16IS75X::begin(uint32_t baud)
{
	if ( protocol == SC16IS75X_PROTOCOL_I2C) {
        WIRE.begin();
    } else {
		::pinMode(device_address_sspin, OUTPUT);
   	    ::digitalWrite(device_address_sspin, HIGH);
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV4);
		SPI.setBitOrder(MSBFIRST);
		SPI.begin();

	};
    //no reset, resets the settings of the previously initialized channel of the same IC
    //ResetDevice();
    FIFOEnable(1);
    SetBaudrate(baud);
    SetLine(8,0,1);
}

int SC16IS75X::available(void)
{
    return FIFOAvailableData();
}

int SC16IS75X::read(void)
{
	if ( peek_flag == 0) {
		return ReadByte();
	} else {
		peek_flag = 0;
		return peek_buf;
	}
}

size_t SC16IS75X::write(uint8_t val)
{
    WriteByte(val);
	return 1;
}

void SC16IS75X::pinMode(uint8_t pin, uint8_t i_o)
{
    GPIOSetPinMode(pin, i_o);
}

void SC16IS75X::digitalWrite(uint8_t pin, uint8_t value)
{
    GPIOSetPinState(pin, value);
}

uint8_t SC16IS75X::digitalRead(uint8_t pin)
{
   return GPIOGetPinState(pin);
}


uint8_t SC16IS75X::ReadRegister(uint8_t reg_addr)
{
   uint8_t result;

  if (protocol == SC16IS75X_PROTOCOL_I2C) { // register read operation via I2C
    WIRE.beginTransmission(device_address_sspin);
    WIRE.write((reg_addr << 3 | serial_device_channel << 1));
    WIRE.endTransmission(0);
    WIRE.requestFrom(device_address_sspin, (uint8_t)1);
    result = WIRE.read();
  } else { // register read operation via SPI
    ::digitalWrite(device_address_sspin, LOW);
    delayMicroseconds(10);
    SPI.transfer(0x80 | ((reg_addr << 3 | serial_device_channel << 1)));
    result = SPI.transfer(0xff);
    delayMicroseconds(10);
    ::digitalWrite(device_address_sspin, HIGH);
  }

#ifdef  SC16IS75X_DEBUG_PRINT
  Serial.print("ReadRegister, channel=");
  Serial.print(serial_device_channel,                        HEX);
  Serial.print(" CS=");
  Serial.print(device_address_sspin, DEC);
  Serial.print(" reg_addr=");
  Serial.print(reg_addr, HEX);
  Serial.print(" reg_calc_addr=");
  Serial.print((reg_addr << 3 | serial_device_channel << 1), HEX);
  Serial.print(" result=");
  Serial.println(result, HEX);
#endif // ifdef  SC16IS75X_DEBUG_PRINT
  return result;

}

void SC16IS75X::WriteRegister(uint8_t reg_addr, uint8_t val)
{
    #ifdef  SC16IS75X_DEBUG_PRINT
  Serial.print("WriteRegister, channel=");
  Serial.print(serial_device_channel,                        HEX);
  Serial.print(" CS=");
  Serial.print(device_address_sspin, DEC);
  Serial.print(" reg_addr=");
  Serial.print(reg_addr, HEX);
  Serial.print(" reg_calc_addr=");
  Serial.print((reg_addr << 3 | serial_device_channel << 1), HEX);
  Serial.print(" val=");
  Serial.println(val, HEX);
#endif // ifdef  SC16IS75X_DEBUG_PRINT

  if (protocol == SC16IS75X_PROTOCOL_I2C) { // register read operation via I2C
    WIRE.beginTransmission(device_address_sspin);
    WIRE.write((reg_addr << 3 | serial_device_channel << 1));
    WIRE.write(val);
    WIRE.endTransmission(1);
  } else {
    ::digitalWrite(device_address_sspin, LOW);
    delayMicroseconds(10);
    SPI.transfer((reg_addr << 3 | serial_device_channel << 1));
    SPI.transfer(val);
    delayMicroseconds(10);
    ::digitalWrite(device_address_sspin, HIGH);
  }
}


int16_t SC16IS75X::SetBaudrate(uint32_t baudrate) //return error of baudrate parts per thousand
{
    uint16_t divisor;
    uint8_t prescaler;
    uint32_t actual_baudrate;
    int16_t error;
    uint8_t temp_lcr;
    if ( (ReadRegister(SC16IS75X_REG_MCR)&0x80) == 0) { //if prescaler==1
        prescaler = 1;
    } else {
        prescaler = 4;
    }

    divisor = (SC16IS75X_CRYSTCAL_FREQ/prescaler)/(baudrate*16);

    temp_lcr = ReadRegister(SC16IS75X_REG_LCR);
    temp_lcr |= 0x80;
    WriteRegister(SC16IS75X_REG_LCR,temp_lcr);
    //write to DLL
    WriteRegister(SC16IS75X_REG_DLL,(uint8_t)divisor);
    //write to DLH
    WriteRegister(SC16IS75X_REG_DLH,(uint8_t)(divisor>>8));
    temp_lcr &= 0x7F;
    WriteRegister(SC16IS75X_REG_LCR,temp_lcr);


    actual_baudrate = (SC16IS75X_CRYSTCAL_FREQ/prescaler)/(16*divisor);
    error = ((float)actual_baudrate-baudrate)*1000/baudrate;
#ifdef  SC16IS75X_DEBUG_PRINT
    Serial.print("Desired baudrate: ");
    Serial.println(baudrate,DEC);
    Serial.print("Calculated divisor: ");
    Serial.println(divisor,DEC);
    Serial.print("Actual baudrate: ");
    Serial.println(actual_baudrate,DEC);
    Serial.print("Baudrate error: ");
    Serial.println(error,DEC);
#endif

    return error;

}

void SC16IS75X::SetLine(uint8_t data_length, uint8_t parity_select, uint8_t stop_length )
{
    uint8_t temp_lcr;
    temp_lcr = ReadRegister(SC16IS75X_REG_LCR);
    temp_lcr &= 0xC0; //Clear the lower six bit of LCR (LCR[0] to LCR[5]
#ifdef  SC16IS75X_DEBUG_PRINT
    Serial.print("LCR Register:0x");
    Serial.println(temp_lcr,DEC);
#endif
    switch (data_length) {            //data length settings
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

    if ( stop_length == 2 ) {
        temp_lcr |= 0x04;
    }

    switch (parity_select) {            //parity selection length settings
        case 0:                         //no parity
             break;
        case 1:                         //odd parity
            temp_lcr |= 0x08;
            break;
        case 2:                         //even parity
            temp_lcr |= 0x18;
            break;
        case 3:                         //force '1' parity
            temp_lcr |= 0x03;
            break;
        case 4:                         //force '0' parity
            break;
        default:
            break;
    }

    WriteRegister(SC16IS75X_REG_LCR,temp_lcr);
}

void SC16IS75X::GPIOSetPinMode(uint8_t pin_number, uint8_t i_o)
{
    uint8_t temp_iodir;

    temp_iodir = ReadRegister(SC16IS75X_REG_IODIR);
    if ( i_o == OUTPUT ) {
      temp_iodir |= (0x01 << pin_number);
    } else {
      temp_iodir &= (uint8_t)~(0x01 << pin_number);
    }

    WriteRegister(SC16IS75X_REG_IODIR, temp_iodir);
    return;
}

void SC16IS75X::GPIOSetPinState(uint8_t pin_number, uint8_t pin_state)
{
    uint8_t temp_iostate;

    temp_iostate = ReadRegister(SC16IS75X_REG_IOSTATE);
    if ( pin_state == 1 ) {
      temp_iostate |= (0x01 << pin_number);
    } else {
      temp_iostate &= (uint8_t)~(0x01 << pin_number);
    }

    WriteRegister(SC16IS75X_REG_IOSTATE, temp_iostate);

#ifdef  SC16IS75X_DEBUG_PRINT
  Serial.print("GPIOSetPinState, channel=");
  Serial.print(serial_device_channel,                        HEX);
  Serial.print(" Pin number=");
  Serial.print(pin_number, DEC);
  Serial.print(" Pin state=");
  Serial.println(pin_state, DEC);
  
#endif // ifdef  SC16IS75X_DEBUG_PRINT

   return;
}


uint8_t SC16IS75X::GPIOGetPinState(uint8_t pin_number)
{
    uint8_t temp_iostate;

    temp_iostate = ReadRegister(SC16IS75X_REG_IOSTATE);
    if (( temp_iostate & (0x01 << pin_number))== 0 ) {
      return 0;
    }
    return 1;
}

uint8_t SC16IS75X::GPIOGetPortState(void)
{

    return ReadRegister(SC16IS75X_REG_IOSTATE);

}

void SC16IS75X::GPIOSetPortMode(uint8_t port_io)
{
    WriteRegister(SC16IS75X_REG_IODIR, port_io);
    return;
}

void SC16IS75X::GPIOSetPortState(uint8_t port_state)
{
    WriteRegister(SC16IS75X_REG_IOSTATE, port_state);
    return;
}

void SC16IS75X::SetPinInterrupt(uint8_t io_int_ena)
{
    WriteRegister(SC16IS75X_REG_IOINTENA, io_int_ena);
    return;
}

void SC16IS75X::ResetDevice(void)
{
    uint8_t reg;

    reg = ReadRegister(SC16IS75X_REG_IOCONTROL);
    reg |= 0x08;
    WriteRegister(SC16IS75X_REG_IOCONTROL, reg);

    return;
}

void SC16IS75X::ModemPin(uint8_t gpio) //gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
    uint8_t temp_iocontrol;

    temp_iocontrol = ReadRegister(SC16IS75X_REG_IOCONTROL);
    if ( gpio == 0 ) {
        temp_iocontrol |= 0x02;
    } else {
        temp_iocontrol &= 0xFD;
    }
    WriteRegister(SC16IS75X_REG_IOCONTROL, temp_iocontrol);

    return;
}

void SC16IS75X::GPIOLatch(uint8_t latch)
{
    uint8_t temp_iocontrol;

    temp_iocontrol = ReadRegister(SC16IS75X_REG_IOCONTROL);
    if ( latch == 0 ) {
        temp_iocontrol &= 0xFE;
    } else {
        temp_iocontrol |= 0x01;
    }
    WriteRegister(SC16IS75X_REG_IOCONTROL, temp_iocontrol);

    return;
}

void SC16IS75X::InterruptControl(uint8_t int_ena)
{
    WriteRegister(SC16IS75X_REG_IER, int_ena);
}

uint8_t SC16IS75X::InterruptPendingTest(void)
{
    return (ReadRegister(SC16IS75X_REG_IIR) & 0x01);
}

void SC16IS75X::__isr(void)
{
    uint8_t irq_src;

    irq_src = ReadRegister(SC16IS75X_REG_IIR);
    irq_src = (irq_src >> 1);
    irq_src &= 0x3F;

    switch (irq_src) {
        case 0x06:                  //Receiver Line Status Error
            break;
        case 0x0c:               //Receiver time-out interrupt
            break;
        case 0x04:               //RHR interrupt
            break;
        case 0x02:               //THR interrupt
            break;
        case 0x00:                  //modem interrupt;
            break;
        case 0x30:                  //input pin change of state
            break;
        case 0x10:                  //XOFF
            break;
        case 0x20:                  //CTS,RTS
            break;
        default:
            break;
    }
    return;
}

void SC16IS75X::FIFOEnable(uint8_t fifo_enable)
{
    uint8_t temp_fcr;

    temp_fcr = ReadRegister(SC16IS75X_REG_FCR);

    if (fifo_enable == 0){
        temp_fcr &= 0xFE;
    } else {
        temp_fcr |= 0x01;
    }
    WriteRegister(SC16IS75X_REG_FCR,temp_fcr);

    return;
}

void SC16IS75X::FIFOReset(uint8_t rx_fifo)
{
     uint8_t temp_fcr;

    temp_fcr = ReadRegister(SC16IS75X_REG_FCR);

    if (rx_fifo == 0){
        temp_fcr |= 0x04;
    } else {
        temp_fcr |= 0x02;
    }
    WriteRegister(SC16IS75X_REG_FCR,temp_fcr);

    return;

}

void SC16IS75X::FIFOSetTriggerLevel(uint8_t rx_fifo, uint8_t length)
{
    uint8_t temp_reg;

    temp_reg = ReadRegister(SC16IS75X_REG_MCR);
    temp_reg |= 0x04;
    WriteRegister(SC16IS75X_REG_MCR,temp_reg); //SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

    temp_reg = ReadRegister(SC16IS75X_REG_EFR);
    WriteRegister(SC16IS75X_REG_EFR, temp_reg|0x10); //set ERF[4] to '1' to use the  enhanced features
    if (rx_fifo == 0) {
        WriteRegister(SC16IS75X_REG_TLR, length<<4); //Tx FIFO trigger level setting
    } else {
        WriteRegister(SC16IS75X_REG_TLR, length);    //Rx FIFO Trigger level setting
    }
    WriteRegister(SC16IS75X_REG_EFR, temp_reg); //restore EFR register

    return;
}

uint8_t SC16IS75X::FIFOAvailableData(void)
{
#ifdef  SC16IS75X_DEBUG_PRINT
    Serial.print("=====Available data:");
    Serial.println(ReadRegister(SC16IS75X_REG_RXLVL), DEC);
#endif
   return ReadRegister(SC16IS75X_REG_RXLVL);
//    return ReadRegister(SC16IS75X_REG_LSR) & 0x01;
}

uint8_t SC16IS75X::FIFOAvailableSpace(void)
{
   return ReadRegister(SC16IS75X_REG_TXLVL);

}

void SC16IS75X::WriteByte(uint8_t val)
{
	uint8_t tmp_lsr;
	/*
    while ( FIFOAvailableSpace() == 0 ){
#ifdef  SC16IS75X_DEBUG_PRINT
		Serial.println("No available space");
#endif

	};

#ifdef  SC16IS75X_DEBUG_PRINT
    Serial.println("++++++++++++Data sent");
#endif
    WriteRegister(SC16IS75X_REG_THR,val);
	*/

	do {
		tmp_lsr = ReadRegister(SC16IS75X_REG_LSR);
	} while ((tmp_lsr&0x20) ==0);

	WriteRegister(SC16IS75X_REG_THR,val);



}

int SC16IS75X::ReadByte(void)
{
	volatile uint8_t val;
	if (FIFOAvailableData() == 0) {
#ifdef  SC16IS75X_DEBUG_PRINT
	Serial.println("No data available");
#endif
		return -1;

	} else {

#ifdef  SC16IS75X_DEBUG_PRINT
	Serial.println("***********Data available***********");
#endif
	  val = ReadRegister(SC16IS75X_REG_RHR);
	  return val;
	}


}

void SC16IS75X::EnableTransmit(uint8_t tx_enable)
{
    uint8_t temp_efcr;
    temp_efcr = ReadRegister(SC16IS75X_REG_EFCR);
    if ( tx_enable == 0) {
        temp_efcr |= 0x04;
    } else {
        temp_efcr &= 0xFB;
    }
    WriteRegister(SC16IS75X_REG_EFCR,temp_efcr);

    return;
}

uint8_t SC16IS75X::ping()
{
	WriteRegister(SC16IS75X_REG_SPR,0x55);
	if (ReadRegister(SC16IS75X_REG_SPR) !=0x55) {
		return 0;
	}

	WriteRegister(SC16IS75X_REG_SPR,0xAA);
	if (ReadRegister(SC16IS75X_REG_SPR) !=0xAA) {
		return 0;
	}

	return 1;

}
/*
void SC16IS75X::setTimeout(uint32_t time_out)
{
	timeout = time_out;
}

size_t SC16IS75X::readBytes(char *buffer, size_t length)
{
	size_t count=0;
	int16_t tmp;

	while (count < length) {
		tmp = readwithtimeout();
		if (tmp < 0) {
			break;
		}
		*buffer++ = (char)tmp;
		count++;
	}

	return count;
}

int16_t SC16IS75X::readwithtimeout()
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

void SC16IS75X::flush()
{
	uint8_t tmp_lsr;

	do {
		tmp_lsr = ReadRegister(SC16IS75X_REG_LSR);
	} while ((tmp_lsr&0x20) ==0);


}

int SC16IS75X:: peek()
{
	if ( peek_flag == 0 ) {
		peek_buf =ReadByte();
		if (  peek_buf >= 0 ) {
			peek_flag = 1;
		}
	}

	return peek_buf;

}
