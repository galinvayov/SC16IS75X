#include <SPI.h>
#include <Wire.h>
#include <SC16IS75X.h>
#include <string.h>

// for SC16IS750, single channel
SC16IS75X i2cuart = SC16IS75X(SC16IS75X_PROTOCOL_I2C, SC16IS75X_ADDRESS_AD, SC16IS750_SINGLE);

/*
Example for SC16IS752, SC16IS762, dual channel
SC16IS75X i2cuart_a = SC16IS75X(SC16IS75X_PROTOCOL_I2C, SC16IS75X_ADDRESS_AD, SC16IS752_CHANNEL_A);
SC16IS75X i2cuart_b = SC16IS75X(SC16IS75X_PROTOCOL_I2C, SC16IS75X_ADDRESS_AD, SC16IS752_CHANNEL_B);
*/

void setup() 
{
    //delay(500);
    Serial.begin(9600);
    Serial.println("Start testing");
    // UART to Serial Bridge Initialization
    i2cuart.begin(9600);               //baudrate setting
    Serial.println("BAUDRATE SET");
    if (i2cuart.ping()!=1) {
        Serial.println("Device not found");
        while(1);
    } else {
        Serial.println("Device found");
    }
	i2cuart.pinMode(7, OUTPUT);
    i2cuart.digitalWrite(7, LOW);
    Serial.println("Start pin 7 togle");
};

void loop() 
{
 
    i2cuart.digitalWrite(7, !i2cuart.digitalRead(7)); // togle pin 7
    
    delay(1000);
 
   
};

