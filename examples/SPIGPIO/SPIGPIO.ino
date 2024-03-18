#include <SPI.h>
#include <Wire.h>
#include <SC16IS75X.h>
#include <string.h>

// for SC16IS750, single channel
SC16IS75X spiuart = SC16IS75X(SC16IS75X_PROTOCOL_SPI, SS, SC16IS750_SINGLE);

/*
Example for SC16IS752, SC16IS762, dual channel
SC16IS75X spiuart_a = SC16IS75X(SC16IS75X_PROTOCOL_SPI, SS, SC16IS752_CHANNEL_A);
SC16IS75X spiuart_b = SC16IS75X(SC16IS75X_PROTOCOL_SPI, SS, SC16IS752_CHANNEL_B);
*/

//Connect TX and RX with a wire and run this sketch
//Pin SS should be connected to CS of the module.
void setup() 
{
    //delay(500);
    Serial.begin(9600);
    Serial.println("Start testing");
    // UART to Serial Bridge Initialization
    spiuart.begin(9600);               //baudrate setting
    Serial.println("BAUDRATE SET");
    if (spiuart.ping()!=1) {
        Serial.println("Device not found");
        while(1);
    } else {
        Serial.println("Device found");
    }
	spiuart.pinMode(7, OUTPUT);
    spiuart.digitalWrite(7, LOW);
    Serial.println("Start pin 7 togle");
};

void loop() 
{
 
    spiuart.digitalWrite(7, !spiuart.digitalRead(7)); // togle pin 7
    
    delay(1000);
 
   
};

