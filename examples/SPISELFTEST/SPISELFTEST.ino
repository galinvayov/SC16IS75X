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
    Serial.println("Start serial communication");
};

void loop() 
{
 
    spiuart.write(0x55);
    delay(10);
    if (spiuart.available()==0) {
        Serial.println("Please connnect TX and RX with a wire and reset your Arduino");
        while(1);
    }        
    if (spiuart.read()!=0x55) {
        Serial.println("Serial communication error");
        while(1);
    }   
    delay(200);
    
    spiuart.write(0xAA);
    delay(10);
    if (spiuart.available()==0) {
        Serial.println("Please connnect TX and RX with a wire and reset your Arduino");
        while(1);
    }  
    if (spiuart.read()!=0xAA) {
        Serial.println("Serial communication error");
        while(1);
    }   
    
    delay(200);
 
 
  
};

