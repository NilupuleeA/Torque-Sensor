#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL  
#define MCP4728_ADDR 0x60 // MCP4728 I2C address

void i2c_init(void) {
  TWSR = 0x00; 
  TWBR = ((F_CPU / 100000UL) - 16) / 2; // Set the TWI frequency to 100kHz
}

void i2c_start(void) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); 
  while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); 
  while (TWCR & (1 << TWSTO));
}

void i2c_write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN); 
  while (!(TWCR & (1 << TWINT)));
}

void mcp4728_set_output(uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = 0b01011110;           // Command byte for fast write to channel D
    buffer[1] = (value >> 8) & 0xFF;  // High byte of DAC value
    buffer[2] = value & 0xFF;         // Low byte of DAC value

    i2c_start();
    i2c_write(MCP4728_ADDR << 1);     // Send MCP4728 address with write bit
    for (uint8_t i = 0; i < 3; i++) {
        i2c_write(buffer[i]);         // Send control byte and DAC value bytes
    }
    i2c_stop();
}

int main(void) {
    i2c_init();    
    mcp4728_set_output(2048); // Set MCP4728 DAC output to 512

    while (1) {
        _delay_ms(1000); 
    }
    
    return 0;
}
