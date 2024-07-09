#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

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

int main(void) {
  // Set up
  uint8_t val = 0;
  i2c_init(); 
  
  // Loop
  while (1) {
    i2c_start(); 
    i2c_write(0x2c << 1);
    i2c_write(0x00); 
    i2c_write(val); // Send potentiometer value byte
    i2c_stop(); 

    val++; // Increment value
    if (val == 64) { // If reached maximum position
      val = 0; // Start over from lowest value
    }
    _delay_ms(500); 
  }
}