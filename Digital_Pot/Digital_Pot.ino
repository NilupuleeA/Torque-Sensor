#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL  // Define CPU frequency (16 MHz for ATmega328P)
#define SCL_CLOCK 100000L // Define I2C clock speed (100 kHz)

void i2c_init(void) {
    // Set SCL frequency
    TWSR = 0x00; // Set prescaler to 1
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2; // Set bit rate register
    // Enable TWI
    TWCR = (1<<TWEN);
}

void i2c_start(void) {
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Send start condition
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // Send stop condition
    while (TWCR & (1<<TWSTO)); // Wait for stop condition to be executed
}

void i2c_write(uint8_t data) {
    TWDR = data; // Load data into TWDR register
    TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
    while (!(TWCR & (1<<TWINT))); // Wait for TWINT flag set
}

int main(void) {
    uint8_t val = 0;
    i2c_init(); // Initialize I2C
    
    while (1) {
        i2c_start(); // Start I2C communication
        i2c_write(0x88); // Write device address (0x44 << 1 for write operation)
        i2c_write(0x00); // Send instruction byte
        i2c_write(val); // Send potentiometer value byte
        i2c_stop(); // Stop I2C communication

        val++; // Increment value
        if (val == 64) { // If reached 64th position (max)
            val = 0; // Start over from lowest value
        }
        _delay_ms(500); // Delay 500 ms
    }
}