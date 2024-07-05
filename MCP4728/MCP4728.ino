const uint8_t LDAC = PD5; 
const uint8_t MCP4728_ADDRESS = 0x60; // I2C address of MCP4728

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
} while (!(TWCR & (1 << TWINT)));
}

void setup() {
  DDRD |= (1 << LDAC); // Set LDAC pin (PD7) as output
  PORTD |= (1 << PD7); // Set LDAC pin high 

  i2c_init(); 

  // Set the DAC output voltage to 0.01V on channel A
  // Calculate the corresponding 12 bit value for 0.01V
  uint16_t value = 8; // 0.01V * 4095 / 5V = 8.19 

  i2c_start();
  i2c_write((MCP4728_ADDRESS << 1) | 0); 
  i2c_write((0 << 1) | ((value >> 8) & 0x0F)); // Upper 4 bits
  i2c_write(value & 0xFF); // Lower 8 bits
  i2c_stop();
}

int main(void){
  setup();
  return 0;
}
