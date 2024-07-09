#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define display_ADDRESS 0x3C
#define display_COMMAND 0x00
#define display_DATA 0x40

#define SCL_PIN PC5
#define SDA_PIN PC4

#define READ_PIN A0

uint8_t buffer[1024];
double ox, oy;
bool Redraw4 = true;
double x = 0, y = 0;

void setup() {
    DDRC |= (1 << SCL_PIN) | (1 << SDA_PIN);

    // Initialize I2C and display
    i2c_init();
    display_init();

    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

    // Clear the display
    display_clear();
    display_show();
}

void loop() {
    ADCSRA |= (1 << ADSC); 
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    uint16_t adc_value = ADC;

    double bvolts = adc_value;
    x += 1;

    DrawCGraph(x, bvolts, 30, 50, 75, 30, 0, 100, 25, 0, 1024, 512, 0, 
    "Bits vs Seconds", Redraw4);

    if (x > 100) {
        while (1) {}
    }

    _delay_ms(1000);
}

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

void display_command(uint8_t command) {
    i2c_start();
    i2c_write(display_ADDRESS << 1);
    i2c_write(display_COMMAND);
    i2c_write(command);
    i2c_stop();
}

void display_init(void) {
    _delay_ms(100);  
    display_command(0xAE); // Display OFF
    display_command(0x20); // Set Memory Addressing Mode
    display_command(0x00); // Horizontal Addressing Mode
    display_command(0xB0); // Set Page Start Address for Page Addressing Mode
    display_command(0xC8); 
    display_command(0x00); // Set low column address
    display_command(0x10); // Set high column address
    display_command(0x40); // Set start line address
    display_command(0x81); // Set contrast control 
    display_command(0xFF); // Max contrast
    display_command(0xA1); 
    display_command(0xA6); // Set Normal Display
    display_command(0xA8); // Set Multiplex Ratio
    display_command(0x3F); 
    display_command(0xA4); // Entire Display ON, Output RAM to Display
    display_command(0xD3); // Set Display Offset
    display_command(0x00); // No offset
    display_command(0xD5); 
    display_command(0xF0); // Highest frequency
    display_command(0xD9); // Set Pre-charge Period
    display_command(0x22); // Pre-charge period
    display_command(0xDA); // Set COM Pins Hardware Configuration
    display_command(0x12);
    display_command(0xDB);
    display_command(0x20); 
    display_command(0x8D); // Set Charge Pump
    display_command(0x14);
    display_command(0xAF); // Display ON
}

void display_clear(void) {
    // Clear the display buffer
    for (int i = 0; i < 1024; i++) {
        buffer[i] = 0x00;
    }
}

void display_show(void) {
    // Send the display buffer content to the OLED display
    for (uint8_t page = 0; page < 8; page++) {
        display_command(0xB0 + page); 
        display_command(0x00);      
        display_command(0x10);        
        i2c_start();                   
        i2c_write(display_ADDRESS << 1); 
        i2c_write(display_DATA);          
        for (uint8_t col = 0; col < 128; col++) {
            i2c_write(buffer[ col ]);   // Display buffer 128
        }
        i2c_stop(); 
    }
}

void display_draw_pixel(int x, int y, bool color) {
    if (x >= 0 && x < 128 && y >= 0 && y < 64) {
        if (color) {
            buffer[x + (y / 8) * 128] |= (1 << (y % 8)); // Set pixel on
        } else {
            buffer[x + (y / 8) * 128] &= ~(1 << (y % 8)); // Set pixel off
        }
    }
}

void DrawCGraph(double x, double y, double gx, double gy, double w, double h, 
double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, 
double dig, const char* title, bool& Redraw) {

    double i;
    double temp;

    if (Redraw) {
        Redraw = false;

        // Draw the title
        for (int i = 0; title[i] != '\0'; i++) {
            display_draw_pixel(i * 6, 0, 1); 
        }

        // Calculate initial display coordinates
        ox = (x - xlo) * (w) / (xhi - xlo) + gx;
        oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

        // Draw y scale
        for (i = ylo; i <= yhi; i += yinc) {
            temp = (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
            if (i == 0) {
                for (int j = gx - 3; j < gx + w + 3; j++) {
                    display_draw_pixel(j, temp, 1); 
                }
            } else {
                for (int j = gx - 3; j < gx; j++) {
                    display_draw_pixel(j, temp, 1);
                }
            }
        }

        // Draw x scale
        for (i = xlo; i <= xhi; i += xinc) {
            temp = (i - xlo) * (w) / (xhi - xlo) + gx;
            if (i == 0) {
                for (int j = gy - h; j < gy + 3; j++) {
                    display_draw_pixel(temp, j, 1); 
                }
            } else {
                for (int j = gy; j < gy + 3; j++) {
                    display_draw_pixel(temp, j, 1);
                }
            }
        }
    }

    // Update current point and draw it on the display
    x = (x - xlo) * (w) / (xhi - xlo) + gx;
    y = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    display_draw_pixel(ox, oy, 1); 
    display_draw_pixel(x, y, 1);  
    ox = x; 
    oy = y;
    display_show(); 
}

// To execute loop function continuously
int main(void) {
    setup(); 
    while (1) {
        loop(); 
    }
    return 0;
}
