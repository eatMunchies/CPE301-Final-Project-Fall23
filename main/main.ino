// Author: Anthony Silva
// Class: CPE301
// Assignment: Final Project - Evaporation Cooler via Arduino and Friends!
// Starting: 12/11/23
// Due Date: 12/15/23

// *** MACROS ***
#define RDA 0x80
#define TBE 0x20
// for pin manipulation!
#define WRITE_HIGH(address, pin_num)  address |= (0x01 << pin_num);
#define WRITE_LOW(address, pin_num)  address &= ~(0x01 << pin_num);

// *** INCLUDES ***
// LCD DISPLAY
#include <LiquidCrystal.h>
const int RS = 22, EN = 2, D4 = 3, D5 = 4, D6 = 5, D7 = 6;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// *** REGISTERS ***
// SERIAL TRANSMISSION
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
// ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// LED Digital pins
// 7: PH4 (RED), 8: PH5 (YELLOW), 9: PH6 (GREEN), 10: PB4 (BLUE)
// for port Hs
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;
// for port Bs
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

class State {
public:
    virtual void enter() = 0; // Called when entering the state
    virtual void update() = 0; // Called in the main loop when in this state
    virtual void exit() = 0;  // Called when exiting the state
};

class RUNNING : public State {
  void enter() override {

  }
  void update() override {

  }
  void exit() override {

  }
};

class IDLE : public State {
  void enter() override {

  }
  void update() override {

  }
  void exit() override {
    
  }
};

class DISABLED : public State {
  void enter() override {

  }
  void update() override {

  }
  void exit() override {
    
  }
};

class ERROR : public State {
  void enter() override {

  }
  void update() override {

  }
  void exit() override {
    
  }
};

void setup()
{
    U0init(9600);
    adc_init();
    lcd.begin(16, 2);
    led_init(); 
}

void loop() 
{
  // Read from the first sensor connected to A0 (channel 0)
  unsigned int temp_humid = adc_read(0);

  // Read from the second sensor connected to A1 (channel 1)
  unsigned int water_level = adc_read(1);
  
  display(water_level, temp_humid);

  // LED TEST
  WRITE_HIGH(*port_h, 4);
  WRITE_HIGH(*port_h, 5);
  WRITE_HIGH(*port_h, 6);
  WRITE_HIGH(*port_b, 4);


  // Add a delay if necessary
  delay(1000); // Delay for a second for example
}

void led_init()
{
  // ALL OUTPUT
  // 7: PH4 (RED), 8: PH5 (YELLOW), 9: PH6 (GREEN), 10: PB4 (BLUE)
  WRITE_HIGH(*ddr_h, 4); // PH4 ddr HIGH (output)
  WRITE_HIGH(*ddr_h, 5); // PH5 ddr HIGH (output)
  WRITE_HIGH(*ddr_h, 6); // PH6 ddr HIGH (output)
  WRITE_HIGH(*ddr_b, 4); // PB4 ddr HIGH (output)

}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void display(int water_level, int temp_humid){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water: ");
  lcd.print(water_level);
  lcd.setCursor(0, 1);
  lcd.print("T/H: ");
  lcd.print(temp_humid);
}