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
#define PIN_READ(address, pin_num) (address & (1 << pin_num)) != 0;

// *** INCLUDES ***
// LCD DISPLAY
#include <LiquidCrystal.h>
const int RS = 22, EN = 24, D4 = 3, D5 = 4, D6 = 5, D7 = 6;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
bool displayData;

// STEPPER
#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 39, 41, 43, 45); // 1N1 = 39, 1N4 = 45

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

// Timer Pointers
volatile unsigned char *myTCCR1A  = 0x80;
volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned char *myTCCR1C  = 0x82;
volatile unsigned char *myTIMSK1  = 0x6F;
volatile unsigned char *myTIFR1   = 0x36;
volatile unsigned int  *myTCNT1   = 0x84;

// GPIO
// LEDS - 7: PH4 (RED), 8: PH5 (YELLOW), 9: PH6 (GREEN), 10: PB4 (BLUE)
// BUTTONS - 12: PB6 (STOP), 13: PB7 (RESET)
// FAN CONTROL - 31: PC6 (IN4), 33: PC4 (IN3), 35: PC2 (ON/OFF/SPEED)
// for port Hs
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;
// for port Bs
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;
// for port Cs
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

// Button flags
bool startButton;
bool resetButton;
bool stopButton;

// ISR
const int startButtonPin = 2;
const int interruptNumber = digitalPinToInterrupt(startButtonPin);

class State {
public:
    virtual void enter() = 0; // Called when entering the state
    virtual void update() = 0; // Called in the main loop when in this state
    virtual void exit() = 0;  // Called when exiting the state
};

State* state;
int currentState; // for debugging 
int newState; // int for holding a number representing the next state to switch to
// 0 : stay same
// 1 : RUNNING
// 2 : idle
// 3 : disabled
// 4 : error

// sensor data
unsigned int temp_humid;
unsigned int water_level;
static const unsigned water_threshold = 610; // TEST VALUES
static const unsigned temp_humid_threshold = 770; // TEST VALUES
unsigned int potPos;
unsigned int desiredPos;
unsigned int currentPos;
void moveToPosition();
void control_fan(bool);

class RUNNING : public State {
  void enter() override {
    // LEDS
    WRITE_LOW(*port_h, 4);
    WRITE_LOW(*port_h, 5);
    WRITE_LOW(*port_h, 6);
    WRITE_HIGH(*port_b, 4); // BLUE
    displayData = true;
    currentState = 1;
    control_fan(true);
  }
  void update() override {
    // stepper motor
    moveToPosition();
    if (stopButton){ // stop button
      newState = 3;
    }
    else if (temp_humid < temp_humid_threshold){ // check temp thresh
      newState = 2;
    }
    else if (water_level < water_threshold){ // check water thresh
      newState = 4;
    }
    
  }
  void exit() override {
    // LEDS
    WRITE_LOW(*port_b, 4);
    control_fan(false);
  }
};

class IDLE : public State {
  void enter() override {
    // LEDS
    WRITE_LOW(*port_h, 4);
    WRITE_LOW(*port_h, 5);
    WRITE_HIGH(*port_h, 6); // GREEN
    WRITE_LOW(*port_b, 4);
    displayData = true;
    currentState = 2;
  }
  void update() override {
    // stepper motor
    moveToPosition();
    if (stopButton){ // check stop button
      newState = 3;
    }
    else if (water_level < water_threshold){ // check water thresh
      newState = 4;
    }
    else if (temp_humid > temp_humid_threshold){ // check temp thresh
      newState = 1;
    }
  }
  void exit() override {
    // LEDS
    WRITE_LOW(*port_h, 6);
  }
};

class DISABLED : public State {
  void enter() override {
    // LEDS
    WRITE_LOW(*port_h, 4);
    WRITE_HIGH(*port_h, 5); // YELLOW
    WRITE_LOW(*port_h, 6);
    WRITE_LOW(*port_b, 4);
    displayData = false;
    currentState = 3;
  }
  void update() override {
    // stepper motor 
    moveToPosition();
    if (startButton){ // start button
      newState = 1;
    }
  }
  void exit() override {
    WRITE_LOW(*port_h, 5);
  }
};

class ERROR : public State {
  void enter() override {
    // LEDS
    WRITE_HIGH(*port_h, 4); // RED
    WRITE_LOW(*port_h, 5);
    WRITE_LOW(*port_h, 6);
    WRITE_LOW(*port_b, 4);
    displayData = true;
    currentState = 4;
  }
  void update() override {
    if (stopButton){
      newState = 3; // DISABLED
    }
    if (resetButton){
      newState = 2; // IDLE
    }
  }
  void exit() override {
    WRITE_LOW(*port_h, 4);
  }
};

// Create state instances
RUNNING runningState; // 1
IDLE idleState; // 2
DISABLED disabledState; // 3
ERROR errorState; // 4

void setup()
{
    U0init(9600); // SERIAL IO
    adc_init(); // ADC
    lcd.begin(16, 2); // LCD
    gpio_init(); // GPIO (LEDS and Buttons)
    isr_setup(); // ISR 
    setup_timer_regs(); // TIMER
    stepper_init(); // STEPPER
    state = &disabledState; // STATE, start in disabled mode
    state->enter(); 
}

void loop() 
{
  // READ STUFF
  // Read from the first sensor connected to A0 (channel 0)
  temp_humid = adc_read(0);

  // Read from the second sensor connected to A1 (channel 1)
  water_level = adc_read(1);

  // Read potentiometer output connected to A2 for vent
  potPos = adc_read(2);
  desiredPos = map(potPos, 0, 1023, 0, stepsPerRevolution);

  // DISPLAY STATE
  if (displayData){
    display(water_level, temp_humid);
  }

  // BUTTONS
  // reset flags
  // start button var automatically updated by ISR
  stopButton = PIN_READ(*pin_b, 6);
  resetButton = PIN_READ(*pin_b, 7);

  // UPDATE STATE
  state->update();

  // DEBUGGING
  Serial.print("Current State: ");
  Serial.println(currentState);
  Serial.print("Next State: ");
  Serial.println(newState);
  Serial.print("Start: ");
  Serial.println(startButton);
  Serial.print("Reset: ");
  Serial.println(resetButton);
  Serial.print("Stop: ");
  Serial.println(stopButton);
  Serial.println();

  // change state
  changeState();

  // Delay for one second
  delay(100);
}

void gpio_init()
{
  // LEDs are OUTPUT
  // 7: PH4 (RED), 8: PH5 (YELLOW), 9: PH6 (GREEN), 10: PB4 (BLUE)
  WRITE_HIGH(*ddr_h, 4); // PH4 ddr HIGH (output)
  WRITE_HIGH(*ddr_h, 5); // PH5 ddr HIGH (output)
  WRITE_HIGH(*ddr_h, 6); // PH6 ddr HIGH (output)
  WRITE_HIGH(*ddr_b, 4); // PB4 ddr HIGH (output)

  // INIT TO OFF
  WRITE_LOW(*port_h, 4); // RED
  WRITE_LOW(*port_h, 5); // YELLOW
  WRITE_LOW(*port_h, 6); // GREEN
  WRITE_LOW(*port_b, 4); // BLUE

  // BUTTONS are INPUT (pull-up resistor)
  // BUTTONS - 11: PB5 (), 12: PB6 (), 13: PB7 ()
  // WRITE_LOW(*ddr_b, 5); // START
  WRITE_LOW(*ddr_b, 6); // STOP
  WRITE_LOW(*ddr_b, 7); // RESET

  // INIT PULL UP RESISTOR
  // WRITE_HIGH(*port_b, 5); // START
  WRITE_HIGH(*port_b, 6); // STOP
  WRITE_HIGH(*port_b, 7); // RESET

  // FAN IS OUTPUT
  // FAN CONTROL - 31: PC6 (IN4), 33: PC4 (IN3), 35: PC2 (ON/OFF/SPEED)
  WRITE_HIGH(*ddr_c, 6);
  WRITE_HIGH(*ddr_c, 4);
  WRITE_HIGH(*ddr_c, 2);
}

// ISR setup function
void isr_setup(){
  pinMode(startButtonPin, INPUT_PULLUP);
  attachInterrupt(interruptNumber, handleStartPress, FALLING);
}

// handle start button press
void handleStartPress(){
  startButton = true;
}

// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}

void stepper_init()
{
  myStepper.setSpeed(100000);
}

void control_fan(bool on)
{
  // FAN CONTROL - 31: PC6 (IN4), 33: PC4 (IN3), 35: PC2 (ENB)
  // IN3 HIGH, IN4 LOW -> Forward
  // IN3 LOW, IN4 HIGH -> BACKWARD
  // ENB HIGH -> ON, LOW -> OFF
  if (on){
    WRITE_LOW(*port_c, 6);
    WRITE_HIGH(*port_c, 4);
    WRITE_HIGH(*port_c, 2);
  }
  else{
    WRITE_LOW(*port_c, 2);
  }
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

void changeState(){
  // change state based on newState int
  if (newState == 0){
    // do nothing!
  }
  else if (newState == 1){
    state->exit();
    state = &runningState;
  }
  else if (newState == 2){
    state->exit();
    state = &idleState;
  }
  else if (newState == 3){
    state->exit();
    state = &disabledState;
  }
  else if (newState == 4){
    state->exit();
    state = &errorState;
  }

  // enter the state!
  state->enter();

  // reset flags after state change
  startButton = false;
  stopButton = false;
  resetButton = false;
}

void moveToPosition(){
  int stepsToMove = desiredPos - currentPos;
  myStepper.step(stepsToMove);
  currentPos = desiredPos;
}