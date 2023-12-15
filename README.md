# CPE301-Final-Project-Fall23
Repository for CPE301 Final Project Swamp Cooler Code 

## Author 
Anthony Silva, 12/15/23, CPE301, UNR

## Description
This repository contains the code for my automated evaporation cooling system. In dry, hot climates such as Reno, Nevada, evaporation cooling systems provide an energy efficient alterative to air conditioners. They work by pulling air into the contrpation through a water soaked pad. The water in the pad evaporates and this cools and humidifies the air. 

### Materials
It works with the 2560MEGA Arduino along with the following hardware included in "The Most Complete Starter Kit Mega Project" by ELEGO
- Arduino (https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf)
- Water Level Detection Sensor Module (https://www.datasheethub.com/arduino-water-level-sensor/)
- Stepper Motor (https://components101.com/motors/28byj-48-stepper-motor)
- LCD1602 Module (with pin header) (https://www.waveshare.com/datasheet/LCD_en_PDF/LCD1602.pdf)
- DHT11 Temperature and Humidity Module (https://components101.com/sensors/dht11-temperature-sensor)
- Fan Blade and 3-6V Motor (https://thecustomizewindows.com/2015/08/arduino-3v-dc-motor-control-transistor-ic-more/)
- Power Supply Module (https://components101.com/modules/5v-mb102-breadboard-power-supply-module)
- L293D (https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/)
- DS1307 RTC (https://www.analog.com/media/en/technical-documentation/data-sheets/ds1307.pdf)
- ULN2003 Stepper Driver (https://www.hadex.cz/spec/m513.pdf)

### Design Overview
This project entails implementing a swamp cooler utilizing an Arduino, lcd, LEDs, buttons, a potentiometer, a water level sensor, a temperature and humidity sensor, a fan motor and L293D driver, a stepper motor and ULN2003 driver, a power supply module, and a DS1307 RTC.

Sensors

The sensors of this project are the water level sensor and the temperature and humidity sensor. These are wired to 5V and ground along with the Arduino through its onboard analog inputs. These sensors are measured once per minute to judge if a state change needs to occur

LEDs

There are four LEDs wired to digital pins on the Arduino, as well as to ground with resistors. These pins are blue, green, yellow, and red to show which state the machine is in. 

Buttons

There are three buttons wired to the Arduino digital pins, 5V and ground with resistors. These are setup to be pull up resistor buttons and represent the Start, Stop, and Restart buttons. They are coded in the Arduino to provide valid input depending on the state of the machine. 

Potentiometer

This potentiometer serves as the input to the stepper motor. It has a 5V and a ground connection and a data connection to an analog input on the Arduino. This analog input is computed into a stepper position that the stepper motor is coded to adhere to. 

Motors and Fan

The motors require a larger power draw than the arduino can supply, which is why the power supply module is present. This module provides an external source of power for the motors to run. The fan motor has its power connected to the external supply and is then connected to the L293D driver on its relevant pinouts. This driver is then connected to the Arduino through digital pin connections for control. The stepper motor is connected to external power as well as its ULN2003 driver. This driver is connected to digital pins on the Arduino for control. The stepper motor is coded in the Arduino to respond to movements in the potentiometer when in a valid state. The fan motor is coded to run on HIGH whenever it is in the RUNNING state. 

LCD

The LCD is connected to 5V, ground, and the Arduino. There is no potentiometer connected to the LCD, it is just connected to HIGH for maximum backlight difference. The LCD is run in 4 bit mode and connected to digital pins on the Arduino. The LCD is made to display the sensor readings when in the appropriate state. If it is not displaying the sensor readings, then it is displaying nothing. 

RTC

The Real Time Clock (RTC) is connected to ground and power along with the Arduino’s SDA and SCL pins. This clock is used to compute internal timers within the code such as the one minute delay in taking sensor readings, the 50ms main loop delay, and the 500ms button check delay. This clock is paired with an ISR in the code to count up to various time intervals to perform these time checks. 

### System Constraints
Water Threshold: 700

When a damp rag was placed on the water sensor, it would go above 700. When it was removed, it would go below 700. This was a good value to test going between ERROR and other states. 

Temperature and Humidity Threshold: 780

The temperature and humidity sensor was inversely related to the temperature of the environment it seemed. When heated up, the sensor would decrease roughly below 780. When cooled, it would increase above 780. This was a good value to test going between RUNNING and IDLE. 

Power Constraints

The motors required a bigger power draw than what the Arduino could supply, causing a design restraint in requiring an extra power supply added to the circuit. This power supply provided a separate and reliable 5V connection for the motors and their drivers. 

State constraints

The following state constraints were put in place to both produce a proper state machine for the swamp cooler to operate through as well as utilize concepts learned in CPE301 throughout the semester. 

- DISABLED: 
1. Yellow LED -> ON, rest OFF
2. No water temperature monitoring
3. Monitor Start button via an ISR
- IDLE:
1. Exact time stamp (via real time clock module) should record transition times
2. Water level continously monitored; change state to ERROR if water level is too low.
3. Green LED -> ON, rest OFF
- ERROR:
1. Motor OFF
2. Monitor RESET Button, change to IDLE if water level is above threshold
3. Error message displayed on LCD
4. Red LED -> ON, rest OFF
- RUNNING:
1. Fan Motor ON
2. Move to IDLE if temp drops below threshold
3. Move to ERROR if water drops below threshold
4. Blue LED -> ON, rest OFF
- All States:
1. Realtime clock must report (via Serial Port) the time of each state transition, as well as any changes to the stepper motor position for the vent
- All States besides DISABLED:
1. Humidity and temperature should be continuously monitored and reported on LDC screen. Update once per minute. 
2. System should respond to changes in vent position contorl
3. Stop button sould turn fan motor OFF (if ON) and turn system to DISABLED