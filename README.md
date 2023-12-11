# CPE301-Final-Project-Fall23
Repository for CPE301 Final Project Swamp Cooler Code 

## Description
This repository contains the code for my automated evaporation cooling system. In dry, hot climates such as Reno, Nevada, evaporation cooling systems provide an energy efficient alterative to air conditioners. They work by pulling air into the contrpation through a water soaked pad. The water in the pad evaporates and this cools and humidifies the air. 

### Materials
It works with an Arduino along with the following hardware included in "The Most Complete Starter Kit Mega Project" by ELEGO
- Water Level Detection Sensor Module (https://www.datasheethub.com/arduino-water-level-sensor/)
- Stepper Motor (https://components101.com/motors/28byj-48-stepper-motor)
- LCD1602 Module (with pin header) (https://www.waveshare.com/datasheet/LCD_en_PDF/LCD1602.pdf)
- DHT11 Temperature and Humidity Module (https://components101.com/sensors/dht11-temperature-sensor)
- Fan Blade and 3-6V Motor (https://thecustomizewindows.com/2015/08/arduino-3v-dc-motor-control-transistor-ic-more/)

### States
This cooler works as a state machine switching between the following states:
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
2. Monitor RESET Button via ISR, change to IDLE if water level is above threshold
3. Error message displayed on LCD
4. Red LED -> ON, rest OFF
- RUNNING:
1. Fan Motor ONN
2. Move to IDLE if temp drops below threshold
3. Move to ERROR if water drops below threshold
4. Blue LED -> ON, rest OFF
- All States:
1. Realtime clock must report (via Serial Port) the time of each state transition, as well as any changes to the stepper motor position for the vent
- All States besides DISABLED:
1. Humidity and temperature should be continuously monitored and reported on LDC screen. Update once per minute. 
2. System should respond to changes in vent position contorl
3. Stop button sould turn fan motor OFF (if ON) and turn system to DISABLED

### Design Overview
lorem ipsum or whatever it says

### System Constraints
filler placeholder

### Author
Anthony Silva, 12/11/23