EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32f405vgt
LIBS:adp3335
LIBS:buzzer
LIBS:q_nmos_gsd
LIBS:ADIS16405
LIBS:uSD_holder
LIBS:ms5611-01ba03
LIBS:microusb
LIBS:usblc6-2
LIBS:IMU-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 8
Title ""
Date "28 oct 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F405VGT IC1
U 1 1 56315E6F
P 2000 1000
F 0 "IC1" H 3100 1000 60  0000 C CNN
F 1 "STM32F405VGT" H 2450 1000 60  0000 C CNN
F 2 "IMU:LQFP100" H 2000 1000 60  0001 C CNN
F 3 "" H 2000 1000 60  0000 C CNN
	1    2000 1000
	1    0    0    -1  
$EndComp
$Sheet
S 5100 1150 1350 650 
U 56316C38
F0 "SDCard" 60
F1 "SDCard.sch" 60
F2 "Test" I L 5100 1400 60 
$EndSheet
$Sheet
S 7300 1150 1500 600 
U 56316CB8
F0 "StatusIndicators" 60
F1 "StatusIndicators.sch" 60
F2 "Test" I L 7300 1400 60 
$EndSheet
$Sheet
S 5100 2150 1350 600 
U 56316D3B
F0 "Power" 60
F1 "Power.sch" 60
F2 "Test" I L 5100 2350 60 
$EndSheet
$Sheet
S 5100 3100 1350 600 
U 56316E47
F0 "IntertialSensors" 60
F1 "InertialSensors.sch" 60
F2 "Test" I L 5100 3350 60 
$EndSheet
$Sheet
S 7300 3100 1450 550 
U 56316E4B
F0 "Altimeter" 60
F1 "Altimeter.sch" 60
F2 "Test" I L 7300 3350 60 
$EndSheet
$Sheet
S 7300 2150 1350 600 
U 56316E4F
F0 "USB" 60
F1 "USB.sch" 60
F2 "Test" I L 7300 2400 60 
$EndSheet
$EndSCHEMATC
