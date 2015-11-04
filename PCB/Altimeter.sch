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
Sheet 7 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MS5611-01BA03 IC?
U 1 1 56392CA0
P 2450 1900
F 0 "IC?" H 2300 2100 60  0000 C CNN
F 1 "MS5611-01BA03" H 2450 1600 60  0000 C CNN
F 2 "" H 2450 1900 60  0000 C CNN
F 3 "" H 2450 1900 60  0000 C CNN
	1    2450 1900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5639302C
P 1900 1900
F 0 "C?" H 1750 1950 50  0000 L CNN
F 1 "100n" H 1650 1850 50  0000 L CNN
F 2 "" H 1900 1900 60  0000 C CNN
F 3 "" H 1900 1900 60  0000 C CNN
	1    1900 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56393055
P 1900 2150
F 0 "#PWR?" H 1900 1900 50  0001 C CNN
F 1 "GND" H 1900 2000 50  0001 C CNN
F 2 "" H 1900 2150 60  0000 C CNN
F 3 "" H 1900 2150 60  0000 C CNN
	1    1900 2150
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5639306D
P 1900 1650
F 0 "#PWR?" H 1900 1500 50  0001 C CNN
F 1 "+3.3V" H 1900 1790 50  0000 C CNN
F 2 "" H 1900 1650 60  0000 C CNN
F 3 "" H 1900 1650 60  0000 C CNN
	1    1900 1650
	1    0    0    -1  
$EndComp
Text HLabel 3000 1800 2    60   Input ~ 0
ALT_SCLK
Text HLabel 3000 1900 2    60   Input ~ 0
ALT_SDI
Text HLabel 3000 2000 2    60   Input ~ 0
ALT_SDO
Text HLabel 3000 2100 2    60   Input ~ 0
ALT_CSB
Wire Wire Line
	2000 2000 2050 2000
Wire Wire Line
	2050 2000 2100 2000
Wire Wire Line
	2100 2100 2050 2100
Wire Wire Line
	2050 2100 2050 2350
Wire Wire Line
	2050 2350 2850 2350
Wire Wire Line
	2850 2350 2850 2100
Wire Wire Line
	2800 2100 2850 2100
Wire Wire Line
	2850 2100 3000 2100
Connection ~ 2850 2100
Wire Wire Line
	2800 2000 3000 2000
Wire Wire Line
	2800 1900 3000 1900
Wire Wire Line
	2800 1800 3000 1800
Wire Wire Line
	2100 1900 2050 1900
Wire Wire Line
	2050 1900 2050 2000
Connection ~ 2050 2000
Wire Wire Line
	2100 1800 2000 1800
Wire Wire Line
	2000 2000 2000 2100
Wire Wire Line
	2000 2100 1900 2100
Wire Wire Line
	2000 1800 2000 1700
Wire Wire Line
	2000 1700 1900 1700
Wire Wire Line
	1900 1650 1900 1700
Wire Wire Line
	1900 1700 1900 1750
Wire Wire Line
	1900 2050 1900 2100
Wire Wire Line
	1900 2100 1900 2150
Connection ~ 1900 1700
Connection ~ 1900 2100
$EndSCHEMATC
