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
Sheet 8 8
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
L USBLC6-2 D?
U 1 1 56395866
P 2800 2850
F 0 "D?" H 2650 3050 60  0000 C CNN
F 1 "USBLC6-2" H 2850 2650 60  0000 C CNN
F 2 "" H 2800 2850 60  0000 C CNN
F 3 "" H 2800 2850 60  0000 C CNN
F 4 "1295310" H 2800 2850 60  0001 C CNN "Farnell"
	1    2800 2850
	1    0    0    -1  
$EndComp
$Comp
L MICROUSB USB?
U 1 1 56395885
P 1900 3100
F 0 "USB?" H 1750 3650 60  0000 C CNN
F 1 "MICROUSB" H 1850 2550 60  0000 C CNN
F 2 "" H 1900 3100 60  0000 C CNN
F 3 "" H 1900 3100 60  0000 C CNN
F 4 "1521869" H 1900 3100 60  0001 C CNN "Farnell"
	1    1900 3100
	1    0    0    -1  
$EndComp
Text HLabel 2300 2550 2    60   Output ~ 0
USB5V
Text HLabel 3400 2750 2    60   Output ~ 0
USB_DM
Text HLabel 3400 2950 2    60   Output ~ 0
USB_DP
Text HLabel 3400 2850 2    60   Output ~ 0
USB5V
Wire Wire Line
	3200 2750 3400 2750
Wire Wire Line
	3200 2850 3400 2850
Wire Wire Line
	3200 2950 3400 2950
Wire Wire Line
	2200 2700 2200 2550
Wire Wire Line
	2200 2550 2300 2550
Wire Wire Line
	2200 2800 2200 2750
Wire Wire Line
	2200 2750 2450 2750
Wire Wire Line
	2200 2900 2200 2950
Wire Wire Line
	2200 2950 2450 2950
$Comp
L GND #PWR?
U 1 1 56395BD6
P 2400 2850
F 0 "#PWR?" H 2400 2850 30  0001 C CNN
F 1 "GND" H 2400 2780 30  0001 C CNN
F 2 "" H 2400 2850 60  0000 C CNN
F 3 "" H 2400 2850 60  0000 C CNN
	1    2400 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 56395C0E
P 2200 3100
F 0 "#PWR?" H 2200 3100 30  0001 C CNN
F 1 "GND" H 2200 3030 30  0001 C CNN
F 2 "" H 2200 3100 60  0000 C CNN
F 3 "" H 2200 3100 60  0000 C CNN
	1    2200 3100
	0    -1   -1   0   
$EndComp
NoConn ~ 2200 3000
NoConn ~ 2200 3250
NoConn ~ 2200 3350
NoConn ~ 2200 3450
NoConn ~ 2200 3550
Wire Wire Line
	2400 2850 2450 2850
$EndSCHEMATC
