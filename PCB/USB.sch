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
LIBS:swd
LIBS:IMU-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 7
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
L USBLC6-2 D8
U 1 1 56395866
P 2800 2850
F 0 "D8" H 2650 3050 60  0000 C CNN
F 1 "USBLC6-2" H 2850 2650 60  0000 C CNN
F 2 "" H 2800 2850 60  0000 C CNN
F 3 "" H 2800 2850 60  0000 C CNN
F 4 "1295310" H 2800 2850 60  0001 C CNN "Farnell"
	1    2800 2850
	1    0    0    -1  
$EndComp
$Comp
L MICROUSB USB1
U 1 1 56395885
P 1900 3100
F 0 "USB1" H 1750 3650 60  0000 C CNN
F 1 "MICROUSB" H 1850 2550 60  0000 C CNN
F 2 "" H 1900 3100 60  0000 C CNN
F 3 "" H 1900 3100 60  0000 C CNN
F 4 "1521869" H 1900 3100 60  0001 C CNN "Farnell"
	1    1900 3100
	1    0    0    -1  
$EndComp
Text HLabel 4300 1900 2    60   Output ~ 0
USB5V
Wire Wire Line
	3200 2750 3600 2750
Wire Wire Line
	3200 2850 3400 2850
Wire Wire Line
	3200 2950 3600 2950
Wire Wire Line
	2200 2700 2200 2550
Wire Wire Line
	2200 2550 2600 2550
Wire Wire Line
	2200 2800 2200 2750
Wire Wire Line
	2200 2750 2450 2750
Wire Wire Line
	2200 2900 2200 2950
Wire Wire Line
	2200 2950 2450 2950
$Comp
L GND #PWR047
U 1 1 56395BD6
P 2400 2850
F 0 "#PWR047" H 2400 2850 30  0001 C CNN
F 1 "GND" H 2400 2780 30  0001 C CNN
F 2 "" H 2400 2850 60  0000 C CNN
F 3 "" H 2400 2850 60  0000 C CNN
	1    2400 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR048
U 1 1 56395C0E
P 2300 3150
F 0 "#PWR048" H 2300 3150 30  0001 C CNN
F 1 "GND" H 2300 3080 30  0001 C CNN
F 2 "" H 2300 3150 60  0000 C CNN
F 3 "" H 2300 3150 60  0000 C CNN
	1    2300 3150
	1    0    0    -1  
$EndComp
NoConn ~ 2200 3000
NoConn ~ 2200 3250
NoConn ~ 2200 3350
NoConn ~ 2200 3450
NoConn ~ 2200 3550
Wire Wire Line
	2400 2850 2450 2850
Wire Wire Line
	4300 1900 4000 1900
Text Label 4000 1900 0    60   ~ 0
VBUS
Text Label 2600 2550 2    60   ~ 0
VBUS
Text Label 3400 2850 2    60   ~ 0
VBUS
Text GLabel 3600 2750 2    60   Input ~ 0
OTG_FS_DM
Text GLabel 3600 2950 2    60   Input ~ 0
OTG_FS_DP
Wire Wire Line
	2200 3100 2300 3100
Wire Wire Line
	2300 3100 2300 3150
$EndSCHEMATC
