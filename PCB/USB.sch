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
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23-6" H 2800 2850 60  0001 C CNN
F 3 "" H 2800 2850 60  0000 C CNN
F 4 "1295310" H 2800 2850 60  0001 C CNN "Farnell"
	1    2800 2850
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
L GND #PWR46
U 1 1 56395BD6
P 2400 2850
F 0 "#PWR46" H 2400 2850 30  0001 C CNN
F 1 "GND" H 2400 2780 30  0001 C CNN
F 2 "" H 2400 2850 60  0000 C CNN
F 3 "" H 2400 2850 60  0000 C CNN
	1    2400 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR45
U 1 1 56395C0E
P 2300 3150
F 0 "#PWR45" H 2300 3150 30  0001 C CNN
F 1 "GND" H 2300 3080 30  0001 C CNN
F 2 "" H 2300 3150 60  0000 C CNN
F 3 "" H 2300 3150 60  0000 C CNN
	1    2300 3150
	1    0    0    -1  
$EndComp
NoConn ~ 2200 3000
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
$Comp
L USB_OTG P6
U 1 1 56492413
P 1900 2900
F 0 "P6" H 2225 2775 50  0000 C CNN
F 1 "USB_OTG" H 1900 3100 50  0000 C CNN
F 2 "Connect:USB_Mini-B" V 1850 2800 60  0001 C CNN
F 3 "" V 1850 2800 60  0000 C CNN
	1    1900 2900
	0    -1   1    0   
$EndComp
NoConn ~ 1800 3300
$EndSCHEMATC
