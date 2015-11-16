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
Sheet 5 7
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
L ADIS16405BMLZ U2
U 1 1 563882BB
P 3150 3400
F 0 "U2" H 4750 3800 60  0000 C CNN
F 1 "ADIS16405BMLZ" H 4750 3700 60  0000 C CNN
F 2 "IMU:ADIS16405BMLZ" H 4750 3640 60  0001 C CNN
F 3 "" H 3150 3400 60  0000 C CNN
	1    3150 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR41
U 1 1 56490925
P 6600 4250
F 0 "#PWR41" H 6600 4000 50  0001 C CNN
F 1 "GND" H 6600 4100 50  0000 C CNN
F 2 "" H 6600 4250 60  0000 C CNN
F 3 "" H 6600 4250 60  0000 C CNN
	1    6600 4250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR42
U 1 1 56490962
P 6600 4750
F 0 "#PWR42" H 6600 4600 50  0001 C CNN
F 1 "+5V" H 6600 4890 50  0000 C CNN
F 2 "" H 6600 4750 60  0000 C CNN
F 3 "" H 6600 4750 60  0000 C CNN
	1    6600 4750
	1    0    0    -1  
$EndComp
Text GLabel 2950 4400 0    60   Input ~ 0
~RST
Text HLabel 2950 4200 0    60   Input ~ 0
SCLK
Text HLabel 2950 4100 0    60   Input ~ 0
MISO
Text HLabel 2950 4000 0    60   Input ~ 0
MOSI
Text HLabel 2950 3900 0    60   Input ~ 0
~SS
Text HLabel 2950 3400 0    60   Input ~ 0
IRQ
NoConn ~ 3150 3600
NoConn ~ 3150 3700
NoConn ~ 6350 3900
Wire Wire Line
	6350 4200 6600 4200
Wire Wire Line
	6450 4000 6450 4400
Wire Wire Line
	6450 4400 6350 4400
Wire Wire Line
	6350 4300 6450 4300
Connection ~ 6450 4300
Wire Wire Line
	6600 4200 6600 4250
Connection ~ 6450 4200
Wire Wire Line
	6350 4600 6450 4600
Wire Wire Line
	6450 4600 6450 4800
Wire Wire Line
	6350 4800 6600 4800
Wire Wire Line
	6600 4800 6600 4750
Connection ~ 6450 4800
Wire Wire Line
	6350 4700 6450 4700
Connection ~ 6450 4700
Wire Wire Line
	2950 4400 3150 4400
Wire Wire Line
	2950 4200 3150 4200
Wire Wire Line
	2950 4100 3150 4100
Wire Wire Line
	3150 4000 2950 4000
Wire Wire Line
	2950 3900 3150 3900
Wire Wire Line
	2950 3400 3150 3400
Wire Wire Line
	6350 4000 6450 4000
NoConn ~ 3150 3500
$EndSCHEMATC
