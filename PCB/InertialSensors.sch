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
LIBS:cga0402mlc-12g
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
L GND #PWR045
U 1 1 56490925
P 7000 3700
F 0 "#PWR045" H 7000 3450 50  0001 C CNN
F 1 "GND" H 7000 3550 50  0000 C CNN
F 2 "" H 7000 3700 60  0000 C CNN
F 3 "" H 7000 3700 60  0000 C CNN
	1    7000 3700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR046
U 1 1 56490962
P 7000 4200
F 0 "#PWR046" H 7000 4050 50  0001 C CNN
F 1 "+5V" H 7000 4340 50  0000 C CNN
F 2 "" H 7000 4200 60  0000 C CNN
F 3 "" H 7000 4200 60  0000 C CNN
	1    7000 4200
	1    0    0    -1  
$EndComp
Text GLabel 5250 3850 0    60   Input ~ 0
~RST
Text HLabel 5250 3650 0    60   Input ~ 0
SCLK
Text HLabel 5250 3550 0    60   Input ~ 0
MISO
Text HLabel 5250 3450 0    60   Input ~ 0
MOSI
Text HLabel 5250 3350 0    60   Input ~ 0
~SS
Text HLabel 5250 2850 0    60   Input ~ 0
IRQ
NoConn ~ 5450 3050
NoConn ~ 5450 3150
NoConn ~ 6750 3350
Wire Wire Line
	6750 3650 7000 3650
Wire Wire Line
	6850 3450 6850 3850
Wire Wire Line
	6850 3850 6750 3850
Wire Wire Line
	6750 3750 6850 3750
Connection ~ 6850 3750
Wire Wire Line
	7000 3650 7000 3700
Connection ~ 6850 3650
Wire Wire Line
	6750 4050 6850 4050
Wire Wire Line
	6850 4050 6850 4250
Wire Wire Line
	6750 4250 7000 4250
Wire Wire Line
	7000 4250 7000 4200
Connection ~ 6850 4250
Wire Wire Line
	6750 4150 6850 4150
Connection ~ 6850 4150
Wire Wire Line
	5250 3850 5450 3850
Wire Wire Line
	5250 3650 5450 3650
Wire Wire Line
	5250 3550 5450 3550
Wire Wire Line
	5450 3450 5250 3450
Wire Wire Line
	5250 3350 5450 3350
Wire Wire Line
	5250 2850 5450 2850
Wire Wire Line
	6750 3450 6850 3450
NoConn ~ 5450 2950
$Comp
L ADIS16405BMLZ U2
U 1 1 5649D269
P 6100 3600
AR Path="/5649D269" Ref="U2"  Part="1" 
AR Path="/56316E47/5649D269" Ref="U2"  Part="1" 
F 0 "U2" H 6100 2650 60  0000 C CNN
F 1 "ADIS16405BMLZ" H 6100 4550 60  0000 C CNN
F 2 "IMU:ADIS16405BMLZ" H 6100 3500 60  0001 C CNN
F 3 "" H 5450 4350 60  0000 C CNN
F 4 "1849515" H 6100 3600 60  0001 C CNN "Farnell"
F 5 "FTMH-112-03-L-DV" H 6100 3600 60  0001 C CNN "DigiKey"
	1    6100 3600
	1    0    0    -1  
$EndComp
Text Notes 5400 4850 0    60   ~ 0
Connector is FTMH-112-03-L-DV\n(Farnell: 1885921)
$EndSCHEMATC
