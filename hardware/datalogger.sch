EESchema Schematic File Version 4
LIBS:datalogger-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
L MCU_Microchip_ATmega:ATmega328P-AU U1
U 1 1 5C3283BE
P 5250 3000
F 0 "U1" H 5850 1600 50  0000 C CNN
F 1 "ATmega328P-AU" H 6100 1500 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 5250 3000 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 5250 3000 50  0001 C CNN
	1    5250 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2400 6500 2400
Wire Wire Line
	5850 2500 6500 2500
Text Label 6500 2400 2    50   ~ 0
PB6(XTAL1)
Text Label 6500 2500 2    50   ~ 0
PB7(XTAL2)
$Comp
L Device:Crystal Y1
U 1 1 5C3293BA
P 10200 1600
F 0 "Y1" H 10200 1332 50  0000 C CNN
F 1 "8MHz" H 10200 1423 50  0000 C CNN
F 2 "Crystal:Crystal_HC49-U_Vertical" H 10200 1600 50  0001 C CNN
F 3 "~" H 10200 1600 50  0001 C CNN
	1    10200 1600
	-1   0    0    1   
$EndComp
$Comp
L Device:C C6
U 1 1 5C32AD42
P 10350 1750
F 0 "C6" H 10465 1796 50  0000 L CNN
F 1 "22pF" H 10465 1705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 10388 1600 50  0001 C CNN
F 3 "~" H 10350 1750 50  0001 C CNN
	1    10350 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5C32B1B2
P 10050 1750
F 0 "C5" H 9700 1800 50  0000 L CNN
F 1 "22pF" H 9700 1700 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 10088 1600 50  0001 C CNN
F 3 "~" H 10050 1750 50  0001 C CNN
	1    10050 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5C32B864
P 10350 1900
F 0 "#PWR016" H 10350 1650 50  0001 C CNN
F 1 "GND" H 10355 1727 50  0000 C CNN
F 2 "" H 10350 1900 50  0001 C CNN
F 3 "" H 10350 1900 50  0001 C CNN
	1    10350 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5C32B9AB
P 10050 1900
F 0 "#PWR015" H 10050 1650 50  0001 C CNN
F 1 "GND" H 10055 1727 50  0000 C CNN
F 2 "" H 10050 1900 50  0001 C CNN
F 3 "" H 10050 1900 50  0001 C CNN
	1    10050 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 1600 10350 900 
Connection ~ 10350 1600
Wire Wire Line
	10050 1600 10050 900 
Connection ~ 10050 1600
Text Label 10050 900  3    50   ~ 0
PB6(XTAL1)
Text Label 10350 900  3    50   ~ 0
PB7(XTAL2)
Wire Wire Line
	5850 3100 6500 3100
Wire Wire Line
	5850 3200 6500 3200
Text Label 6500 3100 2    50   ~ 0
SDA
Text Label 6500 3200 2    50   ~ 0
SCL
Wire Wire Line
	5850 3500 6500 3500
Wire Wire Line
	5850 3600 6500 3600
Text Label 6500 3500 2    50   ~ 0
RX
Text Label 6500 3600 2    50   ~ 0
TX
$Comp
L power:GND #PWR026
U 1 1 5C32D645
P 5250 4500
F 0 "#PWR026" H 5250 4250 50  0001 C CNN
F 1 "GND" H 5255 4327 50  0000 C CNN
F 2 "" H 5250 4500 50  0001 C CNN
F 3 "" H 5250 4500 50  0001 C CNN
	1    5250 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3300 6500 3300
Text Label 6500 3300 2    50   ~ 0
PC6(RESET)
$Comp
L Connector:Conn_01x06_Male J1
U 1 1 5C333771
P 1300 900
F 0 "J1" V 1100 800 50  0000 C CNN
F 1 "FTDI" V 1200 800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 1300 900 50  0001 C CNN
F 3 "~" H 1300 900 50  0001 C CNN
	1    1300 900 
	0    1    1    0   
$EndComp
Text Label 1200 1100 3    50   ~ 0
RX
Text Label 1100 1100 3    50   ~ 0
TX
Text Label 1000 1100 3    50   ~ 0
DTR
Wire Wire Line
	1300 1100 1300 1350
Wire Wire Line
	1400 1100 1500 1100
Wire Wire Line
	1500 1100 1500 1350
Connection ~ 1500 1100
$Comp
L power:GND #PWR012
U 1 1 5C335DE7
P 1500 1350
F 0 "#PWR012" H 1500 1100 50  0001 C CNN
F 1 "GND" V 1505 1222 50  0000 R CNN
F 2 "" H 1500 1350 50  0001 C CNN
F 3 "" H 1500 1350 50  0001 C CNN
	1    1500 1350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5C336023
P 1300 1350
F 0 "#PWR011" H 1300 1200 50  0001 C CNN
F 1 "+3.3V" V 1315 1478 50  0000 L CNN
F 2 "" H 1300 1350 50  0001 C CNN
F 3 "" H 1300 1350 50  0001 C CNN
	1    1300 1350
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x05_Male J2
U 1 1 5C336CAF
P 2400 900
F 0 "J2" V 2200 850 50  0000 L CNN
F 1 "BH1750" V 2300 750 50  0000 L CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 2400 900 50  0001 C CNN
F 3 "~" H 2400 900 50  0001 C CNN
	1    2400 900 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5C33B373
P 2500 1100
F 0 "#PWR03" H 2500 850 50  0001 C CNN
F 1 "GND" H 2505 927 50  0000 C CNN
F 2 "" H 2500 1100 50  0001 C CNN
F 3 "" H 2500 1100 50  0001 C CNN
	1    2500 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR013
U 1 1 5C33B77A
P 2600 1350
F 0 "#PWR013" H 2600 1200 50  0001 C CNN
F 1 "+3.3V" H 2615 1523 50  0000 C CNN
F 2 "" H 2600 1350 50  0001 C CNN
F 3 "" H 2600 1350 50  0001 C CNN
	1    2600 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	2600 1100 2600 1350
Text Label 2400 1100 3    50   ~ 0
SCL
Text Label 2300 1100 3    50   ~ 0
SDA
NoConn ~ 2200 1100
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5C33C59A
P 3250 900
F 0 "J3" V 3050 800 50  0000 L CNN
F 1 "BME280" V 3150 700 50  0000 L CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 3250 900 50  0001 C CNN
F 3 "~" H 3250 900 50  0001 C CNN
	1    3250 900 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5C33DB90
P 3250 1100
F 0 "#PWR04" H 3250 850 50  0001 C CNN
F 1 "GND" H 3255 927 50  0000 C CNN
F 2 "" H 3250 1100 50  0001 C CNN
F 3 "" H 3250 1100 50  0001 C CNN
	1    3250 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 5C33DB96
P 3350 1350
F 0 "#PWR014" H 3350 1200 50  0001 C CNN
F 1 "+3.3V" H 3365 1523 50  0000 C CNN
F 2 "" H 3350 1350 50  0001 C CNN
F 3 "" H 3350 1350 50  0001 C CNN
	1    3350 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	3350 1100 3350 1350
Text Label 3150 1100 3    50   ~ 0
SCL
Text Label 3050 1100 3    50   ~ 0
SDA
$Comp
L Connector:SD_Card J5
U 1 1 5C33EAF2
P 1800 3250
F 0 "J5" H 1800 2585 50  0000 C CNN
F 1 "SD_Card" H 1800 2676 50  0000 C CNN
F 2 "kicad_libraries:MicroSD_TF02D" H 1800 3250 50  0001 C CNN
F 3 "http://portal.fciconnect.com/Comergent//fci/drawing/10067847.pdf" H 1800 3250 50  0001 C CNN
	1    1800 3250
	-1   0    0    1   
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5C340BC0
P 2350 4750
F 0 "BT1" H 2600 4850 50  0000 C CNN
F 1 "3V" H 2600 4750 50  0000 C CNN
F 2 "Battery:BatteryHolder_Keystone_500" V 2350 4810 50  0001 C CNN
F 3 "~" V 2350 4810 50  0001 C CNN
	1    2350 4750
	1    0    0    -1  
$EndComp
$Comp
L Timer_RTC:DS3231M U2
U 1 1 5C3415C4
P 1750 5550
F 0 "U2" H 2000 5150 50  0000 C CNN
F 1 "DS3231M" H 2000 5050 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm" H 1750 4950 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS3231.pdf" H 2020 5600 50  0001 C CNN
	1    1750 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR037
U 1 1 5C342384
P 1750 5950
F 0 "#PWR037" H 1750 5700 50  0001 C CNN
F 1 "GND" H 1755 5777 50  0000 C CNN
F 2 "" H 1750 5950 50  0001 C CNN
F 3 "" H 1750 5950 50  0001 C CNN
	1    1750 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5350 850  5350
Wire Wire Line
	1250 5450 850  5450
Text Label 850  5350 0    50   ~ 0
SCL
Text Label 850  5450 0    50   ~ 0
SDA
Wire Wire Line
	1750 5150 1750 4550
$Comp
L power:GND #PWR029
U 1 1 5C3470BE
P 2350 4850
F 0 "#PWR029" H 2350 4600 50  0001 C CNN
F 1 "GND" H 2355 4677 50  0000 C CNN
F 2 "" H 2350 4850 50  0001 C CNN
F 3 "" H 2350 4850 50  0001 C CNN
	1    2350 4850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR025
U 1 1 5C348164
P 1650 4400
F 0 "#PWR025" H 1650 4250 50  0001 C CNN
F 1 "+3.3V" H 1665 4573 50  0000 C CNN
F 2 "" H 1650 4400 50  0001 C CNN
F 3 "" H 1650 4400 50  0001 C CNN
	1    1650 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5C348B2C
P 8950 1450
F 0 "R1" H 9020 1496 50  0000 L CNN
F 1 "4k7" H 9020 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8880 1450 50  0001 C CNN
F 3 "~" H 8950 1450 50  0001 C CNN
	1    8950 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5C349690
P 9250 1450
F 0 "R2" H 9320 1496 50  0000 L CNN
F 1 "4k7" H 9320 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9180 1450 50  0001 C CNN
F 3 "~" H 9250 1450 50  0001 C CNN
	1    9250 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5C349AFE
P 8950 1300
F 0 "#PWR09" H 8950 1150 50  0001 C CNN
F 1 "+3.3V" H 8965 1473 50  0000 C CNN
F 2 "" H 8950 1300 50  0001 C CNN
F 3 "" H 8950 1300 50  0001 C CNN
	1    8950 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR010
U 1 1 5C34A02B
P 9250 1300
F 0 "#PWR010" H 9250 1150 50  0001 C CNN
F 1 "+3.3V" H 9265 1473 50  0000 C CNN
F 2 "" H 9250 1300 50  0001 C CNN
F 3 "" H 9250 1300 50  0001 C CNN
	1    9250 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 1600 8950 1950
Wire Wire Line
	9250 1600 9250 1950
Text Label 8950 1950 1    50   ~ 0
SDA
Text Label 9250 1950 1    50   ~ 0
SCL
Wire Wire Line
	1750 4550 2350 4550
Wire Wire Line
	1650 4400 1650 4550
Wire Wire Line
	2700 3050 3050 3050
Wire Wire Line
	3050 3050 3050 3350
Wire Wire Line
	2700 3350 3050 3350
Connection ~ 3050 3350
Wire Wire Line
	3050 3350 3050 3950
$Comp
L power:GND #PWR024
U 1 1 5C340EFB
P 3050 3950
F 0 "#PWR024" H 3050 3700 50  0001 C CNN
F 1 "GND" H 3055 3777 50  0000 C CNN
F 2 "" H 3050 3950 50  0001 C CNN
F 3 "" H 3050 3950 50  0001 C CNN
	1    3050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3250 2800 3250
$Comp
L power:+3.3V #PWR018
U 1 1 5C342287
P 2800 2250
F 0 "#PWR018" H 2800 2100 50  0001 C CNN
F 1 "+3.3V" H 2815 2423 50  0000 C CNN
F 2 "" H 2800 2250 50  0001 C CNN
F 3 "" H 2800 2250 50  0001 C CNN
	1    2800 2250
	1    0    0    -1  
$EndComp
NoConn ~ 2700 3650
NoConn ~ 2700 2850
Wire Wire Line
	2700 3550 3450 3550
Wire Wire Line
	2700 3450 3450 3450
Wire Wire Line
	2700 3150 3450 3150
Wire Wire Line
	2700 2950 3450 2950
Text Label 3450 3550 2    50   ~ 0
SD_CS
Text Label 3450 3150 2    50   ~ 0
SD_SCK
Text Label 3450 3450 2    50   ~ 0
MOSI
Text Label 3450 2950 2    50   ~ 0
MISO
Wire Wire Line
	5850 2300 6500 2300
Wire Wire Line
	5850 2200 6500 2200
Wire Wire Line
	5850 2100 6500 2100
Wire Wire Line
	5850 2000 6500 2000
Text Label 6500 2000 2    50   ~ 0
SD_CS
Text Label 6500 2100 2    50   ~ 0
MOSI
Text Label 6500 2200 2    50   ~ 0
MISO
Text Label 6500 2300 2    50   ~ 0
SD_SCK
Wire Wire Line
	2800 2550 2900 2550
Connection ~ 2800 2550
Wire Wire Line
	2800 2250 2800 2550
Wire Wire Line
	2800 2550 2800 3250
$Comp
L power:GND #PWR019
U 1 1 5C36392F
P 3200 2550
F 0 "#PWR019" H 3200 2300 50  0001 C CNN
F 1 "GND" H 3205 2377 50  0000 C CNN
F 2 "" H 3200 2550 50  0001 C CNN
F 3 "" H 3200 2550 50  0001 C CNN
	1    3200 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5C350B48
P 3050 2550
F 0 "C8" V 3302 2550 50  0000 C CNN
F 1 "100nF" V 3211 2550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3088 2400 50  0001 C CNN
F 3 "~" H 3050 2550 50  0001 C CNN
	1    3050 2550
	0    -1   -1   0   
$EndComp
$Sheet
S 10000 5950 1150 450 
U 5C346344
F0 "powerSupply" 50
F1 "power_supply.sch" 50
$EndSheet
NoConn ~ 900  3050
NoConn ~ 900  3150
NoConn ~ 900  3350
NoConn ~ 900  3450
$Comp
L Device:C C11
U 1 1 5C54598E
P 1300 4700
F 0 "C11" H 1415 4746 50  0000 L CNN
F 1 "1uF" H 1415 4655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1338 4550 50  0001 C CNN
F 3 "~" H 1300 4700 50  0001 C CNN
	1    1300 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5C546342
P 1300 4850
F 0 "#PWR028" H 1300 4600 50  0001 C CNN
F 1 "GND" H 1305 4677 50  0000 C CNN
F 2 "" H 1300 4850 50  0001 C CNN
F 3 "" H 1300 4850 50  0001 C CNN
	1    1300 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 4550 1650 4550
Connection ~ 1650 4550
Wire Wire Line
	1650 4550 1650 5150
NoConn ~ 1250 5750
NoConn ~ 2250 5350
Wire Wire Line
	5800 1000 5500 1000
$Comp
L power:GND #PWR07
U 1 1 5C32EE19
P 5800 1300
F 0 "#PWR07" H 5800 1050 50  0001 C CNN
F 1 "GND" H 5805 1127 50  0000 C CNN
F 2 "" H 5800 1300 50  0001 C CNN
F 3 "" H 5800 1300 50  0001 C CNN
	1    5800 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5C32E233
P 5800 1150
F 0 "C3" H 5685 1104 50  0000 R CNN
F 1 "100nF" H 5685 1195 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5838 1000 50  0001 C CNN
F 3 "~" H 5800 1150 50  0001 C CNN
	1    5800 1150
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 5C32D27E
P 5500 1000
F 0 "#PWR02" H 5500 850 50  0001 C CNN
F 1 "+3.3V" H 5515 1173 50  0000 C CNN
F 2 "" H 5500 1000 50  0001 C CNN
F 3 "" H 5500 1000 50  0001 C CNN
	1    5500 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 1000 4800 1000
$Comp
L power:+3.3V #PWR01
U 1 1 5C74051F
P 5150 1000
F 0 "#PWR01" H 5150 850 50  0001 C CNN
F 1 "+3.3V" H 5165 1173 50  0000 C CNN
F 2 "" H 5150 1000 50  0001 C CNN
F 3 "" H 5150 1000 50  0001 C CNN
	1    5150 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5C740519
P 4800 1150
F 0 "C2" H 4685 1104 50  0000 R CNN
F 1 "100nF" H 4685 1195 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4838 1000 50  0001 C CNN
F 3 "~" H 4800 1150 50  0001 C CNN
	1    4800 1150
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 1000 5150 1300
Wire Wire Line
	5150 1300 5250 1300
Wire Wire Line
	5250 1300 5250 1500
Connection ~ 5150 1000
Wire Wire Line
	5500 1000 5350 1000
Wire Wire Line
	5350 1000 5350 1500
Connection ~ 5500 1000
$Comp
L power:GND #PWR06
U 1 1 5C748DB0
P 4800 1300
F 0 "#PWR06" H 4800 1050 50  0001 C CNN
F 1 "GND" H 4805 1127 50  0000 C CNN
F 2 "" H 4800 1300 50  0001 C CNN
F 3 "" H 4800 1300 50  0001 C CNN
	1    4800 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5C74949C
P 4250 1950
F 0 "C7" H 4365 1996 50  0000 L CNN
F 1 "100nF" H 4365 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4288 1800 50  0001 C CNN
F 3 "~" H 4250 1950 50  0001 C CNN
	1    4250 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1800 4250 1800
$Comp
L power:GND #PWR017
U 1 1 5C74ABCB
P 4250 2100
F 0 "#PWR017" H 4250 1850 50  0001 C CNN
F 1 "GND" H 4255 1927 50  0000 C CNN
F 2 "" H 4250 2100 50  0001 C CNN
F 3 "" H 4250 2100 50  0001 C CNN
	1    4250 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5C7699CB
P 6300 1150
F 0 "C4" H 6415 1196 50  0000 L CNN
F 1 "10uF" H 6415 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6338 1000 50  0001 C CNN
F 3 "~" H 6300 1150 50  0001 C CNN
	1    6300 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5C76A63C
P 4400 1150
F 0 "C1" H 4515 1196 50  0000 L CNN
F 1 "10uF" H 4515 1105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4438 1000 50  0001 C CNN
F 3 "~" H 4400 1150 50  0001 C CNN
	1    4400 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 1000 6300 1000
Connection ~ 5800 1000
Wire Wire Line
	4800 1000 4400 1000
Connection ~ 4800 1000
$Comp
L power:GND #PWR08
U 1 1 5C76E052
P 6300 1300
F 0 "#PWR08" H 6300 1050 50  0001 C CNN
F 1 "GND" H 6305 1127 50  0000 C CNN
F 2 "" H 6300 1300 50  0001 C CNN
F 3 "" H 6300 1300 50  0001 C CNN
	1    6300 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C76E53D
P 4400 1300
F 0 "#PWR05" H 4400 1050 50  0001 C CNN
F 1 "GND" H 4405 1127 50  0000 C CNN
F 2 "" H 4400 1300 50  0001 C CNN
F 3 "" H 4400 1300 50  0001 C CNN
	1    4400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3700 6500 3700
Text Label 6500 3700 2    50   ~ 0
INT0
Wire Wire Line
	2250 5650 2650 5650
Text Label 2650 5650 2    50   ~ 0
INT0
$Comp
L Memory_EEPROM:AT24CS32-SSHM U4
U 1 1 5C9A81B2
P 1750 7200
F 0 "U4" H 1150 7000 50  0000 C CNN
F 1 "AT24CS32-SSHM" H 950 6900 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 1750 7200 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8869-SEEPROM-AT24CS32-Datasheet.pdf" H 1750 7200 50  0001 C CNN
	1    1750 7200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1350 7100 850  7100
Wire Wire Line
	1350 7200 850  7200
Text Label 850  7100 0    50   ~ 0
SDA
Text Label 850  7200 0    50   ~ 0
SCL
$Comp
L power:GND #PWR042
U 1 1 5C9B00ED
P 1750 7500
F 0 "#PWR042" H 1750 7250 50  0001 C CNN
F 1 "GND" H 1755 7327 50  0000 C CNN
F 2 "" H 1750 7500 50  0001 C CNN
F 3 "" H 1750 7500 50  0001 C CNN
	1    1750 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 7100 2150 7200
Wire Wire Line
	2150 7500 1750 7500
Connection ~ 2150 7200
Wire Wire Line
	2150 7200 2150 7300
Connection ~ 2150 7300
Wire Wire Line
	2150 7300 2150 7500
Connection ~ 1750 7500
Wire Wire Line
	1350 7300 1350 7500
Wire Wire Line
	1350 7500 1750 7500
$Comp
L power:+3.3V #PWR038
U 1 1 5C9B4798
P 1750 6550
F 0 "#PWR038" H 1750 6400 50  0001 C CNN
F 1 "+3.3V" H 1765 6723 50  0000 C CNN
F 2 "" H 1750 6550 50  0001 C CNN
F 3 "" H 1750 6550 50  0001 C CNN
	1    1750 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6550 1750 6900
$Comp
L Device:C C14
U 1 1 5C9BD42C
P 2150 6700
F 0 "C14" H 2265 6746 50  0000 L CNN
F 1 "100nF" H 2265 6655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2188 6550 50  0001 C CNN
F 3 "~" H 2150 6700 50  0001 C CNN
	1    2150 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6550 2150 6550
Connection ~ 1750 6550
Wire Wire Line
	2150 6850 2150 7100
Connection ~ 2150 7100
Wire Wire Line
	5850 2700 6500 2700
Wire Wire Line
	5850 2800 6500 2800
$Comp
L Connector:AVR-ISP-6 J?
U 1 1 5CA2A4AE
P 10300 3200
AR Path="/5CA26526/5CA2A4AE" Ref="J?"  Part="1" 
AR Path="/5CA2A4AE" Ref="J4"  Part="1" 
F 0 "J4" H 10021 3296 50  0000 R CNN
F 1 "AVR-ISP-6" H 10021 3205 50  0000 R CNN
F 2 "Connector_IDC:IDC-Header_2x03_P2.54mm_Vertical" V 10050 3250 50  0001 C CNN
F 3 " ~" H 9025 2650 50  0001 C CNN
	1    10300 3200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CA2A4B4
P 10400 3600
AR Path="/5CA26526/5CA2A4B4" Ref="#PWR?"  Part="1" 
AR Path="/5CA2A4B4" Ref="#PWR022"  Part="1" 
F 0 "#PWR022" H 10400 3350 50  0001 C CNN
F 1 "GND" H 10405 3427 50  0000 C CNN
F 2 "" H 10400 3600 50  0001 C CNN
F 3 "" H 10400 3600 50  0001 C CNN
	1    10400 3600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9900 3000 9550 3000
Wire Wire Line
	9900 3100 9550 3100
Wire Wire Line
	9900 3200 9550 3200
Text Label 9550 3000 0    50   ~ 0
PB4(MISO)
Text Label 9550 3100 0    50   ~ 0
PB3(MOSI)
Text Label 9550 3200 0    50   ~ 0
PB5(SCK)
$Comp
L Device:R R?
U 1 1 5CA2A4C1
P 9250 2950
AR Path="/5CA26526/5CA2A4C1" Ref="R?"  Part="1" 
AR Path="/5CA2A4C1" Ref="R3"  Part="1" 
F 0 "R3" H 9320 2996 50  0000 L CNN
F 1 "10k" H 9320 2905 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 9180 2950 50  0001 C CNN
F 3 "~" H 9250 2950 50  0001 C CNN
	1    9250 2950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9250 2700 9250 2800
$Comp
L Device:D D?
U 1 1 5CA2A4CA
P 8800 2950
AR Path="/5CA26526/5CA2A4CA" Ref="D?"  Part="1" 
AR Path="/5CA2A4CA" Ref="D1"  Part="1" 
F 0 "D1" V 8754 3029 50  0000 L CNN
F 1 "1N4148" V 8845 3029 50  0000 L CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 8800 2950 50  0001 C CNN
F 3 "~" H 8800 2950 50  0001 C CNN
	1    8800 2950
	0    -1   1    0   
$EndComp
Wire Wire Line
	8800 3100 8800 3300
$Comp
L power:GND #PWR?
U 1 1 5CA2A4D6
P 9250 4550
AR Path="/5CA26526/5CA2A4D6" Ref="#PWR?"  Part="1" 
AR Path="/5CA2A4D6" Ref="#PWR027"  Part="1" 
F 0 "#PWR027" H 9250 4300 50  0001 C CNN
F 1 "GND" H 9255 4377 50  0000 C CNN
F 2 "" H 9250 4550 50  0001 C CNN
F 3 "" H 9250 4550 50  0001 C CNN
	1    9250 4550
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CA2A4DC
P 9650 3600
AR Path="/5CA26526/5CA2A4DC" Ref="C?"  Part="1" 
AR Path="/5CA2A4DC" Ref="C10"  Part="1" 
F 0 "C10" H 9765 3646 50  0000 L CNN
F 1 "100nF" H 9765 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 9688 3450 50  0001 C CNN
F 3 "~" H 9650 3600 50  0001 C CNN
	1    9650 3600
	1    0    0    1   
$EndComp
Wire Wire Line
	9650 4000 9650 3750
Text Label 9650 4000 1    50   ~ 0
DTR
$Comp
L power:+3.3V #PWR?
U 1 1 5CA2A4E4
P 9250 2700
AR Path="/5CA26526/5CA2A4E4" Ref="#PWR?"  Part="1" 
AR Path="/5CA2A4E4" Ref="#PWR020"  Part="1" 
F 0 "#PWR020" H 9250 2550 50  0001 C CNN
F 1 "+3.3V" H 9265 2873 50  0000 C CNN
F 2 "" H 9250 2700 50  0001 C CNN
F 3 "" H 9250 2700 50  0001 C CNN
	1    9250 2700
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5CA2A4EA
P 10400 2700
AR Path="/5CA26526/5CA2A4EA" Ref="#PWR?"  Part="1" 
AR Path="/5CA2A4EA" Ref="#PWR021"  Part="1" 
F 0 "#PWR021" H 10400 2550 50  0001 C CNN
F 1 "+3.3V" H 10415 2873 50  0000 C CNN
F 2 "" H 10400 2700 50  0001 C CNN
F 3 "" H 10400 2700 50  0001 C CNN
	1    10400 2700
	-1   0    0    -1  
$EndComp
Text Label 8200 3300 0    50   ~ 0
PC6(RESET)
$Comp
L Device:R R?
U 1 1 5CA2A4F1
P 9250 3700
AR Path="/5CA26526/5CA2A4F1" Ref="R?"  Part="1" 
AR Path="/5CA2A4F1" Ref="R4"  Part="1" 
F 0 "R4" H 9320 3746 50  0000 L CNN
F 1 "330" H 9320 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9180 3700 50  0001 C CNN
F 3 "~" H 9250 3700 50  0001 C CNN
	1    9250 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3850 9250 4050
Wire Wire Line
	9250 4450 9250 4550
$Comp
L Device:C C?
U 1 1 5CA2A4F9
P 8800 3600
AR Path="/5CA26526/5CA2A4F9" Ref="C?"  Part="1" 
AR Path="/5CA2A4F9" Ref="C9"  Part="1" 
F 0 "C9" H 8915 3646 50  0000 L CNN
F 1 "100nF" H 8915 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8838 3450 50  0001 C CNN
F 3 "~" H 8800 3600 50  0001 C CNN
	1    8800 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 3450 9650 3300
Wire Wire Line
	8800 3300 8800 3450
$Comp
L power:GND #PWR?
U 1 1 5CA2A504
P 8800 3750
AR Path="/5CA26526/5CA2A504" Ref="#PWR?"  Part="1" 
AR Path="/5CA2A504" Ref="#PWR023"  Part="1" 
F 0 "#PWR023" H 8800 3500 50  0001 C CNN
F 1 "GND" H 8805 3577 50  0000 C CNN
F 2 "" H 8800 3750 50  0001 C CNN
F 3 "" H 8800 3750 50  0001 C CNN
	1    8800 3750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 5CA2A50A
P 9250 4250
AR Path="/5CA26526/5CA2A50A" Ref="SW?"  Part="1" 
AR Path="/5CA2A50A" Ref="SW1"  Part="1" 
F 0 "SW1" V 9204 4398 50  0000 L CNN
F 1 "SW_Push" V 9295 4398 50  0000 L CNN
F 2 "Button_Switch_THT:SW_Tactile_SKHH_Angled" H 9250 4450 50  0001 C CNN
F 3 "~" H 9250 4450 50  0001 C CNN
	1    9250 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	8800 2800 9250 2800
Connection ~ 9250 2800
Wire Wire Line
	9250 3100 9250 3300
Connection ~ 9650 3300
Wire Wire Line
	9650 3300 9900 3300
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5CA9AB81
P 10700 5500
F 0 "H3" H 10800 5503 50  0000 L CNN
F 1 "H" H 10800 5458 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 10700 5500 50  0001 C CNN
F 3 "~" H 10700 5500 50  0001 C CNN
	1    10700 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR035
U 1 1 5CA9BCB2
P 10700 5600
F 0 "#PWR035" H 10700 5350 50  0001 C CNN
F 1 "GND" H 10705 5427 50  0000 C CNN
F 2 "" H 10700 5600 50  0001 C CNN
F 3 "" H 10700 5600 50  0001 C CNN
	1    10700 5600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5CA9CD3A
P 11000 5500
F 0 "H4" H 11100 5503 50  0000 L CNN
F 1 "H" H 11100 5458 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 11000 5500 50  0001 C CNN
F 3 "~" H 11000 5500 50  0001 C CNN
	1    11000 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 5CA9CD40
P 11000 5600
F 0 "#PWR036" H 11000 5350 50  0001 C CNN
F 1 "GND" H 11005 5427 50  0000 C CNN
F 2 "" H 11000 5600 50  0001 C CNN
F 3 "" H 11000 5600 50  0001 C CNN
	1    11000 5600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5CA9F75A
P 10400 5500
F 0 "H2" H 10500 5503 50  0000 L CNN
F 1 "H" H 10500 5458 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 10400 5500 50  0001 C CNN
F 3 "~" H 10400 5500 50  0001 C CNN
	1    10400 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5CA9F760
P 10400 5600
F 0 "#PWR034" H 10400 5350 50  0001 C CNN
F 1 "GND" H 10405 5427 50  0000 C CNN
F 2 "" H 10400 5600 50  0001 C CNN
F 3 "" H 10400 5600 50  0001 C CNN
	1    10400 5600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5CAA2056
P 10100 5500
F 0 "H1" H 10200 5503 50  0000 L CNN
F 1 "H" H 10200 5458 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad_Via" H 10100 5500 50  0001 C CNN
F 3 "~" H 10100 5500 50  0001 C CNN
	1    10100 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5CAA205C
P 10100 5600
F 0 "#PWR033" H 10100 5350 50  0001 C CNN
F 1 "GND" H 10105 5427 50  0000 C CNN
F 2 "" H 10100 5600 50  0001 C CNN
F 3 "" H 10100 5600 50  0001 C CNN
	1    10100 5600
	1    0    0    -1  
$EndComp
Text GLabel 6500 2800 2    50   Input ~ 0
ENABLE
Text GLabel 6500 2700 2    50   Input ~ 0
ADC
Connection ~ 9250 3300
Wire Wire Line
	9250 3300 9250 3550
Wire Wire Line
	9250 3300 9650 3300
Connection ~ 8800 3300
Wire Wire Line
	8800 3300 9250 3300
Wire Wire Line
	8200 3300 8800 3300
$Comp
L Interface_USB:CH340G U3
U 1 1 5CA5231A
P 4100 6150
F 0 "U3" H 4400 5500 50  0000 C CNN
F 1 "CH340G" H 4500 5400 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 4150 5600 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 3750 6950 50  0001 C CNN
	1    4100 6150
	1    0    0    -1  
$EndComp
Text GLabel 3700 6050 0    50   Input ~ 0
UD+
Text GLabel 3700 6150 0    50   Input ~ 0
UD-
$Comp
L Device:Crystal Y2
U 1 1 5CA54C4D
P 3450 6800
F 0 "Y2" H 3450 7068 50  0000 C CNN
F 1 "12MHz" H 3450 6977 50  0000 C CNN
F 2 "Crystal:Crystal_HC49-U_Vertical" H 3450 6800 50  0001 C CNN
F 3 "~" H 3450 6800 50  0001 C CNN
	1    3450 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 6550 3600 6550
Wire Wire Line
	3600 6550 3600 6800
Connection ~ 3600 6800
Wire Wire Line
	3600 6800 3600 6950
Wire Wire Line
	3700 6350 3300 6350
Wire Wire Line
	3300 6350 3300 6800
Connection ~ 3300 6800
Wire Wire Line
	3300 6800 3300 6950
$Comp
L Device:C C15
U 1 1 5CA5A86E
P 3300 7100
F 0 "C15" H 3000 7150 50  0000 L CNN
F 1 "22pF" H 3000 7050 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3338 6950 50  0001 C CNN
F 3 "~" H 3300 7100 50  0001 C CNN
	1    3300 7100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5CA5B493
P 3600 7100
F 0 "C16" H 3715 7146 50  0000 L CNN
F 1 "22pF" H 3715 7055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3638 6950 50  0001 C CNN
F 3 "~" H 3600 7100 50  0001 C CNN
	1    3600 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 5CA5C148
P 3600 7250
F 0 "#PWR041" H 3600 7000 50  0001 C CNN
F 1 "GND" H 3605 7077 50  0000 C CNN
F 2 "" H 3600 7250 50  0001 C CNN
F 3 "" H 3600 7250 50  0001 C CNN
	1    3600 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR040
U 1 1 5CA5C765
P 3300 7250
F 0 "#PWR040" H 3300 7000 50  0001 C CNN
F 1 "GND" H 3305 7077 50  0000 C CNN
F 2 "" H 3300 7250 50  0001 C CNN
F 3 "" H 3300 7250 50  0001 C CNN
	1    3300 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR039
U 1 1 5CA5CD8E
P 4100 6750
F 0 "#PWR039" H 4100 6500 50  0001 C CNN
F 1 "GND" H 4105 6577 50  0000 C CNN
F 2 "" H 4100 6750 50  0001 C CNN
F 3 "" H 4100 6750 50  0001 C CNN
	1    4100 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 5CA5F6EB
P 4550 5400
F 0 "#PWR032" H 4550 5150 50  0001 C CNN
F 1 "GND" H 4555 5227 50  0000 C CNN
F 2 "" H 4550 5400 50  0001 C CNN
F 3 "" H 4550 5400 50  0001 C CNN
	1    4550 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR030
U 1 1 5CA5F6F7
P 4100 5100
F 0 "#PWR030" H 4100 4950 50  0001 C CNN
F 1 "+3.3V" H 4115 5273 50  0000 C CNN
F 2 "" H 4100 5100 50  0001 C CNN
F 3 "" H 4100 5100 50  0001 C CNN
	1    4100 5100
	1    0    0    -1  
$EndComp
Connection ~ 4100 5100
Wire Wire Line
	4100 5550 4100 5100
Wire Wire Line
	4100 5100 4550 5100
Wire Wire Line
	4000 5550 4000 5450
Wire Wire Line
	4000 5450 3600 5450
Wire Wire Line
	3600 5450 3600 5100
$Comp
L Device:R R5
U 1 1 5CA6A5A6
P 3850 5100
F 0 "R5" V 3643 5100 50  0000 C CNN
F 1 "0R" V 3734 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3780 5100 50  0001 C CNN
F 3 "~" H 3850 5100 50  0001 C CNN
	1    3850 5100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR031
U 1 1 5CA78219
P 3250 5400
F 0 "#PWR031" H 3250 5150 50  0001 C CNN
F 1 "GND" H 3255 5227 50  0000 C CNN
F 2 "" H 3250 5400 50  0001 C CNN
F 3 "" H 3250 5400 50  0001 C CNN
	1    3250 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5CA7821F
P 3250 5250
F 0 "C12" H 3135 5204 50  0000 R CNN
F 1 "100nF" H 3135 5295 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3288 5100 50  0001 C CNN
F 3 "~" H 3250 5250 50  0001 C CNN
	1    3250 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 5100 3600 5100
Connection ~ 3600 5100
Wire Wire Line
	3600 5100 3700 5100
Wire Wire Line
	4000 5100 4100 5100
$Comp
L Device:C C13
U 1 1 5CA5F6F1
P 4550 5250
F 0 "C13" H 4435 5204 50  0000 R CNN
F 1 "100nF" H 4435 5295 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4588 5100 50  0001 C CNN
F 3 "~" H 4550 5250 50  0001 C CNN
	1    4550 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	4900 5750 5200 5750
Wire Wire Line
	4900 6000 5200 6000
Text Label 5200 5750 2    50   ~ 0
RX
Text Label 5200 6000 2    50   ~ 0
TX
$Comp
L Device:R R6
U 1 1 5CA885ED
P 4750 5750
F 0 "R6" V 4650 5750 50  0000 C CNN
F 1 "1k" V 4650 5900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4680 5750 50  0001 C CNN
F 3 "~" H 4750 5750 50  0001 C CNN
	1    4750 5750
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5CA8BEAB
P 4750 6000
F 0 "R7" V 4650 6000 50  0000 C CNN
F 1 "1k" V 4650 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4680 6000 50  0001 C CNN
F 3 "~" H 4750 6000 50  0001 C CNN
	1    4750 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 5750 4500 5750
Wire Wire Line
	4500 5850 4550 5850
Wire Wire Line
	4550 5850 4550 6000
Wire Wire Line
	4550 6000 4600 6000
$EndSCHEMATC
